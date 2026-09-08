#!/usr/bin/env python3
"""Numeric-only response measurement through the existing controller socket.

Default is read-only. --probe yaw:1:2.5 applies a six-second bounded fixed-angle
trial in Manual; multiple --probe arguments run in order. No images are read.
Capture to ignored run/, then archive off the station. No motor/camera ownership.
"""
import argparse
import json
import io
import math
from pathlib import Path
import select
import socket
import time


class Capture:
    def __init__(self, sockpath, output, timing):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,512*1024)
        self.sock.settimeout(1)
        self.sock.connect(sockpath)
        self.output = output
        self.timing = Path(timing) if timing else None
        self.state = {}
        self.last_trace = self.last_camera = 0
        self.next_trace = 0
        self.probe_spans = {}

    def emit(self, kind, **data):
        self.output.write(json.dumps(dict(kind=kind,host_ns=time.monotonic_ns(),**data),
                                     separators=(',',':'))+'\n')

    def send(self, command, arg=''):
        self.emit('command',command=command,arg=arg)
        self.sock.send(json.dumps(dict(command=command,arg=arg)).encode())

    def pump(self):
        if self.output.tell() > 96*1024*1024:
            raise RuntimeError('96 MiB capture bound reached')
        now = time.monotonic()
        if now >= self.next_trace:
            self.sock.send(b'{"command":"read_control_trace"}')
            self.next_trace = now+.4
            if self.timing and self.timing.exists():
                camera = json.loads(self.timing.read_text())
                if camera['frame_sequence'] != self.last_camera:
                    self.emit('camera',data=camera)
                    self.last_camera = camera['frame_sequence']
        if select.select([self.sock],[],[],.04)[0]:
            packet, _, flags, _ = self.sock.recvmsg(256*1024)
            if not packet or flags & socket.MSG_TRUNC:
                raise RuntimeError('closed or truncated controller packet')
            message = json.loads(packet)
            if message.get('type') == 'control_trace':
                for row in message['rows']:
                    if row['t'] > self.last_trace:
                        self.emit('trace',**row)
                        self.last_trace = row['t']
                        if row.get('omega',0)>0:
                            span=self.probe_spans.setdefault(row['ack'],[row['t'],row['t'],True])
                            span[1]=row['t']
                            span[2]=span[2] and row['safety']==0
            elif message.get('type') == 'telemetry':
                self.state = message
                self.emit('state',data=message)
            else:
                self.emit('response',data=message)

    def healthy(self, origin=None):
        s = self.state
        if (s.get('phase') != 'hold' or s.get('operating_mode') != 'MANUAL' or
            s.get('fault') or s.get('safety_action') != 'ALLOW' or
            not s.get('soft_limits_valid') or s.get('feedback_age_ms',999) > 50 or
            time.monotonic_ns()-s.get('ts_ns',0) > 300_000_000):
            raise RuntimeError('station health/mode/freshness gate failed')
        if origin and any(abs(s[f'q_{axis}_rad']-origin[f'q_{axis}_rad']) > math.radians(7)
                          for axis in ('pitch','yaw')):
            raise RuntimeError('seven degree excursion guard')
        return s.copy()

    def dwell(self, seconds, check=False, origin=None):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            self.pump()
            if check:
                self.healthy(origin)

    def trial(self, arg):
        self.dwell(2,check=True)
        # A fixed pause was insufficient for some loaded-axis corrections.
        # Wait for the controller's own near-zero command gate without weakening it.
        end=time.monotonic()+20
        stationary_since=None
        while time.monotonic()<end:
            self.pump()
            s=self.healthy()
            quiet=all(abs(s.get(f'service_command_rate_{axis}_rad_s',1)) < math.radians(.15)
                      for axis in ('pitch','yaw'))
            stationary_since=(stationary_since or time.monotonic()) if quiet else None
            if stationary_since and time.monotonic()-stationary_since >= .5:
                break
        else:
            raise RuntimeError('station did not settle to the existing probe command gate in 20 seconds')
        origin = self.healthy()
        before = origin.get('cmd_ack_seq',0)
        self.emit('trial_start',arg=arg,origin=origin)
        self.send('response_probe',arg)
        end = time.monotonic()+2
        while self.state.get('cmd_ack_seq',0) <= before and time.monotonic() < end:
            self.pump()
        if (self.state.get('cmd_ack_seq',0) <= before or
            self.state.get('cmd_ack_command') != 'response_probe' or
            not self.state.get('cmd_ack_accepted')):
            raise RuntimeError('controller rejected or did not acknowledge trial: '+
                               self.state.get('cmd_ack_reason',''))
        self.emit('trial_ack',arg=arg,state=self.state)
        self.dwell(7,check=True,origin=origin)
        span=self.probe_spans.get(before+1,[0,0,False])
        valid=span[2] and span[1]-span[0]>=5_800_000_000
        self.emit('trial_end',arg=arg,state=self.state,valid_full_trial=valid)
        print(json.dumps(dict(trial=arg,valid_full_trial=valid,
                              active_seconds=(span[1]-span[0])/1e9)),flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--socket',default='/tmp/ota-stack-1000/control-web.sock')
    p.add_argument('--timing',default='/tmp/ota-stack-1000/perception/timing.json')
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--seconds',type=float,default=10)
    p.add_argument('--probe',action='append',default=[])
    args = p.parse_args()
    if not 0 < args.seconds <= 60 or len(args.probe) > 32:
        p.error('captures are bounded to 60 s or 32 six-second trials')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    # SD-card stalls must not delay the observer while it supervises a trial.
    # Keep this bounded capture in RAM, then persist after Stop Motion.
    with io.StringIO() as output:
        capture = Capture(args.socket,output,args.timing)
        try:
            capture.dwell(1)
            if args.probe:
                for trial in args.probe:
                    capture.trial(trial)
            else:
                capture.dwell(args.seconds)
            capture.emit('complete',state=capture.state)
            print(json.dumps(dict(complete=True,path=str(args.output),
                                 bytes=output.tell(),trials=len(args.probe))),flush=True)
        except BaseException as exc:
            capture.emit('error',error=str(exc),state=capture.state)
            raise
        finally:
            try:
                if args.probe:
                    capture.send('stop_motion')
                    capture.dwell(1)
            finally:
                capture.sock.close()
                args.output.write_text(output.getvalue(),encoding='utf-8')


if __name__ == '__main__':
    main()
