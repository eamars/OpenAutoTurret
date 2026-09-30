"""Run acquisition in a local ARM64 Linux VM, with no network or device passthrough.

Requires separately prepared Debian amd64 (QEMU) and arm64 (kernel, BusyBox,
Python/venv, runtime libraries) package roots. Does not install packages, use
privilege escalation, contact a station, or relax acquisition checks.
"""
import argparse
import gzip
import io
import json
import hashlib
import lzma
import os
from pathlib import Path
import stat
import subprocess
import tarfile

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--phase',choices=['probe','matrix','current-probe','current-matrix',
                                      'characterization-probe','characterization-matrix',
                                      'homing-probe','homing-matrix'],required=True)
parser.add_argument('--packages-root',type=Path,required=True,help='separate amd64/ and arm64/ extracted package directories')
parser.add_argument('--build',type=Path,required=True,help='full ARM64 Firmware CMake build')
parser.add_argument('--output',type=Path,required=True,help='new local directory for immutable VM evidence')
args=parser.parse_args()
tools=Path(__file__).resolve().parent
vm=args.packages_root.resolve(strict=True)
build=args.build.resolve(strict=True)
out=args.output.resolve();out.mkdir(exist_ok=False)
source=vm/'arm64';host=vm/'amd64'
entries={}
def add(name,data,mode=stat.S_IFREG|0o644):
    for parent in Path(name).parents:
        if str(parent)!='.': entries.setdefault(str(parent),(b'',stat.S_IFDIR|0o755))
    entries[name]=(data,mode)
def file(path,name):
    if not path.exists() and not path.is_symlink():raise FileNotFoundError(path)
    if path.is_symlink():add(name,os.readlink(path).encode(),stat.S_IFLNK|0o777)
    elif path.is_file():add(name,path.read_bytes(),path.stat().st_mode)
    elif path.is_dir():
        add(name,b'',stat.S_IFDIR|0o755)
        for child in sorted(path.iterdir()): file(child,name+'/'+child.name)
file(source/'usr/bin/busybox','usr/bin/busybox')
file(source/'usr/bin/python3.13','usr/bin/python3.13')
file(source/'usr/lib/python3.13','usr/lib/python3.13')
for lib in sorted((source/'usr/lib/aarch64-linux-gnu').glob('*.so*')):
    file(lib,'usr/lib/aarch64-linux-gnu/'+lib.name)
for name,target in [('lib','usr/lib'),('bin','usr/bin'),('sbin','usr/bin'),
                    ('usr/lib/ld-linux-aarch64.so.1','aarch64-linux-gnu/ld-linux-aarch64.so.1'),
                    ('usr/bin/python3','python3.13'),('usr/bin/sh','busybox')]:
    add(name,target.encode(),stat.S_IFLNK|0o777)
for name in ('proc','sys','dev','tmp','work/evidence'):
    add(name,b'',stat.S_IFDIR|0o755)
for path,name in [('axis_control_core/commissiond','commissiond'),
                  ('commission_runtime/probe-commission-capture','probe-commission-capture'),
                  ('commission_runtime/test_commission_capture','test_commission_capture')]:
    binary=build/path
    if binary.read_bytes()[:4]!=b'\x7fELF' or binary.read_bytes()[18:20]!=b'\xb7\x00':
        raise ValueError(f'ARM64 ELF required: {binary}')
    file(binary,'work/bin/'+name)
for name in ('adr0022_capture_rehearsal.py','adr0022_capture_review.py',
             'adr0022_current_rehearsal.py','adr0022_current_review.py',
             'adr0022_homing_rehearsal.py','adr0022_homing_review.py'):
    file(tools/name,'work/tools/'+name)
file(tools/'adr0022_arm_vm_guest.py','work/run_vm_rehearsals.py')
kernel=next((source/'boot').glob('vmlinuz-*'))
release=kernel.name.removeprefix('vmlinuz-')
module=source/'usr/lib/modules'/release/'kernel/drivers/virtio/virtio_mmio.ko.xz'
add('work/virtio_mmio.ko',lzma.decompress(module.read_bytes()))
init=f'''#!/bin/sh
export PATH=/usr/bin
export PYTHONUTF8=1
export LD_BIND_NOW=1
busybox mount -t proc proc /proc
busybox mount -t sysfs sysfs /sys
busybox mount -t devtmpfs devtmpfs /dev
busybox chmod 1777 /tmp
busybox ip link set lo up
busybox insmod /work/virtio_mmio.ko || exit 1
echo VM_PROGRESS booted
python3.13 -m venv --without-pip /work/.venv
rc=$?
if [ ! -c /dev/vport0p1 ]; then rc=1; echo VM_ERROR evidence-port-missing; fi
if [ "$rc" = 0 ]; then
  /work/.venv/bin/python /work/run_vm_rehearsals.py {args.phase}
  rc=$?
fi
busybox tar -C /work/evidence -cf /tmp/evidence.tar .
archive_status=$?
if [ "$archive_status" = 0 ]; then
  busybox gzip /tmp/evidence.tar
  archive_status=$?
fi
if [ "$archive_status" = 0 ]; then
  busybox cat /tmp/evidence.tar.gz > /dev/vport0p1
  archive_status=$?
fi
if [ "$archive_status" != 0 ]; then rc=1; fi
echo VM_EXIT $rc
busybox poweroff -f
'''
add('init',init.encode(),stat.S_IFREG|0o755)
archive=io.BytesIO()
def entry(name,data,mode,index):
    encoded=name.encode()+b'\0'
    fields=[index,mode,0,0,1,0,len(data),0,0,0,0,len(encoded),0]
    header=b'070701'+b''.join(f'{v:08x}'.encode() for v in fields)
    archive.write(header+encoded)
    archive.write(b'\0'*((-(110+len(encoded)))%4))
    archive.write(data);archive.write(b'\0'*((-len(data))%4))
for index,(name,(data,mode)) in enumerate(sorted(entries.items()),1):entry(name,data,mode,index)
entry('TRAILER!!!',b'',0,len(entries)+1)
archive.write(b'\0'*((-archive.tell())%512))
initrd=out/'initrd.gz'
with gzip.open(initrd,'wb',compresslevel=3) as target:target.write(archive.getvalue())
command=[str(host/'usr/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2'),'--library-path',str(host/'usr/lib/x86_64-linux-gnu'),
         str(host/'usr/bin/qemu-system-aarch64'),'-L',str(host/'usr/share/qemu'),
         '-machine','virt','-cpu','cortex-a76','-accel','tcg,thread=multi','-smp','2','-m','2048',
         '-display','none','-serial','stdio','-monitor','none','-no-reboot','-nic','none',
         '-chardev',f'file,id=evidence,path={out}/evidence.tar.gz',
         '-device','virtio-serial-device','-device','virtserialport,chardev=evidence,name=ota.evidence',
         '-kernel',str(kernel),'-initrd',str(initrd),'-append','console=ttyAMA0 rdinit=/init panic=1 quiet']
(out/'command.json').write_text(json.dumps(command,indent=2)+'\n')
def identity(path):
    return {'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
metadata={'schema':'adr0022.arm_vm/1','phase':args.phase,'station_accessed':False,
          'network_enabled':False,'host_devices_passed_through':False,
          'assets':[identity(p) for p in (kernel,module,initrd,host/'usr/bin/qemu-system-aarch64',
                                        tools/'adr0022_arm_vm.py',tools/'adr0022_arm_vm_guest.py',
                                        tools/'adr0022_capture_rehearsal.py',tools/'adr0022_capture_review.py',
                                        tools/'adr0022_current_rehearsal.py',tools/'adr0022_current_review.py',
                                        tools/'adr0022_homing_rehearsal.py',tools/'adr0022_homing_review.py',
                                        build/'axis_control_core/commissiond')]}
(out/'manifest.json').write_text(json.dumps(metadata,indent=2)+'\n')
print('Running isolated ARM64 VM (no network or host device passthrough): '+str(out),flush=True)
with (out/'console.log').open('wb') as console:
    process=subprocess.run(command,stdout=console,stderr=subprocess.STDOUT,timeout=480)
log=(out/'console.log').read_text(errors='replace')
for line in log.splitlines():
    if line.startswith(('VM_PROGRESS','VM_RESULT','VM_EXIT')):print(line,flush=True)
if (out/'evidence.tar.gz').stat().st_size:
    with tarfile.open(out/'evidence.tar.gz') as tar:
        tar.extractall(out/'evidence',filter='data')
summary=out/'evidence/summary.json'
if process.returncode or 'VM_EXIT 0' not in log or not summary.is_file() or json.loads(summary.read_text())['status']!='LOCAL_ARM64_VM_PASS':
    print(log[:1500],flush=True)
    raise SystemExit('ARM64 VM rehearsal failed; console and captures retained')
print('ARM64 VM rehearsal PASS',flush=True)
