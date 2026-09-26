"""Exercise real Linux process ownership without camera or CAN access."""
import os, pathlib, shutil, subprocess, sys, time
import textwrap
import pytest

pytestmark = pytest.mark.skipif(sys.platform != 'linux', reason='Linux process and signal integration')


def test_commissioning_ownership_and_stop(tmp_path):
    """Exercise supervision and cross-runtime ownership without CAN or cameras."""
    firmware = tmp_path / 'Firmware'
    (firmware / 'scripts').mkdir(parents=True)
    (firmware / 'build').mkdir()
    script = firmware / 'scripts/run_application.sh'
    shutil.copy(pathlib.Path(__file__).resolve().parents[2] / 'scripts/run_application.sh', script)
    fake_python = tmp_path / 'preflight'
    fake_python.write_text('#!/usr/bin/env bash\nexit 0\n')
    fake_python.chmod(0o755)
    worker = tmp_path / 'worker.py'
    worker.write_text(textwrap.dedent('''\
        import os, signal, time
        from pathlib import Path
        root = Path(os.environ['PROBE_ROOT'])
        def stop(*_):
            (root / 'zero-requested').touch()
            print('COMMISSIONING FINISHED; zero output requested; disabled state unavailable', flush=True)
            raise SystemExit(0)
        signal.signal(signal.SIGTERM, stop)
        (root / 'probe-ready').touch()
        while True: time.sleep(.01)
        '''))
    probe = firmware / 'build/probe-mixed-hardware'
    probe.write_text('#!/usr/bin/env bash\nexec "$PROBE_PY" "$PROBE_ROOT/worker.py"\n')
    probe.chmod(0o755)
    env = os.environ.copy()
    env.update(OTA_RUN_DIR=str(tmp_path / 'runtime'), OTA_PYTHON=str(fake_python),
               PROBE_ROOT=str(tmp_path), PROBE_PY=sys.executable)

    def call(action, *options, environment=None):
        return subprocess.run(['bash', str(script), action, *options], env=environment or env,
                              capture_output=True, text=True, timeout=25)
    try:
        result = call('start', '--commission-hardware')
        assert result.returncode == 0, result.stdout + result.stderr
        for _ in range(100):
            if (tmp_path / 'probe-ready').exists(): break
            time.sleep(.02)
        assert (tmp_path / 'probe-ready').exists()
        assert 'Mode: commissioning' in call('status').stdout
        other_env = dict(env, OTA_RUN_DIR=str(tmp_path / 'other-runtime'))
        refused = call('run', '--commission-hardware', environment=other_env)
        assert refused.returncode != 0
        assert 'Another launcher owns station motion' in refused.stderr
        assert not (tmp_path / 'other-runtime/launcher.pid').exists()
        stopped = call('stop')
        assert stopped.returncode == 0, stopped.stdout + stopped.stderr
        assert (tmp_path / 'zero-requested').exists()
        assert 'not a park/disable certification' in call('status').stdout
        assert 'PARKED' not in call('status').stdout
        assert call('run', '--yaw-voltage', '10').returncode == 2
    finally:
        call('stop')

def test_launcher_lifecycle(tmp_path):
    source=pathlib.Path(__file__).resolve().parents[2]/'scripts/run_application.sh'
    base=tmp_path
    firmware=base/'Firmware';(firmware/'scripts').mkdir(parents=True);(firmware/'build/control').mkdir(parents=True)
    shutil.copy(source,firmware/'scripts/run_application.sh')
    worker=base/'worker.py'
    worker.write_text(textwrap.dedent('''\
    import os,signal,sys,time
    from pathlib import Path
    root=Path(os.environ['PROBE_ROOT']);name=sys.argv[-1]
    (root/(name+'.pid')).write_text(str(os.getpid()))
    def stop(*_):
     (root/(name+'.term')).touch()
     if name=='control':time.sleep(.4);(root/'disabled').touch()
     raise SystemExit(0)
    signal.signal(signal.SIGTERM,stop)
    while True:
     if name=='perception.visiond' and (root/'fail-vision').exists():raise SystemExit(7)
     time.sleep(.02)
    '''))
    fakepy=base/'fake-python'
    fakepy.write_text('''#!/usr/bin/env bash
    if [ "${PROBE_FAIL_PREFLIGHT:-0}" = 1 ]; then exit 17; fi
    if [[ "$1" = -c || "$1" = *.py ]]; then exit 0; fi
    exec "$PROBE_PY" "$PROBE_ROOT/worker.py" "$2"
    ''');fakepy.chmod(0o755)
    controller=firmware/'build/control/controld'
    controller.write_text('#!/usr/bin/env bash\nexec "$PROBE_PY" "$PROBE_ROOT/worker.py" control\n');controller.chmod(0o755)
    env=os.environ.copy();env.update(OTA_RUN_DIR=str(base/'runtime'),OTA_PYTHON=str(fakepy),PROBE_ROOT=str(base),PROBE_PY=sys.executable)
    script=firmware/'scripts/run_application.sh'
    def call(action,*args,ok=True):
     r=subprocess.run(['bash',str(script),action,*args],env=env,text=True,capture_output=True,timeout=25)
     if ok and r.returncode:raise RuntimeError(r.stdout+r.stderr)
     return r
    def wait_for(predicate):
     for _ in range(100):
      if predicate():return
      time.sleep(.05)
     raise RuntimeError('probe deadline')
    try:
     call('start','--sim');wait_for(lambda:all((base/(role+'.pid')).exists() for role in ('control','web.webd.app','perception.visiond')))
     pid=(base/'runtime/launcher.pid').read_text()
     assert 'Already running' in call('start','--sim').stdout
     assert (base/'runtime/launcher.pid').read_text()==pid
     other=base/'other';shutil.copytree(firmware,other)
     rejected=subprocess.run(['bash',str(other/'scripts/run_application.sh'),'start','--sim'],env=env,capture_output=True,timeout=10)
     assert rejected.returncode!=0
     assert (base/'runtime/launcher.pid').read_text()==pid
     assert 'Running' in call('status').stdout
     call('stop');assert (base/'disabled').exists()
     assert 'PARK FAILED or park not confirmed' in call('status', ok=False).stdout
     assert 'Killed' not in (base/'runtime/launcher.log').read_text()
     assert all((base/(role+'.term')).exists() for role in ('control','web.webd.app','perception.visiond'))
     assert call('status',ok=False).returncode==1
     assert 'Already stopped' in call('stop').stdout
     print('PASS detached start, idempotency, status, controlled disable and full cleanup',flush=True)
     call('start','--sim');(base/'fail-vision').touch()
     wait_for(lambda:not (base/'runtime/launcher.pid').exists())
     assert (base/'disabled').exists()
     print('PASS failed camera process shuts down its controller and web siblings',flush=True)
     (base/'fail-vision').unlink()
     call('start','--hold-motion');call('stop')
     assert not (base/'runtime/launcher.pid').exists()
     print('PASS perception-only process remains stoppable through the same script',flush=True)
     env['PROBE_FAIL_PREFLIGHT']='1'
     assert call('start','--sim',ok=False).returncode!=0
     assert not (base/'runtime/launcher.pid').exists()
     del env['PROBE_FAIL_PREFLIGHT']
    finally:
     call('stop',ok=False)
     print('Evidence:',base,flush=True)
