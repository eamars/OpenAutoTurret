"""Exercise real Linux process ownership without camera or CAN access."""
import os, pathlib, shutil, subprocess, sys, time
import textwrap
import pytest

pytestmark = pytest.mark.skipif(sys.platform != 'linux', reason='Linux process and signal integration')


def test_relocate_generated_ctest_paths(tmp_path):
    """Rebase both top-level and nested CTest metadata from the build host."""
    firmware = tmp_path / 'release' / 'Firmware'
    build = firmware / 'build-arm64'
    old_source = '/workspace/OpenAutoTurret/Firmware'
    old_build = old_source + '/build-cross'
    (build / 'control').mkdir(parents=True)
    (build / 'CMakeCache.txt').write_text(
        f'CMAKE_HOME_DIRECTORY:INTERNAL={old_source}\n'
        f'CMAKE_CACHEFILE_DIR:INTERNAL={old_build}\n', encoding='utf-8')
    top = build / 'CTestTestfile.cmake'
    nested = build / 'control' / 'CTestTestfile.cmake'
    top.write_text(f'add_subdirectory("{old_source}/control")\n', encoding='utf-8')
    nested.write_text(
        f'add_test(probe "{old_build}/control/test_probe" "{old_source}/config/turret.yaml")\n',
        encoding='utf-8')
    helper = pathlib.Path(__file__).resolve().parents[1] / 'relocate_ctest_paths.py'

    result = subprocess.run([sys.executable, str(helper), str(build), str(firmware)],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert str(firmware / 'control') in top.read_text(encoding='utf-8')
    relocated = nested.read_text(encoding='utf-8')
    assert str(build / 'control/test_probe') in relocated
    assert str(firmware / 'config/turret.yaml') in relocated

    # The deployment launcher may be retried after a test failure; relocation is idempotent.
    again = subprocess.run([sys.executable, str(helper), str(build), str(firmware)],
                           capture_output=True, text=True, timeout=10)
    assert again.returncode == 0, again.stderr
    assert '0 rebased' in again.stdout


def test_prebuilt_deploy_runs_registered_ctest_suite(tmp_path):
    """The prebuilt deployment path delegates the complete registered suite to CTest."""
    firmware = tmp_path / 'Firmware'
    (firmware / 'scripts').mkdir(parents=True)
    build = firmware / 'build-arm64'
    (build / 'control').mkdir(parents=True)
    (firmware / 'tools').mkdir()
    (firmware / 'config').mkdir()
    (firmware / 'config' / 'turret.yaml').write_text('hardware_profile: test\n')
    (firmware / 'config' / 'turret_mixed.yaml').write_text('hardware_profile: test\n')
    (build / 'control' / 'controld').write_text('#!/bin/sh\nexit 0\n')
    (build / 'control' / 'controld').chmod(0o755)
    old_source = '/build-host/OpenAutoTurret/Firmware'
    old_build = old_source + '/build-arm64'
    (build / 'CMakeCache.txt').write_text(
        f'CMAKE_HOME_DIRECTORY:INTERNAL={old_source}\n'
        f'CMAKE_CACHEFILE_DIR:INTERNAL={old_build}\n', encoding='utf-8')
    metadata = build / 'CTestTestfile.cmake'
    metadata.write_text(f'add_test(fake "{old_build}/control/test_fake")\n')
    source_tools = pathlib.Path(__file__).resolve().parents[1]
    shutil.copy(source_tools / 'relocate_ctest_paths.py', firmware / 'tools')
    shutil.copy(source_tools.parent / 'scripts' / 'run_application.sh',
                firmware / 'scripts' / 'run_application.sh')

    fake_python = tmp_path / 'python'
    fake_python.write_text(
        '#!/bin/sh\n'
        'case "$1" in *relocate_ctest_paths.py) exec "$PROBE_PY" "$@" ;; esac\n'
        'if [ "$1" = -c ]; then echo 42; fi\n'
        'exit 0\n')
    fake_python.chmod(0o755)
    calls = tmp_path / 'ctest-calls'
    fake_ctest = tmp_path / 'ctest'
    fake_ctest.write_text(textwrap.dedent('''\
        #!/bin/bash
        printf '%s\\n' "$*" >> "$PROBE_CTEST_CALLS"
        if [[ "$*" == *--show-only=json-v1* ]]; then
          "$PROBE_PY" -c 'import json; print(json.dumps({"tests": [{}] * 42}))'
        fi
        '''))
    fake_ctest.chmod(0o755)
    env = os.environ.copy()
    env.update(OTA_PYTHON=str(fake_python), OTA_PREBUILT='1',
               PROBE_CTEST_CALLS=str(calls), PROBE_PY=sys.executable,
               PATH=str(tmp_path) + os.pathsep + env['PATH'])
    result = subprocess.run(['bash', str(firmware / 'scripts/run_application.sh'), 'deploy'],
                            env=env, capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
    assert '42 registered tests, 0 failed' in result.stdout
    invocations = calls.read_text().splitlines()
    assert len(invocations) == 2, invocations
    assert '--show-only=json-v1' in invocations[0]
    assert '--output-on-failure' in invocations[1]
    assert str(build / 'control/test_fake') in metadata.read_text(encoding='utf-8')


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
    # The fake probe records the argv it was handed, so "the launcher forwarded the ampere option"
    # is asserted rather than assumed -- the whole point of --yaw-current-a is that a silently
    # dropped option would look exactly like a probe that ran and saw nothing.
    probe.write_text('#!/usr/bin/env bash\nprintf \'%s\\n\' "$@" > "$PROBE_ROOT/last-args.txt"\n'
                     'exec "$PROBE_PY" "$PROBE_ROOT/worker.py"\n')
    probe.chmod(0o755)
    env = os.environ.copy()
    env.update(OTA_RUN_DIR=str(tmp_path / 'runtime'), OTA_PYTHON=str(fake_python),
               PROBE_ROOT=str(tmp_path), PROBE_PY=sys.executable)

    def call(action, *options, environment=None):
        return subprocess.run(['bash', str(script), action, *options], env=environment or env,
                              capture_output=True, text=True, timeout=25)
    try:
        result = call('start', '--commission-hardware', '--yaw-current-a', '0.25')
        assert result.returncode == 0, result.stdout + result.stderr
        forwarded = (tmp_path / 'last-args.txt').read_text().split()
        assert '--yaw-current-a' in forwarded, forwarded
        assert forwarded[forwarded.index('--yaw-current-a') + 1] == '0.25'
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
        # Either spelling of a yaw push is commissioning-only, and the ampere one must be caught by
        # the same rule as the voltage one -- not because 0.25 A is gentle, but because a push
        # outside commissioning means two processes on one CAN bus.
        assert call('run', '--yaw-voltage', '10').returncode == 2
        refused_push = call('run', '--yaw-current-a', '0.25')
        assert refused_push.returncode == 2
        assert 'require --commission-hardware' in refused_push.stderr
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
    if [ "$1" = - ]; then echo 0; exit 0; fi
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
     for role in ('control','web.webd.app','perception.visiond'):
      (base/(role+'.pid')).unlink(missing_ok=True)
      (base/(role+'.term')).unlink(missing_ok=True)
     call('start','--hold-motion')
     wait_for(lambda:(base/'perception.visiond.pid').exists())
     assert not (base/'control.pid').exists()
     assert not (base/'web.webd.app.pid').exists()
     assert 'Mode: perception' in call('status').stdout
     call('stop')
     assert not (base/'runtime/launcher.pid').exists()
     assert (base/'perception.visiond.term').exists()
     print('PASS perception-only process remains stoppable through the same script',flush=True)
     env['PROBE_FAIL_PREFLIGHT']='1'
     assert call('start','--sim',ok=False).returncode!=0
     assert not (base/'runtime/launcher.pid').exists()
     del env['PROBE_FAIL_PREFLIGHT']
    finally:
     call('stop',ok=False)
     print('Evidence:',base,flush=True)
