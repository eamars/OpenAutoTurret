"""Synthetic guest workload for adr0022_arm_vm.py; no physical interfaces."""
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys

sys.path.insert(0,'/work/tools')
from adr0022_capture_rehearsal import rehearse
from adr0022_capture_review import review

phase=sys.argv[1]
root=Path('/work/evidence');root.mkdir(exist_ok=True)
summary={'phase':phase,'architecture':platform.machine(),'kernel':platform.release(),
         'station_accessed':False,'provenance':'SYNTHETIC','checks':[],'status':'INVALID',
         'binary_sha256':hashlib.sha256(Path('/work/bin/commissiond').read_bytes()).hexdigest()}
def executable(name,args):
    print('VM_PROGRESS '+name,flush=True)
    p=subprocess.run(args,capture_output=True,text=True,timeout=30)
    (root/(name+'.stdout')).write_text(p.stdout)
    (root/(name+'.stderr')).write_text(p.stderr)
    summary['checks'].append({'name':name,'returncode':p.returncode})
    if p.returncode: raise RuntimeError(name+' failed')

try:
    executable('kernel-capture',['/work/bin/probe-commission-capture',str(root/'kernel-capture.jsonl')])
    print('VM_PROGRESS integrated-capture',flush=True)
    result=rehearse(Path('/work/bin/commissiond'),root/'capture',duration=3. if phase=='probe' else 120.)
    report=review(root/'capture/capture.jsonl')
    (root/'capture/review.json').write_text(json.dumps(report,indent=2)+'\n')
    summary['checks'].append({'name':'integrated-capture','result':result['result'],'review':report})
    if phase=='matrix':
        executable('native-contracts',['/work/bin/test_commission_capture'])
        for fault in ('can_stale','can_error','can_truncated','imu_sequence','imu_reset','imu_eof',
                      'read_echo','read_source','read_timeout','wrong_uid','reenabled','stop_timeout'):
            print('VM_PROGRESS fault-'+fault,flush=True)
            outcome=rehearse(Path('/work/bin/commissiond'),root/fault,fault=fault)
            summary['checks'].append({'name':fault,'expected_rejection':True,'returncode':outcome['returncode']})
    summary['status']='LOCAL_ARM64_VM_PASS'
except Exception as exc:
    summary['detail']=str(exc)
    raise
finally:
    (root/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print('VM_RESULT '+json.dumps({'status':summary['status'],'checks':len(summary['checks']),
                                 'detail':summary.get('detail')}),flush=True)
