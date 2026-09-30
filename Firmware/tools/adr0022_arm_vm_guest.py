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
    if phase.startswith('current-'):
        from adr0022_current_rehearsal import rehearse as current_rehearse, FAULTS
        from adr0022_current_review import review as current_review
        faults = ('none',) if phase=='current-probe' else FAULTS
        for fault in faults:
            print('VM_PROGRESS current-'+fault,flush=True)
            outcome=current_rehearse(Path('/work/bin/commissiond'),root/('current-'+fault),fault=fault)
            check={'name':'current-'+fault,'returncode':outcome['returncode'],'result':outcome['result']}
            if fault in ('none','write_echo'):
                report=current_review(root/('current-'+fault)/'capture.jsonl')
                (root/('current-'+fault)/'review.json').write_text(json.dumps(report,indent=2)+'\n')
                check['review']=report
            else:
                try:
                    current_review(root/('current-'+fault)/'capture.jsonl')
                except ValueError:
                    check['independent_review_rejected']=True
                else:
                    raise RuntimeError('independent reviewer accepted failed current preparation: '+fault)
            summary['checks'].append(check)
        summary['status']='LOCAL_ARM64_VM_PASS'
        sys.exit(0)
    executable('kernel-capture',['/work/bin/probe-commission-capture',str(root/'kernel-capture.jsonl')])
    print('VM_PROGRESS integrated-capture',flush=True)
    result=rehearse(Path('/work/bin/commissiond'),root/'capture',duration=3. if phase=='probe' else 120.)
    report=review(root/'capture/capture.jsonl')
    (root/'capture/review.json').write_text(json.dumps(report,indent=2)+'\n')
    summary['checks'].append({'name':'integrated-capture','result':result['result'],'review':report})
    print('VM_PROGRESS capability-rejection',flush=True)
    limited=rehearse(Path('/work/bin/commissiond'),root/'capability-rejection',fault='read_rejected')
    report=review(root/'capability-rejection/capture.jsonl')
    (root/'capability-rejection/review.json').write_text(json.dumps(report,indent=2)+'\n')
    if {r['index'] for r in report['measurement_limitations']} != {0x7019,0x701A}:
        raise RuntimeError('missing capability limitations')
    if 'pitch_iqf' in report['streams'] or report['physical_parameters_qualified']:
        raise RuntimeError('rejected current feedback fabricated or qualified')
    summary['checks'].append({'name':'capability-rejection','result':limited['result'],'review':report})
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
