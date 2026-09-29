#!/usr/bin/env python3
"""Audit local Stage 1 artifacts. This cannot authorize or qualify hardware."""
from __future__ import annotations
import argparse
from datetime import datetime,timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from Firmware.commissioning.contracts import digest,PlantSnapshot,ModelSpec,Identity,require,Reason
from Firmware.commissioning.native import Native
from Firmware.commissioning.provenance import method_identity,identification_component
from Firmware.commissioning.synthetic import CONDITIONS
from Firmware.commissioning.parameter_catalog import catalog,pending_document

ADR=ROOT/'Firmware/docs/ADR-002.2'
module=importlib.util.spec_from_file_location('adr0022_stage_evidence',ADR/'reference/contracts.py')
evidence_contract=importlib.util.module_from_spec(module);module.loader.exec_module(evidence_contract)


def file_evidence(path):
    data=path.read_bytes()
    return {'path':str(path.relative_to(ROOT)),'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data)}


def audit(matrix_path,local_root):
    native=Native();method=method_identity(native)
    matrix=json.loads(matrix_path.read_text(encoding='utf-8'))
    require(matrix.get('method_hash')==digest(method) and matrix.get('method')==method,
            Reason.INTEGRATION_MISMATCH,'matrix method/build differs from current mathematical implementation')
    require(matrix.get('physical_access') is False and matrix.get('stage2_started') is False and
            matrix.get('physical_qualification')=='NOT_RUN',Reason.DATA_INVALID,'stage boundary evidence differs')
    expected={(axis,c) for axis in ('yaw','pitch') for c in CONDITIONS}
    rows=matrix['results']
    require(matrix.get('expected_cases')==18 and len(rows)==18 and
            {(r['axis'],r['condition']) for r in rows}==expected and all(r['status']=='PASS' for r in rows),
            Reason.DATA_INVALID,'missing, duplicate or failed synthetic condition')
    for row in rows:
        require(row['method_hash']==matrix['method_hash'] and row['bootstrap_models']==128 and
                row['case_evaluations']==129*3*5*12 and row['phase_margin_deg']>=50 and row['gain_margin_db']>=6,
                Reason.DATA_INVALID,'incomplete uncertainty/performance/margin evaluation')
        for kind in ('dataset','snapshot','candidate'):
            path=ROOT/row[kind];document=json.loads(path.read_text(encoding='utf-8'))
            require(path.stem==digest(document),Reason.DATA_INVALID,f'altered immutable {kind}')
            if kind=='snapshot':
                plant=PlantSnapshot.bind(document,ModelSpec(**document['model_spec']),Identity(**document['identity']))
                fit_method=plant.fit_report['method_hash']
                manifest_path=matrix_path.parent/'methods'/f'{fit_method}.json'
                manifest=json.loads(manifest_path.read_text(encoding='utf-8'))
                require(digest(manifest)==fit_method and identification_component(manifest)==identification_component(method),
                        Reason.INTEGRATION_MISMATCH,'reused bootstrap numerical implementation differs')
                require(plant.identity_hash==row['snapshot_hash'] and plant.identity.provenance=='SYNTHETIC' and
                        plant.fit_report['bootstrap_runs']==128 and all(r['passed'] for r in plant.fit_report['holdout']),
                        Reason.DATA_INVALID,'snapshot lacks complete synthetic fitting evidence')
            if kind=='candidate':
                require(document['qualification']=='OFFLINE_CANDIDATE_ONLY' and document['physical_qualification']=='NOT_RUN' and
                        document['method_hash']==matrix['method_hash'] and document['plant_hash']==row['snapshot_hash'] and
                        digest(document)==row['candidate_hash'] and
                        len(document['case_reports'])==row['case_evaluations'] and all(r['passed'] for r in document['case_reports']),
                        Reason.DATA_INVALID,'candidate identity or complete case evidence differs')
    logs={};test_counts={}
    for name,filename,minimum in [('mathematical','stage1-tests.log',91),('acceptance_contract','reference-tests.log',84)]:
        path=local_root/filename;contents=path.read_text(encoding='utf-8')
        found=re.search(r'Ran (\d+) tests? in ',contents)
        require(found is not None and int(found[1])>=minimum and contents.rstrip().endswith('OK') and
                'FAILED (' not in contents,Reason.DATA_INVALID,f'{name} test suite did not pass completely')
        logs[name]=file_evidence(path);test_counts[name]=int(found[1])
    path=local_root/'catalog-tests.log';contents=path.read_text(encoding='utf-8')
    require('Ran 11 tests' in contents and contents.rstrip().endswith('OK'),
            Reason.DATA_INVALID,'final expanded parameter catalog tests did not pass')
    logs['parameter_catalog_final']=file_evidence(path)
    path=local_root/'ctest.log';contents=path.read_text(encoding='utf-8')
    require('100% tests passed, 0 tests failed out of 82' in contents,Reason.DATA_INVALID,'local native regression suite failed')
    logs['native_regression']=file_evidence(path);test_counts['native_regression']=82
    builds={}
    for name,relative,machine in [
        ('host_core','firmware/axis_control_core/libaxis_control_core.so',62),
        ('host_commissiond','firmware/axis_control_core/commissiond',62),
        ('host_controld','firmware/control/controld',62),
        ('arm64_core','core-arm64/libaxis_control_core.so',183),
        ('arm64_commissiond','core-arm64/commissiond',183)]:
        path=local_root/relative;header=path.read_bytes()[:20]
        require(header[:4]==b'\x7fELF' and header[4:6]==b'\x02\x01' and
                int.from_bytes(header[18:20],'little')==machine,Reason.INTEGRATION_MISMATCH,'build architecture differs: '+name)
        builds[name]=file_evidence(path)
    exported=json.loads((ADR/'contracts/parameter_catalog.json').read_text(encoding='utf-8'))
    require(exported['finite']==catalog(5) and exported['periodic_yaw']==catalog(8) and
            json.loads((ADR/'contracts/parameters.pending.json').read_text(encoding='utf-8'))==pending_document(),
            Reason.INTEGRATION_MISMATCH,'exported parameter contracts differ from implementation')
    implementations={
        'S1-01':['Firmware/axis_control_core/CMakeLists.txt','Firmware/commissioning/tests/test_controller_protocol.py'],
        'S1-02':['Firmware/axis_control_core/axis_control_core.cpp','Firmware/commissioning/breakaway.py'],
        'S1-03':['Firmware/commissioning/identification.py'],
        'S1-04':['Firmware/commissioning/synthesis.py','Firmware/commissioning/sampled_analysis.py'],
        'S1-07':['Firmware/commissioning/parameter_catalog.py','Firmware/commissioning/protocol.py'],
        'S1-08':['Firmware/commissioning/protocol.py','Firmware/commissioning/tests/test_additional_paths.py'],
        'S1-09':['Firmware/commissioning/contracts.py','Firmware/commissioning/parameter_catalog.py'],
        'S1-10':['Firmware/commissioning/measurement.py','Firmware/commissioning/normalization.py'],
        'S1-11':['Firmware/commissioning/qualification.py','Firmware/commissioning/tests/test_stage_boundaries.py'],
        'S2-03':['Firmware/commissioning/calibrate.py','Firmware/commissioning/applicability.py'],
        'S2-04':['Firmware/commissioning/adaptation.py','Firmware/commissioning/tests/test_controller_protocol.py'],
        'REL-01':['Firmware/docs/ADR-002.2/reference/contracts.py','Firmware/docs/ADR-002.2/tests/test_reference.py'],
        'REL-03':['Firmware/tools/adr0022_stage1_check.py']}
    requirements=json.loads((ADR/'contracts/requirements.json').read_text(encoding='utf-8'))['requirements']
    results=[]
    for r in requirements:
        selected=r['stage'] in ('1','all') and r['evidence_kind']=='software'
        paths=implementations[r['id']] if selected else []
        results.append({'id':r['id'],'status':'PASS' if selected else 'NOT_RUN',
                        'execution':'software' if selected else 'not_run',
                        'implementation':[file_evidence(ROOT/p) for p in paths],
                        'evidence_ids':['mathematical','acceptance_contract','native_regression','synthetic_matrix'] if selected else []})
    reasons=evidence_contract.stage1_completion_reasons(requirements,results)
    require(not reasons,Reason.DATA_INVALID,'Stage 1 gate: '+str(reasons))
    source_paths=list((ROOT/'Firmware/commissioning').rglob('*.py'))
    source_paths += [p for p in (ROOT/'Firmware/axis_control_core').iterdir() if p.is_file()]
    source_paths += [ROOT/p for p in ('Firmware/CMakeLists.txt','Firmware/control/CMakeLists.txt',
        'Firmware/control/src/main.cpp','Firmware/docs/ADR-002.2/contracts/requirements.json',
        'Firmware/docs/ADR-002.2/contracts/quality_targets.json','Firmware/docs/ADR-002.2/docs/07_STAGE1_OFFLINE_OVERRIDE.md')]
    return {'version':'adr0022.stage1-report/2','recorded_at_utc':datetime.now(timezone.utc).isoformat(),
            'workspace_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
            'status':'STAGE1_MATHEMATICAL_SOFTWARE_PASS','ready_for_stage2_entry':True,'stage2_started':False,
            'physical_access':False,'full_adr_status':'NOT_DONE',
            'stage_status':{'mathematical_software':'PASS','hardware_capability':'NOT_RUN',
                            'physical_calibration':'NOT_RUN','physical_plant_identification':'NOT_RUN','3a':'NOT_RUN','3b':'NOT_RUN'},
            'method':method,'method_hash':digest(method),'builds':builds,'logs':logs,'test_counts':test_counts,
            'identification_component_hash':identification_component(method),
            'source_files':[file_evidence(p) for p in sorted(source_paths)],
            'synthetic_matrix':file_evidence(matrix_path),'conditions':rows,
            'case_evaluations':sum(r['case_evaluations'] for r in rows),'requirements':results,
            'stage1_gate_reasons':reasons,
            'full_adr_unmet':evidence_contract.completion_reasons(requirements,results,False),
            'limits':['Synthetic results do not identify or qualify the station.',
                      'Host executable replay is not normal-production 3b evidence.',
                      'ARM64 cross-build covers the shared core and independent replay program; no target execution.',
                      'retained_homing excluded from local CTest as required by repository instructions.',
                      'Finite model/domain validation is not a proof for arbitrary unknown physics.']}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--local-root',type=Path,default=ROOT/'run/adr0022-local')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    try:
        report=audit(args.local_root/'qualification/matrix.json',args.local_root)
        if args.output:
            args.output.parent.mkdir(parents=True,exist_ok=True)
            args.output.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
        print(json.dumps({'status':report['status'],'stage2_started':False,'full_adr_status':'NOT_DONE',
                          'test_counts':report['test_counts'],'synthetic_cases':report['case_evaluations']}))
        return 0
    except (ValueError,KeyError,OSError,TypeError) as exc:
        print(json.dumps({'status':'STAGE1_NOT_READY','detail':str(exc)}));return 2


if __name__=='__main__':raise SystemExit(main())
