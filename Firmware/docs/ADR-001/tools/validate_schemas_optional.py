#!/usr/bin/env python3
"""Optional local schema validation. Requires jsonschema; never installs it."""
from __future__ import annotations
import copy
from importlib.metadata import version
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from offline_checks import strict_json, read_ndjson


def main() -> int:
    try:
        from jsonschema import Draft202012Validator
    except ImportError:
        print('Optional dependency jsonschema is absent. Standard-library unit tests remain usable.', file=sys.stderr)
        return 2
    validators = {}
    for path in sorted((ROOT / 'schemas').glob('*.schema.json')):
        data = strict_json(path.read_text(encoding='utf-8'))
        Draft202012Validator.check_schema(data)
        validators[path.name] = Draft202012Validator(data)
    pairs = [
        ('selected_observation.json','selected_observation.schema.json'),
        ('validation_context.json','validation_context.schema.json'),
        ('stop_evidence.json','stop_evidence.schema.json'),
        ('qualification_report_template.json','qualification_template.schema.json'),
    ]
    positives=0
    for fixture,schema in pairs:
        validators[schema].validate(strict_json((ROOT / 'examples' / fixture).read_text(encoding='utf-8')))
        positives+=1
    for record in read_ndjson(ROOT / 'examples/synthetic_trace.ndjson'):
        validators['trace_event.schema.json'].validate(record)
        positives+=1
    obs=strict_json((ROOT / 'examples/selected_observation.json').read_text(encoding='utf-8'))
    negatives=[]
    for key,val in [('source_frame_sequence',43),('t_observation_ns',1000000000),('camera_generation',True),('schema_version','unknown')]:
        bad=copy.deepcopy(obs);bad[key]=val;negatives.append((validators['selected_observation.schema.json'],bad))
    bad=copy.deepcopy(obs);bad['unexpected_wire_field']=True;negatives.append((validators['selected_observation.schema.json'],bad))
    stop=strict_json((ROOT / 'examples/stop_evidence.json').read_text(encoding='utf-8'));stop['yaw']['disable_confirmed']=True
    negatives.append((validators['stop_evidence.schema.json'],stop))
    for validator,record in negatives:
        if not list(validator.iter_errors(record)):
            raise ValueError('Expected schema rejection was not produced')
    result={'status':'PASS','scope':'PACKAGE_SCHEMAS_AND_SYNTHETIC_FIXTURES_ONLY',
            'jsonschema_version':version('jsonschema'),'schemas_checked':len(validators),
            'positive_fixture_records_checked':positives,'negative_cases_rejected':len(negatives),
            'hardware_qualification':'NOT_RUN','repository_compatibility':'NOT_RUN'}
    print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))
    return 0

if __name__=='__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as exc:
        print(f'error: {exc}',file=sys.stderr)
        raise SystemExit(2)
