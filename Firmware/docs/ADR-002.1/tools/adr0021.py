#!/usr/bin/env python3
"""ADR-002.1 OFFLINE reference contract. No hardware or network transports.

Input traces must already contain unwrapped, deduplicated primary encoder samples
and the actual backend reference. This is not a raw CAN parser or a motor tuner.
"""
from __future__ import annotations
import argparse
import bisect
import csv
import hashlib
import json
import math
import random
import statistics
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]

class ContractError(ValueError):
    pass

def finite(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ContractError(f'{name}: finite numeric value required')
    result = float(value)
    if not math.isfinite(result):
        raise ContractError(f'{name}: NaN/Inf forbidden')
    return result

def digest(value: Any) -> str:
    """Hash canonical JSON; actual firmware must canonicalize its encoding first."""
    try:
        blob = json.dumps(value, sort_keys=True, separators=(',', ':'),
                          ensure_ascii=False, allow_nan=False).encode('utf-8')
    except (ValueError, TypeError) as exc:
        raise ContractError('Non-canonical/non-finite JSON') from exc
    return hashlib.sha256(blob).hexdigest()

def load_json(path: Path) -> dict[str, Any]:
    with path.open(encoding='utf-8') as stream:
        return json.load(stream)

def save_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')

def quantile(values: list[float], p: float) -> float:
    if not values:
        raise ContractError('Empty quantile')
    ordered = sorted(values)
    x = (len(ordered)-1)*p
    i = int(math.floor(x))
    j = min(i+1, len(ordered)-1)
    return ordered[i] + (x-i)*(ordered[j]-ordered[i])

def rms(values: list[float]) -> float:
    if not values:
        raise ContractError('Empty RMS')
    return math.sqrt(statistics.fmean(x*x for x in values))

def line_fit(ts: list[float], qs: list[float]) -> tuple[float, float]:
    if len(ts) != len(qs) or len(ts) < 2:
        raise ContractError('Insufficient regression data')
    mt, mq = statistics.fmean(ts), statistics.fmean(qs)
    den = sum((t-mt)**2 for t in ts)
    if den <= 0:
        raise ContractError('Zero time span')
    slope = sum((t-mt)*(q-mq) for t,q in zip(ts,qs))/den
    return mq-slope*mt, slope

def validate_space(spec: dict[str, Any]) -> None:
    if spec.get('physical_execution_authorized') is not False:
        raise ContractError('Reference planner is OFFLINE_ONLY')
    count = spec.get('count')
    if isinstance(count, bool) or not isinstance(count, int) or not 1 <= count <= 16:
        raise ContractError('Coarse count must be an integer in 1..16')
    seed = spec.get('seed')
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise ContractError('Integer seed required')
    dims = spec.get('dimensions', [])
    if not dims or len({d.get('name') for d in dims}) != len(dims):
        raise ContractError('Nonempty unique dimensions required')
    for d in dims:
        lo, hi = finite(d['low'], 'low'), finite(d['high'], 'high')
        if lo >= hi or d.get('scale') not in ('linear', 'log'):
            raise ContractError('Invalid dimension interval/scale')
        if d['scale'] == 'log' and lo <= 0:
            raise ContractError('Log interval must be positive')
    names = {d['name'] for d in dims}
    for key, d in spec.get('derived', {}).items():
        if key in names or d['run_key'] not in names or d['fraction_key'] not in names:
            raise ContractError('Invalid derived field references')
        finite(d['upper'], 'upper')

def stratified_points(count: int, dims: int, seed: int,
                      intervals: list[tuple[float,float]] | None = None) -> list[list[float]]:
    rng = random.Random(seed)
    cols: list[list[float]] = []
    for j in range(dims):
        order = list(range(count)); rng.shuffle(order)
        lo, hi = (intervals[j] if intervals else (0.0, 1.0))
        if not (0 <= lo < hi <= 1):
            raise ContractError('Normalized interval outside [0,1]')
        cols.append([lo + (hi-lo)*(i+0.5)/count for i in order])
    return [[cols[j][i] for j in range(dims)] for i in range(count)]

def decode_point(spec: dict[str, Any], point: list[float]) -> dict[str, float]:
    if len(point) != len(spec['dimensions']):
        raise ContractError('Wrong point dimension')
    values: dict[str,float] = {}
    for d,z in zip(spec['dimensions'], point):
        z = finite(z, 'normalized')
        if not 0 <= z <= 1:
            raise ContractError('Point outside domain')
        lo, hi = d['low'], d['high']
        x = math.exp(math.log(lo)+z*(math.log(hi)-math.log(lo))) if d['scale']=='log' else lo+z*(hi-lo)
        values[d['name']] = round(x,12)
    for name, d in spec.get('derived',{}).items():
        run, frac, upper = values[d['run_key']], values[d['fraction_key']], d['upper']
        if not (0 <= frac <= 1 and 0 <= run <= upper):
            raise ContractError('Invalid breakaway/run mapping')
        values[name] = round(run+frac*(upper-run),12)
    for d in spec.get('derived',{}).values():
        values.pop(d['fraction_key'],None)
    return values

def make_candidates(spec: dict[str,Any], points: list[list[float]], prefix: str) -> list[dict[str,Any]]:
    candidates, seen = [], set()
    for i, point in enumerate(points):
        params = decode_point(spec,point)
        h = digest(params)
        if h in seen:
            continue
        seen.add(h)
        candidates.append({'candidate_id':f'{prefix}-{i:03d}', 'normalized':point,
                           'parameters':params, 'parameters_hash':h,
                           'direction_order':[1,-1] if i%2==0 else [-1,1]})
    return candidates

def make_plan(spec: dict[str,Any], policy: dict[str,Any]) -> dict[str,Any]:
    validate_space(spec)
    if spec['count'] != policy['search']['coarse_count'] or spec['seed'] != policy['search']['seed']:
        raise ContractError('Candidate count/seed differs from frozen policy')
    points = stratified_points(spec['count'],len(spec['dimensions']),spec['seed'])
    payload = {'adr':'002.1','offline_only':True,'physical_execution_authorized':False,
               'space_hash':digest(spec),'policy_hash':digest(policy),'seed':spec['seed'],
               'candidates':make_candidates(spec,points,'coarse')}
    return {**payload,'plan_hash':digest(payload)}

def verify_plan(plan: dict[str,Any]) -> None:
    actual = dict(plan); expected = actual.pop('plan_hash',None)
    if expected != digest(actual) or actual.get('physical_execution_authorized') is not False:
        raise ContractError('Plan changed or reference scope violated')

def refine_plan(spec: dict[str,Any], centers: list[dict[str,Any]], policy: dict[str,Any]) -> list[dict[str,Any]]:
    validate_space(spec)
    if len(centers) != 2:
        raise ContractError('Exactly two valid frozen centers required')
    result, seen = [], set()
    for i,center in enumerate(centers):
        point = center['normalized']
        if len(point) != len(spec['dimensions']) or any(not 0<=finite(z,'center')<=1 for z in point):
            raise ContractError('Invalid center')
        half = policy['search']['refine_half_width_normalized']
        boxes = [(max(0,z-half),min(1,z+half)) for z in point]
        points = stratified_points(4,len(point),spec['seed']+i+1,boxes)
        for item in make_candidates(spec,points,f'refine{i}'):
            if item['parameters_hash'] not in seen:
                result.append(item); seen.add(item['parameters_hash'])
    return result

class TrialGate:
    """Executable mock of the receipt barrier; not an actual parameter writer."""
    def __init__(self, expected_values: dict[str,Any], revision: int, binary_hash: str, plan_hash: str):
        self.expected_hash = digest(expected_values)
        self.revision, self.binary_hash, self.plan_hash = revision,binary_hash,plan_hash
        self.state, self.run_count = 'APPLY',0

    def _abort(self, message: str) -> None:
        self.state = 'ABORT_CAMPAIGN'
        raise ContractError(message)

    def accept_receipt(self, receipt: dict[str,Any]) -> None:
        if self.state != 'APPLY':
            self._abort('Unexpected or duplicate parameter receipt')
        if receipt.get('accepted') is not True or receipt.get('readback_verified') is not True:
            self._abort('Application/readback failed: RUN forbidden')
        if receipt.get('revision') != self.revision or receipt.get('binary_hash') != self.binary_hash:
            self._abort('Wrong revision or binary')
        if receipt.get('plan_hash') != self.plan_hash:
            self._abort('Wrong frozen plan')
        try:
            actual = digest(receipt['effective_values'])
        except (KeyError,ContractError):
            self._abort('Missing/invalid effective values')
        if actual != self.expected_hash or receipt.get('effective_hash') != actual:
            self._abort('Requested/effective values mismatch')
        self.state = 'LOG_READY'

    def start(self, log_ready: bool) -> None:
        if self.state != 'LOG_READY' or log_ready is not True:
            self._abort('Verified parameters and complete capture readiness required')
        self.run_count += 1; self.state = 'RUN'

    def observe_identity(self, parameters_hash: str, revision: int, binary_hash: str) -> None:
        if self.state != 'RUN' or (parameters_hash,revision,binary_hash) != (self.expected_hash,self.revision,self.binary_hash):
            self._abort('Active trial identity changed')

    def finish(self, result: str) -> None:
        if self.state != 'RUN' or result not in ('PASS_SCOPE','FAIL_QUALITY','INVALID_DATA','HARD_ABORT'):
            self._abort('Invalid completion')
        self.state = 'ABORT_CAMPAIGN' if result=='HARD_ABORT' else 'FINISHED'

def apply_registry(registry: dict[str,dict[str,Any]], values: dict[str,Any]) -> dict[str,float]:
    """Offline type/range gate. Production implementation must enforce server-side."""
    checked: dict[str,float] = {}
    for name,value in values.items():
        if name not in registry or registry[name].get('mutability')!='experiment_writable':
            raise ContractError(f'{name}: not experiment-writable')
        x = finite(value,name); d=registry[name]
        if not d['low'] <= x <= d['high']:
            raise ContractError(f'{name}: outside approved test interval')
        checked[name]=x
    return checked

def validate_rows(rows: list[dict[str,Any]], expected_hash: str, m: dict[str,Any]) -> list[dict[str,Any]]:
    if not rows:
        raise ContractError('Missing trace')
    result=[]; previous=None
    for row in rows:
        item={'t_s':finite(row['t_s'],'t_s'),'q_deg':finite(row['q_deg'],'q_deg'),
              'be_cmd_dps':finite(row['be_cmd_dps'],'be_cmd_dps'), 'phase':row['phase']}
        if row.get('parameters_hash') != expected_hash:
            raise ContractError('Parameter hash mismatch in trace')
        if row.get('reference_origin') != 'be_cmd':
            raise ContractError('Reference is not actual backend be_cmd')
        if previous is not None:
            dt=item['t_s']-previous
            if dt <= 0 or dt > m['max_gap_s']+1e-9:
                raise ContractError('Nonmonotonic/duplicate timestamps or missing trace interval')
        previous=item['t_s']; result.append(item)
    return result

def local_velocities(points: list[dict[str,Any]], m: dict[str,Any]) -> list[float]:
    ts=[p['t_s'] for p in points]; qs=[p['q_deg'] for p in points]
    half=m['velocity_window_s']/2; values=[]
    for t in ts:
        if t < ts[0]+half-1e-9 or t > ts[-1]-half+1e-9:
            continue
        a=bisect.bisect_left(ts,t-half-1e-9); b=bisect.bisect_right(ts,t+half+1e-9)
        if b-a<m['velocity_samples_min'] or ts[b-1]-ts[a]<m['velocity_span_min_s']-1e-9:
            continue
        values.append(line_fit(ts[a:b],qs[a:b])[1])
    if not values:
        raise ContractError('Insufficient independent data for fixed velocity estimator')
    return values

def clip_hold(points: list[dict[str,Any]], seconds: float) -> list[float]:
    if not points or points[-1]['t_s']-points[0]['t_s']<seconds-1e-9:
        raise ContractError('Incomplete fixed hold observation')
    end=points[0]['t_s']+seconds
    values=[p['q_deg'] for p in points if p['t_s']<=end+1e-9]
    ts=[p['t_s'] for p in points]
    j=bisect.bisect_left(ts,end)
    if j<len(points) and abs(ts[j]-end)>1e-9:
        p0,p1=points[j-1],points[j]
        w=(end-p0['t_s'])/(p1['t_s']-p0['t_s'])
        values.append(p0['q_deg']+w*(p1['q_deg']-p0['q_deg']))
    return values

def score(rows: list[dict[str,Any]], reference_dps: float, expected_hash: str,
          policy: dict[str,Any]) -> dict[str,Any]:
    m=policy['metrics']
    try:
        ref=finite(reference_dps,'reference_dps')
        if ref==0: raise ContractError('Nonzero steady reference required')
        normalized=validate_rows(rows,expected_hash,m)
        steady=[r for r in normalized if r['phase']=='steady']
        hold=[r for r in normalized if r['phase']=='hold']
        if len(steady)<5 or steady[-1]['t_s']-steady[0]['t_s']<m['steady_min_s']-1e-9:
            raise ContractError('Insufficient steady interval')
        indices=[i for i,r in enumerate(normalized) if r['phase']=='steady']
        hi=[i for i,r in enumerate(normalized) if r['phase']=='hold']
        if indices[-1]-indices[0]+1!=len(indices) or not hi or hi[0]<=indices[-1] or hi[-1]-hi[0]+1!=len(hi):
            raise ContractError('Phase segmentation invalid')
        tol=max(m['reference_absolute_tolerance_dps'],abs(ref)*m['reference_relative_tolerance'])
        if any(abs(r['be_cmd_dps']-ref)>tol for r in steady):
            raise ContractError('INVALID_REFERENCE: final backend plateau not reached')
        if any(abs(r['be_cmd_dps'])>m['reference_absolute_tolerance_dps'] for r in hold):
            raise ContractError('Hold reference is not zero')
        ts=[p['t_s'] for p in steady]; qs=[p['q_deg'] for p in steady]
        intercept,slope=line_fit(ts,qs)
        residual=[q-(intercept+slope*t) for q,t in zip(qs,ts)]
        velocities=local_velocities(steady,m)
        mean_v=statistics.fmean(velocities)
        jv=rms([v-mean_v for v in velocities]); ev=rms([v-ref for v in velocities])
        ratio=mean_v/ref
        active=statistics.fmean(float(math.copysign(1,ref)*v>=m['active_speed_fraction']*abs(ref)) for v in velocities)
        hq=clip_hold(hold,m['hold_window_s']); drift=max(abs(q-hq[0]) for q in hq)
        jq=quantile(residual,.95)-quantile(residual,.05); span=max(residual)-min(residual)
        vlimit=max(m['speed_floor_dps'],abs(ref)*m['speed_fraction'])
        values={'jitter_position_p95_p5_deg':jq,'residual_full_span_deg':span,
                'jitter_velocity_rms_dps':jv,'tracking_rmse_dps':ev,
                'tracking_ratio':ratio,'active_fraction':active,'stop_drift_deg':drift}
        checks={'position_jitter':jq<=m['jitter_position_p95_p5_deg'],
                'position_spike':span<=m['residual_full_span_deg'],
                'velocity_jitter':jv<=vlimit,'tracking_rmse':ev<=vlimit,
                'tracking_ratio':m['tracking_ratio_min']<=ratio<=m['tracking_ratio_max'],
                'sustained_motion':active>=m['active_fraction_min'], 'stop_drift':drift<=m['stop_drift_deg']}
        ratios=[jq/m['jitter_position_p95_p5_deg'],span/m['residual_full_span_deg'],jv/vlimit,ev/vlimit,
                abs(ratio-1)/0.1,(1-active)/(1-m['active_fraction_min']),drift/m['stop_drift_deg']]
        for name,value in values.items(): finite(value,name)
        for value in ratios: finite(value,'normalized metric')
        return {'status':'PASS_SCOPE' if all(checks.values()) else 'FAIL_QUALITY',
                'metrics':values,'checks':checks,'reasons':[k for k,v in checks.items() if not v],
                'max_normalized_quality':max(ratios),'mean_normalized_quality':statistics.fmean(ratios),
                'steady_duration_s':ts[-1]-ts[0],'hold_window_s':m['hold_window_s'],
                'independent_rows':len(normalized),'metrics_hash':digest(m),'parameters_hash':expected_hash,
                'qualification':'SYNTHETIC_OR_SUPPLIED_TRACE_ONLY_NOT_HARDWARE_QUALIFICATION'}
    except (ContractError,KeyError,TypeError,OverflowError) as exc:
        return {'status':'INVALID_DATA','reasons':[str(exc)],'max_normalized_quality':None,'mean_normalized_quality':None}

def aggregate(candidate_id: str, results: dict[str,dict[str,Any]], required_cases: list[str]) -> dict[str,Any]:
    if len(required_cases)!=len(set(required_cases)) or set(results)!=set(required_cases):
        return {'candidate_id':candidate_id,'status':'INVALID_DATA','reasons':['Required case set incomplete/mismatched']}
    items=list(results.values())
    if not items or any(r['status'] not in ('PASS_SCOPE','FAIL_QUALITY') for r in items):
        return {'candidate_id':candidate_id,'status':'INVALID_DATA','reasons':['Invalid trial in required group']}
    return {'candidate_id':candidate_id,'status':'PASS_SCOPE' if all(r['status']=='PASS_SCOPE' for r in items) else 'FAIL_QUALITY',
            'max_normalized_quality':max(r['max_normalized_quality'] for r in items),
            'mean_normalized_quality':statistics.fmean(r['mean_normalized_quality'] for r in items)}

def rank_results(results: list[dict[str,Any]], feasible_only: bool=False) -> list[dict[str,Any]]:
    allowed=('PASS_SCOPE',) if feasible_only else ('PASS_SCOPE','FAIL_QUALITY')
    valid=[r for r in results if r.get('status') in allowed]
    for r in valid:
        finite(r['max_normalized_quality'],'score'); finite(r['mean_normalized_quality'],'score')
    return sorted(valid,key=lambda r:(r['max_normalized_quality'],r['mean_normalized_quality'],r['candidate_id']))

def next_stage(results: list[dict[str,Any]], stage: str, expected_candidate_ids: list[str]) -> dict[str,Any]:
    if stage not in ('coarse','refine'): raise ContractError('Unexpected search stage')
    actual_ids=[r.get('candidate_id') for r in results]
    if (not expected_candidate_ids or len(set(expected_candidate_ids))!=len(expected_candidate_ids)
        or len(set(actual_ids))!=len(actual_ids) or set(actual_ids)!=set(expected_candidate_ids)):
        return {'next':'ABORT_CAMPAIGN','candidate_ids':[], 'reason':'Incomplete or changed frozen candidate set'}
    if any(r.get('status') in ('HARD_ABORT','INVALID_DATA','BLOCKED') for r in results):
        return {'next':'ABORT_CAMPAIGN','candidate_ids':[]}
    feasible=rank_results(results,True)
    if feasible:
        return {'next':'CONFIRM','candidate_ids':[r['candidate_id'] for r in feasible[:2]]}
    valid=rank_results(results)
    if stage=='coarse' and len(valid)>=2:
        return {'next':'REFINE_ONCE','candidate_ids':[r['candidate_id'] for r in valid[:2]]}
    return {'next':'NO_FEASIBLE_CANDIDATE','candidate_ids':[]}

def retry_decision(status: str, prior_exact_retries: int) -> str:
    if isinstance(prior_exact_retries,bool) or not isinstance(prior_exact_retries,int) or prior_exact_retries<0:
        raise ContractError('Invalid retry counter')
    if status=='INVALID_DATA':
        return 'RETRY_IDENTICAL_ONCE' if prior_exact_retries==0 else 'ABORT_CAMPAIGN'
    if status=='HARD_ABORT': return 'ABORT_CAMPAIGN'
    if status in ('PASS_SCOPE','FAIL_QUALITY'): return 'ADVANCE_FROZEN_PLAN'
    return 'ABORT_CAMPAIGN'

def synthetic_trace(kind: str, parameters_hash: str, direction: int=1) -> list[dict[str,Any]]:
    """Explicitly synthetic; 50 Hz samples, approximate report encoder resolution."""
    if direction not in (-1,1): raise ContractError('direction must be +/-1')
    dt=.02; resolution=.044; rows=[]
    def quant(x: float) -> float: return round(x/resolution)*resolution
    def position(t: float) -> float:
        if kind=='stalled': return 0.0
        if kind=='creep': return direction*.1*t
        base=direction*3*t
        if kind=='oscillating': base+=.25*math.sin(2*math.pi*4*t)
        if kind=='spike' and abs(t-1.5)<1e-8: base+=.6
        return base
    for i in range(151):
        t=round(i*dt,8)
        rows.append({'t_s':t,'q_deg':quant(position(t)),'be_cmd_dps':direction*3.0,
                     'phase':'steady','parameters_hash':parameters_hash,'reference_origin':'be_cmd'})
    end=rows[-1]['q_deg']
    for i in range(1,127):
        elapsed=i*dt
        q=end + (.18*elapsed if kind=='drifting' else 0.0)
        rows.append({'t_s':round(3+elapsed,8),'q_deg':quant(q),'be_cmd_dps':0.0,
                     'phase':'hold','parameters_hash':parameters_hash,'reference_origin':'be_cmd'})
    return rows

def read_trace(path: Path) -> list[dict[str,Any]]:
    with path.open(newline='',encoding='utf-8') as stream:
        rows=list(csv.DictReader(stream))
    for row in rows:
        for field in ('t_s','q_deg','be_cmd_dps'):
            row[field]=float(row[field])
    return rows

def write_trace(path: Path, rows: list[dict[str,Any]]) -> None:
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)

def demo(output: Path) -> dict[str,Any]:
    policy=load_json(ROOT/'manifests/policy.json'); spec=load_json(ROOT/'manifests/search_space.example.json')
    plan=make_plan(spec,policy); verify_plan(plan); save_json(output/'plan.json',plan)
    params={'kp':1.0,'ki':.6}; h=digest(params)
    expected={'smooth':'PASS_SCOPE','stalled':'FAIL_QUALITY','creep':'FAIL_QUALITY','oscillating':'FAIL_QUALITY','drifting':'FAIL_QUALITY','spike':'FAIL_QUALITY'}
    results={}
    for name,want in expected.items():
        rows=synthetic_trace(name,h); write_trace(output/f'{name}.csv',rows)
        result=score(rows,3.0,h,policy); results[name]=result
        if result['status']!=want: raise ContractError(f'Demo {name}: expected {want}, got {result}')
    save_json(output/'metrics.json',results)
    gate=TrialGate({'kp':2.0,'ki':.6},1,'demo-binary',plan['plan_hash'])
    blocked=False
    try: gate.accept_receipt({'accepted':False,'readback_verified':False})
    except ContractError: blocked=gate.state=='ABORT_CAMPAIGN' and gate.run_count==0
    report={'offline_only':True,'generated_candidates':len(plan['candidates']),
            'synthetic_cases':{k:v['status'] for k,v in results.items()},
            'apply_rejected_run_count':gate.run_count,'apply_rejected_blocked':blocked}
    if not blocked: raise ContractError('Apply rejection was not blocked')
    save_json(output/'summary.json',report); return report

def main() -> int:
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    p=sub.add_parser('demo'); p.add_argument('--output',type=Path,required=True)
    p=sub.add_parser('plan'); p.add_argument('--spec',type=Path,required=True); p.add_argument('--output',type=Path,required=True)
    p=sub.add_parser('score'); p.add_argument('--input',type=Path,required=True); p.add_argument('--reference-dps',type=float,required=True); p.add_argument('--parameters-hash',required=True); p.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    try:
        policy=load_json(ROOT/'manifests/policy.json')
        if args.command=='demo': result=demo(args.output)
        elif args.command=='plan': result=make_plan(load_json(args.spec),policy); save_json(args.output,result)
        else: result=score(read_trace(args.input),args.reference_dps,args.parameters_hash,policy); save_json(args.output,result)
        print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))
        return 2 if result.get('status')=='INVALID_DATA' else 0
    except (ContractError,OSError,ValueError,KeyError) as exc:
        parser.exit(2,f'ERROR: {exc}\n')

if __name__=='__main__':
    raise SystemExit(main())
