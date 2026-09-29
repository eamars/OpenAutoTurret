"""Run synthetic mathematics and metadata-contract examples only."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
from .model_math import integral_rows, fit_initializer, ideal_pi, linear_discrete_matrix, spectral_radius
from .contracts import canonical_hash, bundle_reasons, DIGEST_KEYS


def synthetic_data():
    # A constructed smooth velocity history and exact current-equivalent law.
    # These are not measured GM6020/CyberGear properties.
    t = np.arange(0., 40., .005)
    w = .4*np.sin(.9*t) + .13*np.sin(2.3*t+.2)
    alpha = .36*np.cos(.9*t) + .299*np.cos(2.3*t+.2)
    q = -.4/.9*np.cos(.9*t) - .13/2.3*np.cos(2.3*t+.2)
    d = np.sign(w).astype(int)
    a,b,hp,hm = .08,.045,.12,-.07
    iq = a*alpha + b*w + np.where(d>0,hp,hm)
    return (t,q,w,iq,d), {'a':a,'b':b,'load_positive':hp,'load_negative':hm}


def fixture_bundle():
    """Synthetic TEST declarations, including physical claims, to test the gate.

    Nothing returned by this function is a real physical certificate.
    """
    p = {'candidate_id':'SYNTHETIC_TEST_ONLY',**{k:canonical_hash('fixture:'+k) for k in DIGEST_KEYS},
         'axes':['yaw','pitch'],'conditions':['BASELINE'],
         'required_cases':['response','smoothness','stop']}
    certs=[]
    for stage in ('3a','3b'):
        coverage=[{'axis':a,'condition':c,'case':s,'status':'PASS','valid_data':True,
                   'repetitions':3,'operating_point_hash':canonical_hash('synthetic:'+c)}
                  for a in p['axes'] for c in p['conditions'] for s in p['required_cases']]
        certs.append({
          'stage':stage,
          'started_at':'2026-09-29T10:00:00Z' if stage=='3a' else '2026-09-29T11:00:00Z',
          'finished_at':'2026-09-29T10:30:00Z' if stage=='3a' else '2026-09-29T11:30:00Z',
          'bench_certificate_hash':None,
          'program':'commissiond' if stage=='3a' else 'production',
          'route':'independent_shared_core' if stage=='3a' else 'normal_production_chain',
          'execution':'physical','shadow':False,'status':'PASS',
          'candidate_id':p['candidate_id'],**{k:p[k] for k in DIGEST_KEYS},
          'actual_parameters_hash':p['parameters_hash'],'binary_hash':canonical_hash('binary:'+stage),
          'trace_hashes':[canonical_hash('synthetic-trace:'+stage)],'unresolved_abort':False,
          'coverage':coverage,'production_load_verified':stage=='3b',
          'lifecycle_verified':stage=='3b','parity_passed':stage=='3b'})
    certs[1]['bench_certificate_hash']=canonical_hash(certs[0])
    return p,certs


def run_demo(output: Path):
    output.mkdir(parents=True,exist_ok=True)
    data,truth=synthetic_data()
    X,y=integral_rows(*data)
    fit=fit_initializer(X,y)
    pi=ideal_pi(fit.a,fit.b,5.)
    rho=spectral_radius(linear_discrete_matrix(fit.a,fit.b,pi['kp_A_s_per_rad'],pi['ki_A_per_rad'],.005))
    profile,certs=fixture_bundle()
    report={'mode':'SYNTHETIC_OFFLINE_ONLY','hardware_accessed':False,
      'plant_values_are':'invented_mathematical_example_not_real_motor_gains',
      'truth':truth,'fit':fit.__dict__,'ideal_pi_example':pi,'simplified_local_discrete_spectral_radius':rho,
      'gate_3a_only_rejection_reasons':bundle_reasons(profile,certs[:1]),
      'synthetic_metadata_pair_reasons':bundle_reasons(profile,certs),
      'physical_attestation':'not provided; metadata validator cannot establish physical truth'}
    (output/'synthetic_report.json').write_text(json.dumps(report,indent=2)+'\n')
    np.savetxt(output/'synthetic_trace.csv',np.column_stack(data),delimiter=',',
               header='t_s,q_rad,omega_rad_s,iq_A,direction',comments='')
    # Expose schema examples as SYNTHETIC, deliberately NOT promotable certificates.
    for cert in certs:
        cert['execution']='synthetic'
    certs[1]['bench_certificate_hash']=canonical_hash(certs[0])
    for name,obj in [('profile',profile),('3a_evidence',certs[0]),('3b_evidence',certs[1])]:
        (output/f'{name}.json').write_text(json.dumps(obj,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('reports/demo'))
    run_demo(parser.parse_args().output)
