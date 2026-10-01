"""Declared automatic selected-family local synthesis probe, synthetic only."""
from dataclasses import asdict, replace
import argparse
import copy
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.synthesis import selected_family_solve
from Firmware.commissioning.family_sampled_analysis import FamilySampledAnalysis, LocalGains, observer_tick
from Firmware.commissioning.model_family import FamilyNative
from Firmware.commissioning.native import Native, Controller, CObservation
from Firmware.tools.adr0022_family_sampled_probe import family_fixture, family_native_prefix, reference_delta

WN_GRID = (.5,1.,2.,4.)
DYNAMIC_POLICY = "SHARED_POSTERIOR_STEADY_STATE_REFERENCE"
DYNAMIC_DAMPING_GRID = (1.,1.5,2.)
DYNAMIC_THIRD_DAMPING_GRID = (2.5,3.,4.)
DYNAMIC_TOLERANCE_DAMPING_GRID = (1.,1.5,2.,2.5,3.,4.)


def save(path, record):
    path.write_text(json.dumps(record,indent=2)+"\n")


def fresh_directory(output):
    if output.exists() and any(output.iterdir()): raise ValueError("fresh evidence directory required")
    output.mkdir(parents=True,exist_ok=True)


def candidate_fixture(candidate):
    record=json.loads(candidate.read_text())
    if record['status'] != 'SELECTED_OFFLINE_LOCAL_CANDIDATE': raise ValueError('selected local candidate required')
    gains=LocalGains(**record['gains'])
    fixtures=[family_fixture(d,filtered=True,stribeck=True) for d in (-1,1)]
    declared=json.loads((candidate.parent/'predeclared-contract.json').read_text())
    if declared['model'] != fixtures[0][0].document() or declared['schedule'] != asdict(fixtures[0][4]) or \
            declared['local_points'] != [asdict(f[1]) for f in fixtures] or declared['nominal_gains'] != asdict(fixtures[0][5]):
        raise ValueError('candidate fixture differs from its original synthesis declaration')
    return gains,fixtures


def candidate_parameters(template,gains):
    parameters=copy.deepcopy(template)
    for name,value in asdict(gains).items(): setattr(parameters,name,value)
    return parameters


def dynamic_fixture(speed_deg_s=5., *, max_step=None):
    """The frozen root forecast model, core/observer and signed plateau points."""
    from Firmware.commissioning.family_analysis import SlidingPoint, SlidingSupport
    from Firmware.commissioning.family_sampled_analysis import SampleSchedule
    from Firmware.commissioning.motor_feedforward import parameters_for_family
    from Firmware.tools.adr0022_closed_loop_estimator_probe import controller_parameters, estimator_model
    model=replace(estimator_model(),actuator='first_order',actuator_tau=.0073,
        transport_delay=.0087,current_tau=.012,current_delay=.0029,friction='stribeck',
        stribeck_negative=.07,stribeck_positive=.05)
    if max_step is not None: model=replace(model,max_step=max_step)
    template=parameters_for_family(controller_parameters(),model,
        actuator_policy='STEADY_STATE_REFERENCE',actuation_memory_max_s=.03)
    nominal=LocalGains(template.kp,template.ki,template.kpos,template.kaw)
    schedule=SampleSchedule(dt_s=.005,encoder_period=1,gyro_period=4,
        encoder_age_s=0.,gyro_availability_age_s=0.,encoder_quantum_rad=0.,gyro_quantum_rad_s=0.,
        immediate_successful_ack=True,limits_inactive=True)
    fixtures=[]
    for d in (-1,1):
        point=SlidingPoint(q_rad=0.,v_rad_s=d*float(np.deg2rad(speed_deg_s)),
            configuration_id='synthetic-family-forecast',frame='output_shaft_rad')
        support=SlidingSupport(configuration_id=point.configuration_id,frame=point.frame,
            q_min_rad=-1.,q_max_rad=1.,v_min_rad_s=.02 if d>0 else -.3,
            v_max_rad_s=.3 if d>0 else -.02)
        fixtures.append((model,point,support,template,schedule,nominal,DYNAMIC_POLICY))
    return fixtures


def dynamic_local_probe(output, *, damping_grid=(1.,), phase_required_deg=50., gain_required_db=6.):
    fresh_directory(output)
    fixtures=dynamic_fixture()
    model,_,_,params,schedule,nominal,policy=fixtures[0]
    points,supports=tuple(f[1] for f in fixtures),tuple(f[2] for f in fixtures)
    declaration={'scope':'SYNTHETIC_DYNAMIC_SELECTED_FAMILY_GAIN_CURVE',
        'wn_grid_rad_s':list(WN_GRID),'damping_ratio_grid':list(damping_grid),
        'candidate_budget':len(WN_GRID)*len(damping_grid),
        'model':model.document(),'local_points':[asdict(p) for p in points],
        'sliding_supports':[asdict(s) for s in supports],'schedule':asdict(schedule),
        'nominal_gains':asdict(nominal),'ff_policy':policy,
        'FF_derivatives':'d/dqpost=Lprime/g; d/dvref=(B+Fprime)/g; d/daref=a/g; dynamic actuator/transport remain forward',
        'initializer':'signed minimum D; Kp=(2*zeta*a*wn-D)/g; Ki=a*wn^2/g; Kpos=wn/5; nominal Kaw unchanged',
        'selection':'fastest all-points passing bandwidth; lowest passing declared damping ratio breaks ties',
        'phase_required_deg':phase_required_deg,'gain_required_db':gain_required_db,'changes_defaults':False,
        'original_contract':{'phase_required_deg':50.,'gain_required_db':6.},
        'performance_contract_revised':phase_required_deg!=50. or gain_required_db!=6.,
        'native_prefix_and_full_shape':'separate gates; not replaced by local margin pass',
        'later_validation_speed_deg_s':8.,'physical_qualification':'NOT_RUN'}
    save(output/'predeclared-contract.json',declaration)
    began=time.monotonic()
    result=selected_family_solve(model,points,supports,params.observer,nominal,schedule,
        wn_grid=WN_GRID,ff_policy=policy,damping_ratio_grid=damping_grid,
        phase_required_deg=phase_required_deg,gain_required_db=gain_required_db)
    record={**result.document(),'elapsed_s':time.monotonic()-began,'ff_policy':policy}
    save(output/'local-result.json',record)
    print(json.dumps({'status':record['status'],'selected_wn_rad_s':record['selected_wn_rad_s'],
        'gains':record['gains'],'selected_damping_ratio':record['selected_damping_ratio'],
        'phase_required_deg':record['phase_required_deg'],'gain_required_db':record['gain_required_db'],
        'candidates':[{'wn':c['wn_rad_s'],'zeta':c['damping_ratio'],'passed':c['passed'],
            'points':[{'v_rad_s':p['v_rad_s'],'passed':p['passed'],
                'phase_margin_deg':p.get('diagnostics',{}).get('phase_margin_deg'),
                'gain_margin_db':p.get('diagnostics',{}).get('gain_margin_db'),
                'status':p.get('diagnostics',{}).get('status')} for p in c['points']]}
            for c in record['candidates']]},indent=2),flush=True)
    return result,fixtures


def dynamic_candidate_fixture(candidate):
    record=json.loads(candidate.read_text())
    if record['status'] != 'SELECTED_OFFLINE_LOCAL_CANDIDATE':
        raise ValueError('no selected dynamic candidate; native/full gates cannot use failed curve gains')
    fixtures=dynamic_fixture()
    declared=json.loads((candidate.parent/'predeclared-contract.json').read_text())
    if declared['model'] != fixtures[0][0].document() or declared['schedule'] != asdict(fixtures[0][4]) or \
            declared['local_points'] != [asdict(f[1]) for f in fixtures] or \
            declared['sliding_supports'] != [asdict(f[2]) for f in fixtures] or \
            declared['nominal_gains'] != asdict(fixtures[0][5]) or declared['ff_policy'] != DYNAMIC_POLICY:
        raise ValueError('selected dynamic candidate differs from frozen model/points/support/timing/policy')
    if record.get('phase_required_deg',50.) != declared['phase_required_deg'] or \
            record.get('gain_required_db',6.) != declared['gain_required_db']:
        raise ValueError('selected candidate margin requirements differ from declaration')
    return LocalGains(**record['gains']),fixtures


def dynamic_full_probe(output,library,candidate,*,validation=False):
    """Original complete shape, actual MotorFF reset/phase/fault path, own TX."""
    from Firmware.commissioning.family_forecast import forecast, synthetic_motion_metrics
    from Firmware.commissioning.motor_feedforward import BoundedStartPolicy, parameters_for_family
    from Firmware.commissioning.synthetic_family_oracle import independent_rollout
    from Firmware.tools.adr0022_family_forecast_probe import (contract, feedforward_fixture,
        velocity_plan, phased_planned_packet)
    fresh_directory(output)
    gains,fixtures=dynamic_candidate_fixture(candidate)
    model,_,_,template,_,_,_=fixtures[0]
    speed=8. if validation else 5.
    cases=[{'speed_deg_s':d*speed,'noisy':noisy,'seed':seed}
        for d in (-1,1) for noisy,seed in ((False,101),(True,101),(True,307))]
    limits={'q_rms_rad':1e-5,'gyro_rms_rad_s':1e-4,'current_rms_A':1e-9}
    start_fields=dict(configuration_id='synthetic-family-forecast',
        source='Frozen synthetic development60mA candidate; not qualified',q_min_rad=-1.,q_max_rad=1.,
        static_negative_interval_A=(.156,.164),static_positive_interval_A=(.156,.164),
        negative_excess_A=.060,positive_excess_A=.060,max_attempt_s=.200,
        max_command_dose_A2s=.026,max_attempts=1)
    declaration={'scope':'SYNTHETIC_FULL_SHAPE_SELECTED_CANDIDATE_THROUGH_REAL_MOTOR_FF',
        'candidate':str(candidate),'gains':asdict(gains),'model':model.document(),'library':str(library),
        'cases':cases,'validation_without_retuning':validation,'reference':'existing shaped_velocity + root phased_planned_packet',
        'reference_period_s':.005,'sample_control_gyro_period_s':[.001,.005,.020],
        'start_policy':start_fields,'actuator_policy':'STEADY_STATE_REFERENCE','actuation_memory_max_s':.03,
        'initial':[0.]*5,'successful_input_prehistory_A':0.,'initializations_each':1,
        'numerical_forward_gates':limits,'quality_predicates':'existing synthetic_motion_metrics; all original thresholds unchanged',
        'no_feedback_or_limiter_or_observer_defaults_changed':True,'physical_qualification':'NOT_RUN'}
    local=json.loads(candidate.read_text())
    declaration['local_margin_requirements']={key:local.get(key,default) for key,default in
        (('phase_required_deg',50.),('gain_required_db',6.))}
    declaration['planned_start_program']=None
    save(output/'predeclared-contract.json',declaration)
    native,family=Native(library),FamilyNative(library)
    results=[]
    for case in cases:
        plan=velocity_plan(case['speed_deg_s'])
        c=replace(contract(case['seed'],case['noisy'],round(float(plan[0][-1]),9)),
            frame='output_shaft_rad',trajectory_id='existing-shaped-velocity-plateau-and-fixed-stop')
        start_policy=BoundedStartPolicy(**start_fields)
        support,state=feedforward_fixture(c,model,actuator_policy='STEADY_STATE_REFERENCE',
            actuation_memory_max_s=.03,start_policy=start_policy)
        parameters=parameters_for_family(candidate_parameters(template,gains),model,
            actuator_policy=support.actuator_policy,actuation_memory_max_s=support.actuation_memory_max_s,
            start_policy=start_policy)
        reference=lambda now: phased_planned_packet(c,now,plan[0],plan[1],plan[2])
        began=time.monotonic()
        data=forecast(native,family,model,parameters,c,reference,initial=np.zeros(5),
            feedforward_support=support,feedforward_state=state)
        independent=independent_rollout(model,data['t'],data['tx_t'],data['tx_A'],np.zeros(5)).trace
        errors={'q_rms_rad':float(np.sqrt(np.mean((data['truth'][:,0]-independent[:,0])**2))),
            'gyro_rms_rad_s':float(np.sqrt(np.mean((data['truth'][data['v_new'],3]-independent[data['v_new'],3])**2))),
            'current_rms_A':float(np.sqrt(np.mean((data['truth'][:,4]-independent[:,4])**2)))}
        completed=data['report']['outcome']['status']=='COMPLETED'
        if completed:
            quality=synthetic_motion_metrics(data,c,**plan[2],gyro_bandwidth_hz=10.)
        else:
            quality={'status':'NOT_RUN','detail':'fault ended forecast before full original shape/quality window'}
        row={**case,'elapsed_s':time.monotonic()-began,'report':data['report'],
            'forward_errors_same_own_successful_input':errors,
            'forward_passed':all(errors[k]<=gate for k,gate in limits.items()),
            'complete_shape':completed,'original_quality':quality,
            'original_quality_passed':bool(quality.get('metrics',{}).get('passed',False)),
            'successful_ACK_count':len(data['tx_t'])-1,
            'reference_timing':plan[2]}
        row['numerical_interface_passed']=completed and row['forward_passed'] and data['report']['causal_sensor_replay_max_error']<=1e-12
        results.append(row)
        label=f"signed{case['speed_deg_s']:g}-{'noisy' if case['noisy'] else 'pristine'}-{case['seed']}"
        np.savez_compressed(output/f'{label}.npz',**{k:v for k,v in data.items() if isinstance(v,np.ndarray)},
            independent_same_realized_input=independent)
        save(output/'results.json',{'cases':results,'all_numerical_interface_passed':all(r['numerical_interface_passed'] for r in results),
            'all_original_quality_passed':all(r['original_quality_passed'] for r in results),
            'physical_qualification':'NOT_RUN'})
        print(json.dumps({**case,'outcome':row['report']['outcome'],'forward_passed':row['forward_passed'],
            'quality':row['original_quality'],'elapsed_s':row['elapsed_s']}),flush=True)
    return results


def local_probe(output, *, phase_required_deg=50., gain_required_db=6.):
    fresh_directory(output)
    fixtures = [family_fixture(d,filtered=True,stribeck=True) for d in (-1,1)]
    model,_,_,params,schedule,nominal,policy = fixtures[0]
    points,supports = tuple(f[1] for f in fixtures),tuple(f[2] for f in fixtures)
    declaration = {"scope":"SYNTHETIC_OFFLINE_LOCAL_GAIN_SYNTHESIS_DESIGN_PROBE",
        "wn_grid_rad_s":list(WN_GRID), "candidate_budget":len(WN_GRID),
        "local_points":[asdict(p) for p in points], "model":model.document(),
        "schedule":asdict(schedule), "nominal_gains":asdict(nominal),
        "initializer":"D=min(B+dF/dv), Kp=(2*a*wn-D)/actuator_gain, Ki=a*wn**2/actuator_gain, Kpos=wn/5, nominal Kaw unchanged",
        "phase_required_deg":phase_required_deg,"gain_required_db":gain_required_db,
        "same_PI_observer_limiter":True,"changes_defaults":False,"physical_qualification":"NOT_RUN"}
    (output/"predeclared-contract.json").write_text(json.dumps(declaration,indent=2)+"\n")
    started=time.monotonic()
    result=selected_family_solve(model,points,supports,params.observer,nominal,schedule,
        wn_grid=WN_GRID,ff_policy=policy,phase_required_deg=phase_required_deg,gain_required_db=gain_required_db)
    record={**result.document(),"elapsed_s":time.monotonic()-started}
    (output/"local-result.json").write_text(json.dumps(record,indent=2)+"\n")
    compact={"status":record["status"],"selected_wn_rad_s":record["selected_wn_rad_s"],"gains":record["gains"],
        "candidates":[{"wn":r["wn_rad_s"],"passed":r["passed"],
            "margins":[{k:p["diagnostics"].get(k) for k in ("phase_margin_deg","gain_margin_db","passed","status")}
                for p in r["points"]]} for r in record["candidates"]]}
    print(json.dumps(compact,indent=2),flush=True)
    return result,fixtures


def prefix_probe(output,library,candidate,*,inject=3800):
    fresh_directory(output)
    gains,fixtures=candidate_fixture(candidate)
    declaration={'scope':'SYNTHETIC_SELECTED_CANDIDATE_NATIVE_OBSERVABLE_PREFIX',
        'library':str(library),'candidate':str(candidate),'gains':asdict(gains),
        'injection_tick':inject,'total_ticks':inject+80,'eps':1e-5,'native_max_step_s':5e-6,
        'native_gate':1e-7,'changes_defaults':False,'hidden_native_state_jacobian':'NOT_VERIFIED',
        'plant':'preceding native causal state retained; zero-delay sensors only',
        'physical_qualification':'NOT_RUN'}
    save(output/'predeclared-contract.json',declaration)
    native,family_native=Native(library),FamilyNative(library)
    results=[]
    for fixture,direction in zip(fixtures,(-1,1)):
        model,point,support,template,schedule,_,policy=fixture
        params=candidate_parameters(template,gains)
        analysis=FamilySampledAnalysis(model,point,support,params.observer,gains,schedule,ff_policy=policy)
        plus,pp,pc=family_native_prefix(native,family_native,model,params,schedule,direction,policy,1,
            count=inject+80,inject=inject,eps=1e-5,reference_perturb=True,causal_tick=True)
        minus,mp,mc=family_native_prefix(native,family_native,model,params,schedule,direction,policy,-1,
            count=inject+80,inject=inject,eps=1e-5,reference_perturb=True,causal_tick=True)
        if pc != mc: raise ValueError('paired prefix clock schedule differs')
        observed=(plus-minus)/2e-5; measurements=(pp[:,:3]-mp[:,:3])/2e-5
        state=np.zeros(analysis.dimension); expected=[]; expected_measurements=[]
        P=np.diag([params.observer.initial_position_variance,params.observer.initial_velocity_variance])
        for k in range(1,inject+81):
            fresh=(k==1 or k%4==0) and k*schedule.dt_s-analysis.gyro_source_age_s>=0.
            F,K,P=observer_tick(params.observer,schedule.dt_s,P,encoder_fresh=True,gyro_fresh=fresh,
                encoder_age_s=schedule.encoder_age_s,gyro_age_s=analysis.gyro_source_age_s)
            perturb=np.array([np.sin(k*.47),.7*np.cos(k*.31)]) if k>=inject else np.zeros(2)
            state,out,measured=analysis.closed_step(state,k%analysis.period,observer_pair=(F,K),
                measurement_delta=perturb,reference_delta=reference_delta(k,inject) if k>=inject else (0.,0.,0.))
            expected.append(out); expected_measurements.append(measured)
        error=float(np.max(np.abs(observed-np.array(expected))))
        merror=float(np.max(np.abs(measurements-np.array(expected_measurements))))
        warmup=float(np.max(np.abs((pp[inject-21:inject-1,3]+mp[inject-21:inject-1,3])/2-point.v_rad_s)))
        q_inside=bool(np.all((pp[:,0]>support.q_min_rad)&(pp[:,0]<support.q_max_rad)&
            (mp[:,0]>support.q_min_rad)&(mp[:,0]<support.q_max_rad)))
        row={'direction':direction,'max_controller_observable_error':error,
            'max_measurement_error':merror,'pre_injection_velocity_deviation':warmup,
            'passed':max(error,merror)<1e-7 and warmup<1e-8 and q_inside,
            'all_positions_inside_supplied_sliding_support':q_inside,'clock':pc}
        results.append(row)
        np.savez_compressed(output/f'direction-{direction}.npz',
            t=np.arange(1,inject+81)*schedule.dt_s,observed=observed,expected=np.array(expected),
            observed_measurements=measurements,expected_measurements=np.array(expected_measurements))
        save(output/'results.json',{'cases':results,'all_passed':all(r['passed'] for r in results)})
        print(json.dumps(row),flush=True)
    return results


def raw_posterior_forecast(native,backend,model,params,c,reference,initial,initial_command):
    """Narrow moving-state math probe through the actual lower-level core hook.

    This does not instantiate or claim equivalence to MotorFeedforward's
    stationary-reset/fault protocol. One native core owns all feedback/ACK state.
    """
    from Firmware.commissioning.family_forecast import _reference
    c.validate()
    t=np.arange(int(round(c.duration_s/c.sample_dt_s))+1)*c.sample_dt_s
    t[-1]=c.duration_s
    tx_t,tx_A=[-.1],[initial_command]
    epoch=max(.1,model.gyro_delay)+1.
    commands,sensors,refs,posteriors=[],[],[],[]
    outcome={'status':'COMPLETED','time_s':c.duration_s}
    with Controller(native,params) as core:
        first=backend.rollout(model,t[:2],np.array(tx_t),np.array(tx_A),initial)
        expected_initial=np.r_[initial[:3],initial[3]+model.gyro_bias,model.current_gain*initial[4]+model.current_bias]
        if not np.allclose(first[0,:5],expected_initial,rtol=0.,atol=2e-15):
            raise ValueError('backend acquisition state or model observation map changed')
        core.reset(epoch,float(first[0,0]),float(first[0,1]),initial_command,accepted_time=epoch-.1)
        for k in range(c.control_period_samples,len(t),c.control_period_samples):
            now=float(t[k])
            trace=backend.rollout(model,t[:k+1],np.array(tx_t),np.array(tx_A),initial)
            gindex=k//c.gyro_period_samples*c.gyro_period_samples
            gsource=float(t[gindex]-model.gyro_delay)
            q,g=float(trace[k,0]),float(trace[gindex,3])
            observation=CObservation(now+epoch,now+epoch,gsource+epoch,q,g,k+1,gindex+1,1,1,int(gsource>=0.))
            packet=reference(now); ref=_reference(packet,now,c)
            def compute(posterior,r):
                if packet.trajectory_phase != 'TRACKING' or r.velocity <= 0 or posterior.velocity <= 0:
                    raise ValueError('raw probe leaves its declared positive sliding FF slice')
                load=model.load_offset+(model.load_slope*(posterior.position-model.q_origin) if model.load=='affine' else 0.)
                friction=model.coulomb_positive+(model.static_positive-model.coulomb_positive)*math.exp(-(r.velocity/model.stribeck_positive)**model.stribeck_power)
                value=(model.a*r.acceleration+load+model.viscous*r.velocity+friction-model.actuator_bias)/model.actuator_gain
                posteriors.append([now,posterior.position,posterior.velocity,posterior.encoder_time-epoch,
                    posterior.gyro_time-epoch,posterior.accepted_current,posterior.accepted_time-epoch])
                return value
            out=core.step_posterior_feedforward(observation,ref,compute)
            commands.append([now,out.requested,out.limited,out.feedforward,out.integral,out.position,out.velocity,
                out.sequence,out.status,out.motion,out.start_increment])
            sensors.append([now,q,g,gsource])
            refs.append([now,ref.position,ref.velocity,ref.acceleration])
            if out.status not in (0,3):
                outcome={'status':'CORE_FAULT','time_s':now,'core_status':int(out.status)}
                t=t[:k+1];break
            if not core.ack(out,accepted_time=now+epoch):
                outcome={'status':'ACK_REJECTED','time_s':now};t=t[:k+1];break
            tx_t.append(now);tx_A.append(float(out.limited))
    tx_t,tx_A=np.array(tx_t),np.array(tx_A)
    truth=backend.rollout(model,t,tx_t,tx_A,initial)
    gindices=np.arange(len(t))//c.gyro_period_samples*c.gyro_period_samples
    sensors=np.array(sensors).reshape(-1,4)
    indices=np.rint(sensors[:,0]/c.sample_dt_s).astype(int)
    causal=max(float(np.max(abs(sensors[:,1]-truth[indices,0]))),float(np.max(abs(sensors[:,2]-truth[gindices[indices],3]))))
    return {'t':t,'truth':truth,'q':truth[:,0],'v':truth[gindices,3],'current':truth[:,4],
        'q_new':np.ones(len(t),dtype=bool),'v_new':np.arange(len(t))%c.gyro_period_samples==0,
        'current_new':np.ones(len(t),dtype=bool),'tx_t':tx_t,'tx_A':tx_A,'initial':initial,
        'commands':np.array(commands),'references':np.array(refs),'causal_sensors':sensors,
        'ff_posteriors':np.array(posteriors),
        'report':{'outcome':outcome,'contract':asdict(c),'control_plant_backend':getattr(backend,'provenance','NATIVE_FAMILY'),
            'controller_policy':'ACTUAL_NATIVE_CORE_RAW_SHARED_POSTERIOR_FF_MATH_PROBE',
            'higher_level_adapter_gate':{'status':'UNSUPPORTED','reason':'INVALID_STATE',
                'detail':'MotorFeedforward retains its stationary-only reset requirement'},
            'higher_level_MotorFeedforward_moving_reset':'UNSUPPORTED; existing stationary-reset guard retained',
            'higher_level_reset_fault_equivalence':'NOT_VERIFIED','plant_state_initializations':1,
            'initial_state_and_model_observation_map_matched':True,
            'initial_velocity_supplied_rad_s':float(initial[1]),'core_clock_epoch_translation_s':epoch,
            'gyro_clock':'source time = fresh availability time - supplied model gyro_delay',
            'future_realized_inputs_substituted':False,'future_measured_state_substituted':False,
            'successful_ACK_count':len(tx_t)-1,'no_automatic_rearm':True,
            'causal_sensor_replay_max_error':causal,'maximum_successful_command_A':float(np.max(abs(tx_A))),
            'physical_stage3a':'NOT_RUN','physical_stage3b':'NOT_RUN','deployment_authorized':False}}


def nonlinear_probe(output,library,candidate,*,raw_posterior=False):
    from Firmware.commissioning.family_forecast import ForecastContract,IndependentForecastPlant
    from Firmware.commissioning.motor_feedforward import ReferencePacket,parameters_for_family,Failure
    from Firmware.commissioning.synthetic_family_oracle import independent_rollout
    fresh_directory(output)
    gains,fixtures=candidate_fixture(candidate)
    model,point,_,template,_,_,_=fixtures[1]
    c=ForecastContract(duration_s=.6,sample_dt_s=.001,control_period_samples=5,gyro_period_samples=20,
        encoder_noise_rad=0.,encoder_quantum_rad=0.,gyro_noise_rad_s=0.,current_noise_A=0.,seed=4103,
        configuration_id=point.configuration_id,trajectory_id='declared-sliding-shaped-bump',frame=point.frame)
    params=parameters_for_family(candidate_parameters(template,gains),model)
    velocity=.1; coefficient=.05
    friction=model.coulomb_positive+(model.static_positive-model.coulomb_positive)*math.exp(-(velocity/model.stribeck_positive)**model.stribeck_power)
    initial_command=(model.load_offset+model.viscous*velocity+friction-model.actuator_bias)/model.actuator_gain
    initial=np.array([0.,velocity,model.actuator_gain*initial_command+model.actuator_bias,velocity,
        model.actuator_gain*initial_command+model.actuator_bias])
    def reference(now):
        x=now/c.duration_s
        q=velocity*now+coefficient*x**3*(1-x)**3
        v=velocity+coefficient/c.duration_s*(3*x**2-12*x**3+15*x**4-6*x**5)
        a=coefficient/c.duration_s**2*(6*x-36*x**2+60*x**3-30*x**4)
        return ReferencePacket(q_ref_rad=q,v_ref_rad_s=v,a_ref_rad_s2=a,time_s=now,source_time_s=0.,
            expires_at_s=c.duration_s,frame=c.frame,configuration_id=c.configuration_id,
            trajectory_id=c.trajectory_id,generation=1,fresh=True,valid=True,trajectory_phase='TRACKING')
    declaration={'scope':'SYNTHETIC_NONLINEAR_SLIDING_FULL_FEEDBACK_COMPARISON_ONLY',
        'candidate':str(candidate),'library':str(library),'contract':asdict(c),'gains':asdict(gains),
        'model':model.document(),'initial':initial.tolist(),'prehistory_command_A':initial_command,
        'reference':'q=.1*t+.05*x^3*(1-x)^3, x=t/.6; analytic matching v/a, TRACKING phase',
        'existing_parameter_limits':{name:getattr(params,name) for name in ('current_cap','slew','integral_cap','velocity_cap','rest_speed','sustained_s','start_timeout_s')},
        'start_policy':'existing static point diagnostic retained; begins moving, no fabricated DEPARTURE',
        'numerical_forward_gates':{'q_rms_rad':1e-5,'gyro_rms_rad_s':1e-4,'current_rms_A':1e-9},
        'two_backends_generate_own_future_successful_TX':True,
        'controller_path':'RAW_NATIVE_SHARED_POSTERIOR_CALLBACK' if raw_posterior else 'MOTOR_FEEDFORWARD_ADAPTER',
        'MotorFeedforward_moving_reset':'UNSUPPORTED; stationary-only guard retained',
        'full_start_reversal_stop_qualification':'NOT_RUN','physical_qualification':'NOT_RUN'}
    save(output/'predeclared-contract.json',declaration)
    if not raw_posterior:
        rejected={'status':'UNSUPPORTED','reason':Failure.INVALID_STATE.value,
            'detail':'moving initial state is outside MotorFeedforward stationary-reset contract; guard retained',
            'cases':[],'all_passed':False,'physical_qualification':'NOT_RUN'}
        save(output/'results.json',rejected)
        print(json.dumps(rejected),flush=True)
        return []
    native,family_native=Native(library),FamilyNative(library)
    results=[]; forecasts=[]
    for name,backend in [('native',family_native),('independent',IndependentForecastPlant())]:
        began=time.monotonic()
        data=raw_posterior_forecast(native,backend,model,params,c,reference,initial,initial_command)
        other=(independent_rollout(model,data['t'],data['tx_t'],data['tx_A'],initial).trace if name=='native'
            else family_native.rollout(model,data['t'],data['tx_t'],data['tx_A'],initial))
        errors={'q_rms_rad':float(np.sqrt(np.mean((data['truth'][:,0]-other[:,0])**2))),
            'gyro_rms_rad_s':float(np.sqrt(np.mean((data['truth'][data['v_new'],3]-other[data['v_new'],3])**2))),
            'current_rms_A':float(np.sqrt(np.mean((data['truth'][:,4]-other[:,4])**2)))}
        row={'backend':name,'elapsed_s':time.monotonic()-began,'forward_errors_same_realized_input':errors,
            'forward_passed':all(errors[k]<=v for k,v in declaration['numerical_forward_gates'].items()),
            'report':data['report']}
        row['passed']=row['forward_passed'] and row['report']['outcome']['status']=='COMPLETED' and row['report']['causal_sensor_replay_max_error']<1e-12
        results.append(row); forecasts.append(data)
        np.savez_compressed(output/f'{name}.npz',**{k:v for k,v in data.items() if isinstance(v,np.ndarray)},comparison_same_input=other)
        save(output/'results.json',{'cases':results,'all_passed':all(r['passed'] for r in results)})
        print(json.dumps({'backend':name,'passed':row['passed'],'errors':errors,'outcome':row['report']['outcome'],'elapsed_s':row['elapsed_s']}),flush=True)
    if forecasts[0]['tx_A'].shape == forecasts[1]['tx_A'].shape and np.array_equal(forecasts[0]['tx_t'],forecasts[1]['tx_t']):
        own_input_difference=float(np.max(np.abs(forecasts[0]['tx_A']-forecasts[1]['tx_A'])))
    else: own_input_difference=None
    save(output/'results.json',{'cases':results,'all_passed':all(r['passed'] for r in results),
        'own_successful_TX_max_difference_A_diagnostic':own_input_difference,
        'qualification':'SYNTHETIC_SLIDING_DESIGN_PROBE_ONLY; full nonlinear/physical qualification NOT_RUN'})
    return results


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--stage",choices=('local','prefix','nonlinear','nonlinear-raw','dynamic-local','dynamic-damped-local','dynamic-damped-third','dynamic-tolerance-local','dynamic-full','dynamic-validation'),default='local')
    parser.add_argument("--library",type=Path)
    parser.add_argument("--candidate",type=Path)
    parser.add_argument("--injection-tick",type=int,default=3800)
    parser.add_argument("--phase-required-deg",type=float,default=50.)
    parser.add_argument("--gain-required-db",type=float,default=6.)
    args=parser.parse_args()
    margins=dict(phase_required_deg=args.phase_required_deg,gain_required_db=args.gain_required_db)
    if args.stage == 'local': local_probe(args.output,**margins)
    elif args.stage == 'dynamic-local': dynamic_local_probe(args.output,**margins)
    elif args.stage == 'dynamic-damped-local': dynamic_local_probe(args.output,damping_grid=DYNAMIC_DAMPING_GRID,**margins)
    elif args.stage == 'dynamic-damped-third': dynamic_local_probe(args.output,damping_grid=DYNAMIC_THIRD_DAMPING_GRID,**margins)
    elif args.stage == 'dynamic-tolerance-local': dynamic_local_probe(args.output,damping_grid=DYNAMIC_TOLERANCE_DAMPING_GRID,**margins)
    else:
        if args.library is None or args.candidate is None: parser.error('prefix/nonlinear require --library and --candidate')
        if args.stage=='prefix': prefix_probe(args.output,args.library,args.candidate,inject=args.injection_tick)
        elif args.stage.startswith('dynamic-'): dynamic_full_probe(args.output,args.library,args.candidate,
            validation=args.stage=='dynamic-validation')
        else: nonlinear_probe(args.output,args.library,args.candidate,raw_posterior=args.stage=='nonlinear-raw')
