"""Bounded supplied loaded-parent/native-pair probes; synthetic interface only."""
from dataclasses import asdict, replace
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.assembly_dynamics import ActuatorMap
from Firmware.commissioning.assembly_forecast import CausalParent, CoupledControllerPair, JointReference, JointSupport
from Firmware.commissioning.family_assets import native_parameter_document
from Firmware.commissioning.native import CObservation, Controller, Native
from Firmware.tools.adr0022_assembly_probe import fixture, serializable, zero_load
from Firmware.tools.adr0022_assembly_forecast_probe import native_fixture, vector_plan
from Firmware.tools.adr0022_closed_loop_estimator_probe import controller_parameters


INITIAL_Q = np.array([.3,.4])
BALANCED_COM_M = np.array([.001,.01,.0005])
CURRENT_CAP_A = .35
CONTROL_DT_S = .005
TRANSPORT_DELAY_S, GYRO_TAU_S, GYRO_DELAY_S = .0073,.012,.0041
Q_LOW,Q_HIGH=np.array([.2,.30]),np.array([.4,.45])
GRAVITY_TABLE_COMMAND_ERROR_A=1e-5


def write(path,value):
    path.write_text(json.dumps(serializable(value),indent=2,allow_nan=False,
        default=lambda x:x.tolist() if isinstance(x,np.ndarray) else x.item())+'\n')


def supplied_fixture(*, balanced):
    assembly=fixture()
    if balanced:
        assembly=replace(assembly,pitch_payload=replace(assembly.pitch_payload,com_in_body_m=BALANCED_COM_M),
            configuration_id='SYNTHETIC_NEAR_BALANCED_LOADED_TWO_JOINT_FIXTURE')
    actuator=ActuatorMap(torque_per_effective_amp_Nm=np.array([.5,.4]),
        command_gain=np.array([2.,1.5]),command_bias_A=np.array([.01,-.02]))
    return assembly,actuator


def equilibrium_current(assembly,actuator,q=INITIAL_Q):
    demand=assembly.current_demand(q,np.zeros(2),np.zeros(2),actuator,
        zero_load(assembly)(0.,q,np.zeros(2)),now_s=0.,max_load_age_s=0.)
    return demand


def equilibrium_probe(output):
    output.mkdir(parents=True,exist_ok=False)
    assert controller_parameters().current_cap==CURRENT_CAP_A
    declaration={'scope':'SUPPLIED_UPRIGHT_GRAVITY_EQUILIBRIUM_AND_EXISTING_CURRENT_ENVELOPE',
        'original_parent':serializable(asdict(fixture())),'separate_balanced_COM_m':BALANCED_COM_M.tolist(),
        'initial_q_rad':INITIAL_Q.tolist(),'initial_velocity_rad_s':[0.,0.],
        'existing_current_cap_A':CURRENT_CAP_A,'near_balanced_other_mass_inertia_mounting_gravity':'UNCHANGED',
        'source_filter_transport_s':[GYRO_DELAY_S,GYRO_TAU_S,TRANSPORT_DELAY_S],
        'holding_current':'computed from supplied G, nonunit gain/bias; constant accepted prehistory',
        'physical_identification':'NOT_RUN','physical_qualification':'NOT_RUN'}
    write(output/'predeclared-contract.json',declaration)
    rows=[]
    for balanced in(False,True):
        assembly,actuator=supplied_fixture(balanced=balanced)
        demand=equilibrium_current(assembly,actuator)
        feasible=bool(np.all(abs(demand['command_current_A'])<=CURRENT_CAP_A))
        row={'geometry':'NEAR_BALANCED_SUPPLIED' if balanced else 'ORIGINAL_SUPPLIED',
            'gravity_torque_Nm':assembly.dynamics(INITIAL_Q,np.zeros(2)).G_Nm,'effective_holding_A':demand['effective_current_A'],
            'host_holding_A':demand['command_current_A'],'status':'EQUILIBRIUM_CURRENT_FEASIBLE' if feasible else 'ENVELOPE_LIMITED',
            'motion_controller_trial':'NOT_RUN'}
        if feasible:
            initial=np.r_[INITIAL_Q,np.zeros(2)]
            parent=CausalParent(assembly,actuator,initial,zero_load(assembly),max_step_s=.0025,
                transport_delay_s=TRANSPORT_DELAY_S,gyro_filter_tau_s=GYRO_TAU_S,max_load_age_s=0.,
                prehistory_command_A=demand['command_current_A'])
            state=parent.advance(.020)
            row['known_equilibrium_parent_hold_max_state_error']=float(np.max(abs(state-initial)))
            assert row['known_equilibrium_parent_hold_max_state_error']<1e-12
        rows.append(row)
    write(output/'equilibrium-result.json',{'cases':rows,'original_rejection_preserved':True,
        'qualification':'CURRENT_ENVELOPE_INTERFACE_ONLY; NATIVE_GRAVITY_OBSERVER_MAP_NOT_YET_VERIFIED'})
    print((output/'equilibrium-result.json').read_text(),flush=True)
    return rows


def upright_geometry_probe(output):
    """Independent closed-form components for this upright diagonal fixture."""
    output.mkdir(parents=True,exist_ok=False)
    assembly,_=supplied_fixture(balanced=True)
    points=[INITIAL_Q,INITIAL_Q+np.array([.07,-.04])]
    points.extend(np.array([yaw,pitch]) for yaw in (.2,.3,.4) for pitch in (.30,.375,.45))
    declaration={'scope':'INDEPENDENT_SUPPLIED_UPRIGHT_G_AND_M01_COMPONENTS',
        'q_rad':[q.tolist() for q in points],'COM_m':BALANCED_COM_M.tolist(),'mass_kg':2.,
        'assumptions':'original supplied upright axes, diagonal payload COM inertia, unchanged rigid mounting',
        'equations':'x=cx*cos(pitch)+cz*sin(pitch); z=-cx*sin(pitch)+cz*cos(pitch); G=[0,-m*9.81*x]; M01=-m*cy*z',
        'error_gates':{'gravity_Nm':1e-12,'cross_inertia_kg_m2':1e-12},'physical_qualification':'NOT_RUN'}
    write(output/'predeclared-contract.json',declaration)
    rows=[]
    cx,cy,cz=BALANCED_COM_M
    for q in points:
        pitch=q[1];x=cx*math.cos(pitch)+cz*math.sin(pitch);z=-cx*math.sin(pitch)+cz*math.cos(pitch)
        gravity=np.array([0.,-2.*9.81*x]);cross=-2.*cy*z
        actual=assembly.dynamics(q,np.zeros(2))
        rows.append({'q_rad':q.tolist(),'independent_G_Nm':gravity.tolist(),'independent_M01_kg_m2':cross,
            'gravity_error_Nm':float(np.max(abs(actual.G_Nm-gravity))),
            'cross_inertia_error_kg_m2':float(abs(actual.M_kg_m2[0,1]-cross))})
    gravity_error=max(row['gravity_error_Nm'] for row in rows)
    cross_error=max(row['cross_inertia_error_kg_m2'] for row in rows)
    report={'passed':gravity_error<=1e-12 and cross_error<=1e-12,'points':rows,
        'max_gravity_error_Nm':gravity_error,'max_cross_inertia_error_kg_m2':cross_error,
        'scope':'two independently derived components of supplied upright geometry; full tensor/physical qualification not inferred'}
    write(output/'result.json',report)
    print(json.dumps(report,indent=2),flush=True)
    return report


def known_gravity_parameters(assembly,actuator):
    """A bounded copied scalar nominal table; full coupled FF remains exact."""
    params=native_fixture(assembly,INITIAL_Q,actuator)
    for axis,p in enumerate(params):
        p.model.q[:5]=np.linspace(Q_LOW[axis],Q_HIGH[axis],5)
        p.model.z[:]=np.linspace(Q_LOW[1-axis],Q_HIGH[1-axis],3)
        for iz,z in enumerate(p.model.z):
            q=INITIAL_Q.copy();q[1-axis]=z
            p.model.theta[iz]=assembly.dynamics(q,np.zeros(2)).M_kg_m2[axis,axis]/(
                actuator.torque_per_effective_amp_Nm[axis]*actuator.command_gain[axis])
            p.model.theta[3+iz]=0.
            for iq,x in enumerate(p.model.q[:5]):
                q[axis]=x
                current=equilibrium_current(assembly,actuator,q)['command_current_A'][axis]
                for direction in range(2):
                    entry=direction*15+iz*5+iq
                    p.model.theta[6+entry]=current
                    p.start_total[entry]=current
                    p.start_censored[entry]=0
        p.model.theta[36]=TRANSPORT_DELAY_S
    return params


def gravity_table_error(assembly,actuator,params):
    """Sampled validation only; continuous remainder is calculated separately."""
    errors=[]
    for axis,p in enumerate(params):
        worst=0.
        for x in np.linspace(Q_LOW[axis],Q_HIGH[axis],41):
            for z in np.linspace(Q_LOW[1-axis],Q_HIGH[1-axis],7):
                q=INITIAL_Q.copy();q[axis]=x;q[1-axis]=z
                exact=equilibrium_current(assembly,actuator,q)['command_current_A'][axis]
                rows=[np.interp(x,list(p.model.q[:5]),list(p.model.theta[6+iz*5:11+iz*5])) for iz in range(3)]
                approximate=np.interp(z,list(p.model.z),rows)
                worst=max(worst,abs(float(approximate)-exact))
        errors.append(float(worst))
    return errors


def upright_gravity_table_bound(assembly,actuator,params):
    """Independent continuous linear-interpolation remainder for upright G."""
    if not (np.array_equal(assembly.base_rotation_world,np.eye(3))
            and np.array_equal(assembly.yaw_axis_in_base,[0.,0.,1.])
            and np.array_equal(assembly.pitch_axis_in_yaw,[0.,1.,0.])
            and np.array_equal(assembly.gravity_world_m_s2,[0.,0.,-9.81])):
        raise ValueError('continuous gravity table remainder requires the declared upright geometry')
    cx,_,cz=assembly.pitch_payload.com_in_joint_m
    scale=float(abs(actuator.torque_per_effective_amp_Nm[1]*actuator.command_gain[1]))
    second=assembly.pitch_payload.mass_kg*9.81*math.hypot(cx,cz)/scale
    spacing=float(np.max(np.diff(list(params[1].model.q[:params[1].model.n]))))
    bound=second*spacing**2/8.
    return {'scope':'upright supplied G/bias only; no cable/friction/external loads; own-q table domain only',
        'yaw_command_remainder_A':0.,'pitch_command_second_derivative_bound_A_rad2':second,
        'maximum_pitch_node_spacing_rad':spacing,'pitch_command_remainder_A':bound,
        'equation':'|pitch command second derivative| <= m*g*hypot(cx,cz)/abs(kappa*gain); linear error <= M2*h^2/8',
        'other_axis_interpolation':'yaw G is constant; pitch G independent of yaw',
        'constant_command_bias_remainder_A':0.}


def gravity_table_bound_probe(output):
    output.mkdir(parents=True,exist_ok=False)
    assembly,actuator=supplied_fixture(balanced=True)
    params=known_gravity_parameters(assembly,actuator)
    write(output/'contract.json',{'scope':'FROZEN_UPRIGHT_CONTINUOUS_TABLE_REMAINDER_AND_SEPARATE_SAMPLED_VALIDATION',
        'table_q_rad':[list(p.model.q[:p.model.n]) for p in params],
        'declared_command_error_bound_A':GRAVITY_TABLE_COMMAND_ERROR_A,
        'friction_cable_external_load_Nm':[0.,0.,0.],'physical_qualification':'NOT_RUN'})
    analytic=upright_gravity_table_bound(assembly,actuator,params)
    sampled=gravity_table_error(assembly,actuator,params)
    report={'sampled_grid_max_error_A':sampled,'sampled_grid_size_per_axis':[41,7],
        'analytic_continuous_remainder':analytic,
        'continuous_bound_passed':bool(analytic['pitch_command_remainder_A']<=GRAVITY_TABLE_COMMAND_ERROR_A),
        'sampled_values_within_continuous_remainder':bool(sampled[0]<=1e-14 and sampled[1]<=analytic['pitch_command_remainder_A'])}
    write(output/'result.json',report);print(json.dumps(report,indent=2),flush=True)
    return report


def loaded_forecast(output,library,*,moving,step_s=.0025,move_s=2.):
    output.mkdir(parents=True,exist_ok=False)
    assembly,actuator=supplied_fixture(balanced=True)
    params=known_gravity_parameters(assembly,actuator)
    interpolation_error=gravity_table_error(assembly,actuator,params)
    analytic_remainder=upright_gravity_table_bound(assembly,actuator,params)
    assert analytic_remainder['pitch_command_remainder_A']<=GRAVITY_TABLE_COMMAND_ERROR_A
    assert max(interpolation_error)<=GRAVITY_TABLE_COMMAND_ERROR_A
    initial=np.r_[INITIAL_Q,np.zeros(2)]
    prehistory=equilibrium_current(assembly,actuator)['command_current_A']
    assert np.all(abs(prehistory)<=CURRENT_CAP_A)
    duration=.2+move_s+2. if moving else 2.
    times=np.arange(round(duration/CONTROL_DT_S)+1)*CONTROL_DT_S;times[-1]=duration
    support=JointSupport(q_min_rad=Q_LOW,q_max_rad=Q_HIGH,velocity_max_rad_s=np.full(2,.8),
        acceleration_max_rad_s2=np.full(2,np.deg2rad(30.)),max_reference_source_age_s=duration,
        qualification='SYNTHETIC_OFFLINE')
    trajectory='simultaneous-quintic-two-joint-displacement' if moving else 'stationary-known-loaded-equilibrium'
    def reference(now):
        if moving:return vector_plan(now,INITIAL_Q,move_s=move_s,duration_s=duration,configuration_id=assembly.configuration_id)
        return JointReference(q_ref_rad=INITIAL_Q,v_ref_rad_s=np.zeros(2),a_ref_rad_s2=np.zeros(2),
            time_s=now,source_time_s=0.,expires_at_s=duration,frame='logical-output-joint-rad',
            configuration_id=assembly.configuration_id,trajectory_id=trajectory,generation=1,fresh=True,valid=True)
    declaration={'scope':'NEAR_BALANCED_LOADED_SUPPLIED_PARENT_TWO_NATIVE_OWNERS_LOWER_CALLBACK',
        'parent':serializable(asdict(assembly)),'actuator':serializable(asdict(actuator)),
        'initial_q_v':initial.tolist(),'prehistory_command_A':prehistory.tolist(),'native_reset_command_A':prehistory.tolist(),
        'case':'simultaneous-quintic-full2s-stop' if moving else 'stationary-hold',
        'move_s':move_s if moving else None,'duration_s':duration,'control_dt_s':CONTROL_DT_S,
        'numerical_step_s':step_s,'gyro_filter_tau_s':GYRO_TAU_S,'gyro_source_delay_s':GYRO_DELAY_S,
        'transport_delay_s':TRANSPORT_DELAY_S,'gyro_period_cycles':4,'noise':'NONE',
        'nominal_table_role':'known bounded G/bias and diagonal M table for native scalar model/START; no friction kick',
        'observer_role':'unchanged native kinematic encoder/gyro observer; scalar gravity table is not an identified gravity observer',
        'full_coupled_feedforward':'exact supplied M/C/G at same-cycle native posterior vector, coherent planned v/a',
        'nominal_table_interpolation_error_A':interpolation_error,'nominal_table_error_bound_A':GRAVITY_TABLE_COMMAND_ERROR_A,
        'nominal_table_interpolation_validation':'sampled 41 own-q by7 other-q; continuous remainder separate',
        'nominal_table_analytic_continuous_remainder':analytic_remainder,
        'nominal_other_axis_posture':'planned other q; full coupling/actual geometry remains in vector callback',
        'q_support_rad':[Q_LOW.tolist(),Q_HIGH.tolist()],
        'limits':{'current_cap_A':.35,'slew_A_s':2.,'gains_observer_guards':'copied benchmark unchanged'},
        'gates':{'causal_replay':1e-12,'state_refinement':1e-8,'filtered_gyro_refinement':1e-6,
            'complete_shape_and_no_native_fault':True,'paired_output_before_ACK':True},
        'friction_cable_external_load_Nm':[0.,0.],'physical_identification':'NOT_RUN','physical_qualification':'NOT_RUN'}
    write(output/'predeclared-contract.json',declaration)
    parent=CausalParent(assembly,actuator,initial,zero_load(assembly),max_step_s=step_s,
        transport_delay_s=TRANSPORT_DELAY_S,gyro_filter_tau_s=GYRO_TAU_S,max_load_age_s=0.,prehistory_command_A=prehistory)
    native=Native(library);states=[initial.copy()];accepted=[prehistory.copy()]
    commands=[];posteriors=[];refs=[];sensors=[];ff=[];orders=[];readback=[]
    outcome={'status':'COMPLETED','time_s':duration}
    with Controller(native,params[0]) as yaw,Controller(native,params[1]) as pitch:
        for axis,core in enumerate((yaw,pitch)):
            core.reset(1.,initial[axis],0.,float(prehistory[axis]),generation=1,accepted_time=.9)
            readback.append(native_parameter_document(core.read_parameters())==native_parameter_document(params[axis]))
        pair=CoupledControllerPair((yaw,pitch),assembly,actuator,core_epoch_s=1.,frame='logical-output-joint-rad',
            trajectory_id=trajectory,max_load_age_s=0.,support=support)
        gyro_source=0.;gyro=np.zeros(2);gyro_seq=0
        for index,now in enumerate(times[1:],1):
            state=parent.advance(float(now));states.append(state)
            fresh=index%4==0
            if fresh:
                gyro_source=float(now)-GYRO_DELAY_S;gyro=parent.gyro_at(gyro_source);gyro_seq+=1
            observations=tuple(CObservation(now+1.,now+1.,gyro_source+1.,state[k],gyro[k],index,gyro_seq,1,1,int(fresh)) for k in range(2))
            ref=reference(float(now));before=len(pair.trace);prior_receipts=len(pair.receipts)
            try:
                result=pair.step(observations,ref,zero_load(assembly)(now,state[:2],state[2:]))
                command=pair.acknowledge(result,record_receipt=parent.accept_receipt)
            except Exception as exc:
                outcome={'status':'PAIRED_FAULT','time_s':float(now),'detail':str(exc),'both_inhibited':pair.faulted,
                    'individual_receipts_before_fault':len(pair.receipts)-prior_receipts,
                    'retained_current_A':parent.commands[-1].tolist()};break
            accepted.append(command);orders.append(pair.trace[before:])
            commands.append([[o.requested,o.limited,o.feedforward,o.integral,o.sequence,o.status,o.motion,o.start_increment] for o in result.outputs])
            posteriors.append(np.r_[[p.position for p in result.posteriors],[p.velocity for p in result.posteriors]])
            refs.append(np.r_[ref.q_ref_rad,ref.v_ref_rad_s,ref.a_ref_rad_s2]);ff.append(result.demand['command_current_A'])
            sensors.append(np.r_[now,state[:2],gyro,gyro_source,int(fresh)])
    t=times[:len(states)];states=np.asarray(states);accepted=np.asarray(accepted)
    def replay(h):
        other=CausalParent(assembly,actuator,initial,zero_load(assembly),max_step_s=h,
            transport_delay_s=TRANSPORT_DELAY_S,gyro_filter_tau_s=GYRO_TAU_S,max_load_age_s=0.,prehistory_command_A=prehistory)
        other.tx_times=parent.tx_times.copy();other.commands=[x.copy() for x in parent.commands]
        rows=[initial.copy()]
        for at in t[1:]:rows.append(other.advance(float(at)))
        return np.asarray(rows),other
    repeated,repeated_parent=replay(step_s);fine,fine_parent=replay(step_s/2)
    causal=float(np.max(abs(repeated-states)));refinement=float(np.max(abs(fine-repeated)))
    sensor_error=max((float(np.max(abs(repeated_parent.gyro_at(row[5])-row[3:5]))) for row in sensors),default=0.)
    filter_error=max((float(np.max(abs(fine_parent.gyro_at(row[5])-row[3:5]))) for row in sensors),default=0.)
    current=float(np.max(abs(accepted)));slew=float(np.max(abs(np.diff(accepted,axis=0))/CONTROL_DT_S)) if len(accepted)>1 else 0.
    order=all(x==['VECTOR_FF','PITCH_OUTPUT','YAW_OUTPUT','YAW_ACK','PITCH_ACK'] for x in orders)
    coupling=np.array([assembly.dynamics(x[:2],x[2:]).M_kg_m2[0,1] for x in states])
    completed=outcome['status']=='COMPLETED'
    stop_start=.2+move_s if moving else 0.
    stop=states[t>=stop_start,:2]
    drift=np.max(abs(stop-stop[0]),axis=0).tolist() if len(stop) else None
    passed=bool(completed and all(readback) and order and causal<=1e-12 and sensor_error<=1e-12
        and refinement<=1e-8 and filter_error<=1e-6 and current<=.35 and slew<=2.+1e-10)
    report={'outcome':outcome,'mathematical_interface_passed':passed,'actual_native_readback':readback,
        'maximum_successful_current_A':current,'maximum_successful_slew_A_s':slew,'paired_output_before_ACK':order,
        'causal_parent_replay_error':causal,'causal_gyro_replay_error':sensor_error,'state_refinement_error':refinement,
        'filtered_gyro_refinement_error':filter_error,'nonzero_M01_range_kg_m2':[float(coupling.min()),float(coupling.max())],
        'full2s_stop_samples_generated':completed and moving,'observed_stop_drift_rad':drift,
        'one_parent_initial_state':True,'distinct_native_handles':2,'successful_individual_receipts':len(pair.receipts),
        'full_motor_quality':'NOT_QUALIFIED; this supplied loaded parent has no identification/physical support',
        'physical_current_cutoff':'NOT_ESTABLISHED','physical_identification':'NOT_RUN','physical_qualification':'NOT_RUN',
        'deployment_authorized':False}
    write(output/'result.json',report)
    np.savez_compressed(output/'trace.npz',t=t,parent_q_v=states,accepted_A=accepted,commands=np.asarray(commands),
        posterior_q_v=np.asarray(posteriors),references=np.asarray(refs),control_sensors=np.asarray(sensors),vector_ff_A=np.asarray(ff),
        replay_q_v=repeated,refined_q_v=fine,successful_t=np.asarray(parent.tx_times),successful_A=np.asarray(parent.commands),
        per_axis_receipts=np.asarray([[r.axis,r.accepted_time_s,r.command_current_A,r.effective_current_A,r.torque_Nm,r.command_token,r.generation] for r in pair.receipts]))
    print(json.dumps(report,indent=2),flush=True)
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--stage',choices=('equilibrium','geometry','table-bound','hold','move'),default='equilibrium')
    parser.add_argument('--library',type=Path)
    parser.add_argument('--max-step-s',type=float,choices=(.0025,.00125,.000625),default=.0025)
    args=parser.parse_args()
    if args.stage=='equilibrium':equilibrium_probe(args.output)
    elif args.stage=='geometry':upright_geometry_probe(args.output)
    elif args.stage=='table-bound':gravity_table_bound_probe(args.output)
    else:
        if args.library is None:parser.error('native loaded forecast requires --library')
        loaded_forecast(args.output,args.library,moving=args.stage=='move',step_s=args.max_step_s)
