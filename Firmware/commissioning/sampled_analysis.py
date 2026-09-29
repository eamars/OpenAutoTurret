"""Lifted sampled-data checks for declared asynchronous sensor cadence/filtering.

The periodic sampler is an explicit mathematical hypothesis. Actual irregular timing
is exercised by timestamped core replay, and must not be silently called periodic.
"""
from __future__ import annotations
import math
import numpy as np
from scipy.linalg import expm
from scipy.optimize import linear_sum_assignment
from .contracts import Reason,require


def delayed_plant(a,b,slope,dt,delay):
    """Exact ZOH split for the two commands straddling a fractional TX delay."""
    augmented=np.array([[0,1,0],[-slope/a,-b/a,1/a],[0,0,0]],float)
    whole=int(np.floor(delay/dt+1e-12));fraction=max(0.,delay-whole*dt)
    if fraction<1e-12:fraction=0.
    remainder=expm(augmented*(dt-fraction))
    early=expm(augmented*fraction)
    return (expm(augmented*dt)[:2,:2],remainder[:2,2],
            remainder[:2,:2]@early[:2,2],whole,int(fraction>0))


def periodic_observer(observer,dt,encoder_period,gyro_period,age):
    period=math.lcm(encoder_period,gyro_period)
    require(period<=64,Reason.MEASUREMENT_LIMITED,"sampling superperiod exceeds bounded solver domain")
    F=np.array([[1.,dt],[0.,1.]])
    Q=observer.process_variance*np.array([[dt**4/4,dt**3/2],[dt**3/2,dt**2]])
    P=np.diag([observer.initial_position_variance,observer.initial_velocity_variance])
    gains=[]
    for cycle in range(1000):
        before=P.copy();gains=[]
        for phase in range(period):
            P=F@P@F.T+Q;M=np.eye(2);K=np.zeros((2,2))
            for index,H,variance,active in (
                (0,np.array([1.,-age]),observer.encoder_variance+observer.process_variance*age**4/4,phase%encoder_period==0),
                (1,np.array([0.,1.]),observer.gyro_variance+observer.process_variance*age**2,phase%gyro_period==0)):
                if not active:continue
                gain=P@H/(H@P@H+variance);update=np.eye(2)-np.outer(gain,H)
                P=update@P;M=update@M;K=update@K;K[:,index]+=gain
            gains.append((M@F,K))
        if np.max(np.abs(P-before))<1e-14:break
    return gains


def lifted_loop(a,b,slope,ff_slope,values,observer,simulation,delay):
    dt=simulation.dt
    A,Bnew,Bold,whole,extra=delayed_plant(a,b,slope,dt,delay)
    drive_steps=whole+extra
    rx_steps=int(np.ceil(simulation.measurement_delay/dt-1e-12))
    require(max(drive_steps,rx_steps)<=200,Reason.ENVELOPE_LIMITED,"delay outside finite sampled model")
    gains=periodic_observer(observer,dt,simulation.encoder_period,simulation.gyro_period,rx_steps*dt)
    # x=[plant q/v, causal gyro-filter state, observer q/v, integral, e_previous,
    #    command history, measurement q/v history]
    n=7+drive_steps+2*rx_steps
    H=np.zeros(n);H[3:5]=[ff_slope-values["kp"]*values["kpos"],-values["kp"]];H[5]=1
    error=np.array([-values["kpos"],-1.])
    weight=dt/(simulation.gyro_filter_tau+dt)
    phases=[]
    for F,K in gains:
        def step(x,u):
            applied=x[7+whole-1] if whole else u
            old=x[7+whole] if extra else 0.
            plant=A@x[:2]+Bnew*applied+Bold*old
            measured=x[-2:] if rx_steps else plant
            gyro=x[2]+weight*(measured[1]-x[2])
            estimate=F@x[3:5]+K@np.array([measured[0],gyro])
            e=error@x[3:5]
            commands=np.r_[u,x[7:7+drive_steps-1]] if drive_steps else np.empty(0)
            observations=np.r_[plant,x[7+drive_steps:-2]] if rx_steps else np.empty(0)
            return np.r_[plant,gyro,estimate,x[5]+values["ki"]*dt*(e+x[6])/2,e,commands,observations]
        A0=np.column_stack([step(x,0.) for x in np.eye(n)])
        B0=step(np.zeros(n),1.)
        phases.append((A0,B0))
    period=len(phases);state=np.eye(n);inputs=np.zeros((n,period));C=[];D=[];closed=np.eye(n)
    for j,(A0,B0) in enumerate(phases):
        C.append(H@state);D.append(H@inputs)
        state=A0@state;inputs=A0@inputs;inputs[:,j]+=B0
        closed=(A0+np.outer(B0,H))@closed
    return state,inputs,np.asarray(C),np.asarray(D),closed,period


def sampled_margins(parts,dt):
    A,B,C,D,closed,period=parts
    radius=float(np.max(np.abs(np.linalg.eigvals(closed))))**(1/period)
    if radius>=1:return {"passed":False,"spectral_radius":radius}
    previous=None;best=None
    for points in (512,1024,2048,4096):
        w=np.geomspace(1e-5/dt,(np.pi/(period*dt))*(1-1e-8),points)
        z=np.exp(1j*w*period*dt)
        transfer=-(C@np.linalg.solve(z[:,None,None]*np.eye(len(A))-A,
                     np.broadcast_to(B,(len(z),)+B.shape))+D)
        eigen=np.linalg.eigvals(transfer)
        # Follow eigenvalue branches; sorting each frequency by real part would
        # manufacture or erase crossings when lifted harmonics exchange order.
        for k in range(1,len(eigen)):
            _,order=linear_sum_assignment(np.abs(eigen[k-1,:,None]-eigen[k,None,:]))
            eigen[k]=eigen[k,order]
        phase_margins=[];gain_margins=[]
        for branch in eigen.T:
            mag=np.abs(branch);phase=np.unwrap(np.angle(branch))
            for k in np.flatnonzero((mag[:-1]-1)*(mag[1:]-1)<=0):
                fraction=(1-mag[k])/(mag[k+1]-mag[k]) if mag[k+1]!=mag[k] else 0
                angle=phase[k]+fraction*(phase[k+1]-phase[k])
                phase_margins.append(abs(float(np.rad2deg(np.angle(-np.exp(1j*angle))))))
            for k in np.flatnonzero(np.imag(branch[:-1])*np.imag(branch[1:])<=0):
                ratio=-branch[k].imag/(branch[k+1].imag-branch[k].imag) if branch[k+1].imag!=branch[k].imag else 0
                value=branch[k]+ratio*(branch[k+1]-branch[k])
                if value.real<0:gain_margins.append(abs(float(20*np.log10(abs(value)))))
        best={"phase_margin_deg":min(phase_margins) if phase_margins else -180.,
              "gain_margin_db":min(gain_margins) if gain_margins else 300.,
              "spectral_radius":radius,"frequency_samples":points,"sampling_superperiod":period}
        current=np.array([best["phase_margin_deg"],best["gain_margin_db"]])
        if previous is not None and np.max(np.abs(current-previous))<.02:
            # Reserve the numerical convergence tolerance instead of crediting it.
            best["passed"]=bool(current[0]-.02>=50 and current[1]-.02>=6)
            return best
        previous=current
    return {**best,"passed":False,"reason":"unresolved sampled-data margin resolution"}
