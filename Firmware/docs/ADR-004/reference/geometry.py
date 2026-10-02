"""ADR-004 offline geometry, not production firmware or a motion planner.

R_XY maps coordinates in Y into X. Angles are radians. Camera axes are
right/down/forward; nominal base axes are forward/left/up. Matrices for display
may include reflections; matrices for physical rotations may not.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Sequence

Vec = tuple[float, float, float]
Mat = tuple[Vec, Vec, Vec]
I3: Mat = ((1.,0.,0.),(0.,1.,0.),(0.,0.,1.))
NOMINAL_R_PC: Mat = ((0.,0.,1.),(-1.,0.,0.),(0.,-1.,0.))
Z: Vec = (0.,0.,1.)
TAU = 2.0 * math.pi


def finite(*values: float) -> None:
    if any(isinstance(v, bool) or not math.isfinite(v) for v in values):
        raise ValueError('finite numeric values required')


def vector(v: Sequence[float]) -> Vec:
    if len(v) != 3:
        raise ValueError('3-vector required')
    finite(*v)
    return tuple(float(x) for x in v)  # type: ignore[return-value]


def dot(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(x*y for x,y in zip(vector(a),vector(b)))


def cross(a: Sequence[float], b: Sequence[float]) -> Vec:
    x,y,z=vector(a); u,v,w=vector(b)
    return y*w-z*v, z*u-x*w, x*v-y*u


def norm(a: Sequence[float]) -> float:
    return math.sqrt(dot(a,a))


def unit(a: Sequence[float]) -> Vec:
    n=norm(a)
    if n < 1e-12:
        raise ValueError('direction undefined for near-zero vector')
    return tuple(x/n for x in a)  # type: ignore[return-value]


def transpose(m: Mat) -> Mat:
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))  # type: ignore[return-value]


def mv(m: Mat, v: Sequence[float]) -> Vec:
    return tuple(dot(row,v) for row in m)  # type: ignore[return-value]


def mm(a: Mat, b: Mat) -> Mat:
    bt=transpose(b)
    return tuple(tuple(dot(r,c) for c in bt) for r in a)  # type: ignore[return-value]


def determinant(m: Mat) -> float:
    return dot(m[0],cross(m[1],m[2]))


def inverse(m: Mat) -> Mat:
    d=determinant(m)
    if abs(d)<1e-12:
        raise ValueError('singular matrix')
    cof=(cross(m[1],m[2]),cross(m[2],m[0]),cross(m[0],m[1]))
    return tuple(tuple(x/d for x in row) for row in transpose(cof))  # type: ignore[return-value]


def require_rotation(m: Mat, tolerance: float=1e-8) -> None:
    if len(m)!=3 or any(len(r)!=3 for r in m):
        raise ValueError('3x3 rotation required')
    check=mm(m,transpose(m))
    if abs(determinant(m)-1)>tolerance or any(abs(check[i][j]-I3[i][j])>tolerance for i in range(3) for j in range(3)):
        raise ValueError('proper orthogonal rotation required')


def rx(a: float) -> Mat:
    finite(a); c,s=math.cos(a),math.sin(a)
    return ((1.,0.,0.),(0.,c,-s),(0.,s,c))


def ry(a: float) -> Mat:
    finite(a); c,s=math.cos(a),math.sin(a)
    return ((c,0.,s),(0.,1.,0.),(-s,0.,c))


def rz(a: float) -> Mat:
    finite(a); c,s=math.cos(a),math.sin(a)
    return ((c,-s,0.),(s,c,0.),(0.,0.,1.))


def quat_matrix(xyzw: Sequence[float]) -> Mat:
    if len(xyzw)!=4:
        raise ValueError('quaternion must be XYZW')
    finite(*xyzw)
    n=math.sqrt(sum(x*x for x in xyzw))
    if n<1e-12:
        raise ValueError('zero quaternion')
    x,y,z,w=(v/n for v in xyzw)
    return ((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),
            (2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),
            (2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)))


def base_up(raw_r_ns: Mat, r_ps: Mat, yaw: float, pitch: float) -> Vec:
    """Requires time-aligned joints and untared, convention-verified raw pose."""
    require_rotation(raw_r_ns); require_rotation(r_ps)
    r_bs=mm(mm(rz(yaw),ry(pitch)),r_ps)
    return unit(mv(r_bs,mv(transpose(raw_r_ns),Z)))


def horizontal_basis(up: Sequence[float]) -> tuple[Vec, Vec]:
    u=unit(up)
    def projection(a: Vec) -> Vec:
        k=dot(a,u)
        return tuple(a[i]-k*u[i] for i in range(3))  # type: ignore[return-value]
    e=projection((1.,0.,0.))
    if norm(e)<0.1:
        e=projection((0.,1.,0.))
    e1=unit(e)
    return e1,unit(cross(u,e1))


def level_ray(up: Sequence[float], heading: float, elevation: float=0.) -> Vec:
    finite(heading,elevation)
    u=unit(up); e1,e2=horizontal_basis(u)
    return tuple(math.cos(elevation)*(math.cos(heading)*e1[i]+math.sin(heading)*e2[i])+math.sin(elevation)*u[i] for i in range(3))  # type: ignore[return-value]


def mount_class(up: Sequence[float] | None, previous: str='UNKNOWN') -> str:
    if up is None:
        return 'UNKNOWN'
    c=unit(up)[2]
    enter=math.sin(math.radians(7)); leave=math.sin(math.radians(3))
    if c>enter:
        return 'UPRIGHT'
    if c<-enter:
        return 'INVERTED'
    if previous=='UPRIGHT' and c>leave:
        return 'UPRIGHT'
    if previous=='INVERTED' and c<-leave:
        return 'INVERTED'
    return 'SIDEWAYS'


def optical_ray(yaw: float, pitch: float, r_pc: Mat=NOMINAL_R_PC) -> Vec:
    require_rotation(r_pc)
    return unit(mv(mm(mm(rz(yaw),ry(pitch)),r_pc),Z))


def equivalents(angle: float, bounds: tuple[float,float] | None, near: float) -> list[float]:
    finite(angle,near)
    if bounds is None:
        return [angle+TAU*math.floor((near-angle)/TAU+0.5)]
    lo,hi=bounds; finite(lo,hi)
    if lo>hi:
        raise ValueError('invalid interval')
    start=math.ceil((lo-angle-1e-12)/TAU); end=math.floor((hi-angle+1e-12)/TAU)
    if end-start>1000:
        raise ValueError('unreasonable offline enumeration; use continuous representation')
    return [angle+TAU*k for k in range(start,end+1)]


@dataclass(frozen=True)
class JointSolution:
    yaw: float
    pitch: float
    branch: int


def ik_candidates(ray_b: Sequence[float], pitch_bounds: tuple[float,float],
                  yaw_bounds: tuple[float,float] | None=None,
                  seed: tuple[float,float]=(0.,0.), r_pc: Mat=NOMINAL_R_PC) -> list[JointSolution]:
    """Point IK illustration only. Does not certify a continuous path or stopping."""
    require_rotation(r_pc)
    d=unit(ray_b); body=mv(r_pc,Z); radius=math.hypot(body[0],body[2])
    if math.hypot(d[0],d[1])<1e-9 or radius<1e-9:
        return []  # singular direction; no arbitrary yaw
    z=d[2]/radius
    if abs(z)>1+1e-12:
        return []
    root=math.acos(max(-1.,min(1.,z))); offset=math.atan2(body[0],body[2])
    found=[]
    for branch,sign in enumerate((1.,-1.)):
        p0=sign*root-offset
        for p in equivalents(p0,pitch_bounds,seed[1]):
            xr=body[0]*math.cos(p)+body[2]*math.sin(p)
            y0=math.atan2(d[1],d[0])-math.atan2(body[1],xr)
            for y in equivalents(y0,yaw_bounds,seed[0]):
                if norm(tuple(a-b for a,b in zip(optical_ray(y,p,r_pc),d)))<1e-8:
                    if not any(abs(y-v.yaw)<1e-10 and abs(p-v.pitch)<1e-10 for v in found):
                        found.append(JointSolution(y,p,branch))
    return sorted(found,key=lambda v:((v.yaw-seed[0])**2+(v.pitch-seed[1])**2,v.branch,v.yaw,v.pitch))


def horizon_line(up_b: Sequence[float], r_bc: Mat, k: Mat,
                 display: Mat=I3, vertical_exclusion: float=math.radians(2)) -> Vec | None:
    """Pinhole/rectified image. Real distortion needs the existing local projection."""
    require_rotation(r_bc); finite(vertical_exclusion)
    if not 0<=vertical_exclusion<math.pi/2:
        raise ValueError('invalid vertical exclusion')
    n=mv(transpose(r_bc),unit(up_b))
    if math.hypot(n[0],n[1])<=max(1e-12,math.sin(vertical_exclusion)):
        return None
    l=mv(transpose(inverse(k)),n)
    l=mv(transpose(inverse(display)),l)
    s=math.hypot(l[0],l[1])
    if s<1e-12:
        return None
    return tuple(v/s for v in l)  # type: ignore[return-value]


def nearest_line_angle(angle: float, previous: float=0.) -> float:
    finite(angle,previous)
    return previous+(angle-previous+math.pi/2)%math.pi-math.pi/2


def reticle_angle(line: Vec, previous: float=0.) -> float:
    a,b,_=vector(line)
    if math.hypot(a,b)<1e-12:
        raise ValueError('horizon direction undefined')
    return nearest_line_angle(math.atan2(-a,b),previous)


def reticle_segments(center: tuple[float,float], angle: float, half_length: float, half_gap: float):
    """Two collinear segments around a fixed center; no image or motor transforms."""
    finite(*center,angle,half_length,half_gap)
    if not 0<=half_gap<half_length:
        raise ValueError('require 0 <= half_gap < half_length')
    c,s=math.cos(angle),math.sin(angle)
    def p(t): return center[0]+t*c,center[1]+t*s
    return (p(-half_length),p(-half_gap)),(p(half_gap),p(half_length))


def path_chain_rule(d1: Sequence[float], d2: Sequence[float], d3: Sequence[float],
                    phase_rate: float, phase_accel: float, phase_jerk: float):
    """Pure chain rule for two joints; not a scalar trajectory timing solver."""
    if not len(d1)==len(d2)==len(d3)==2:
        raise ValueError('two joint derivative entries required')
    finite(*d1,*d2,*d3,phase_rate,phase_accel,phase_jerk)
    v=tuple(x*phase_rate for x in d1)
    a=tuple(d2[i]*phase_rate**2+d1[i]*phase_accel for i in range(2))
    j=tuple(d3[i]*phase_rate**3+3*d2[i]*phase_rate*phase_accel+d1[i]*phase_jerk for i in range(2))
    return v,a,j


def stop_interval_inside(q: float, stop_offsets: tuple[float,float],
                         soft_bounds: tuple[float,float] | None, extra_reserve: float) -> bool:
    """Check a supplied certified stopping interval, NOT compute physical braking.

    Caller must include acceleration, jerk, latency, model and actuation error in
    stop_offsets. None means explicitly certified unbounded, never unknown.
    """
    low,high=stop_offsets
    finite(q,low,high,extra_reserve)
    if low>0 or high<0 or extra_reserve<0:
        raise ValueError('stop offsets must enclose present pose; nonnegative reserve')
    if soft_bounds is None:
        return True
    lo,hi=soft_bounds; finite(lo,hi)
    if lo>hi:
        raise ValueError('invalid soft interval')
    return q+low>=lo+extra_reserve and q+high<=hi-extra_reserve
