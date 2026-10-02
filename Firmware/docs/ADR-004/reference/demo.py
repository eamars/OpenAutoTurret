"""Generate a synthetic tilted horizontal path; never accesses devices."""
from __future__ import annotations
import json
import math
from pathlib import Path
from .geometry import *


def main() -> None:
    up=unit((0.25,-0.15,0.96))
    seed=(0.,0.); rows=[]
    for degrees in range(-60,61,10):
        ray=level_ray(up,math.radians(degrees))
        choices=ik_candidates(ray,(-math.radians(45),math.radians(45)),seed=seed)
        if not choices:
            rows.append({'heading_deg':degrees,'status':'NO_POINT_SOLUTION'})
            continue
        s=choices[0]; seed=(s.yaw,s.pitch)
        r_bc=mm(mm(rz(s.yaw),ry(s.pitch)),NOMINAL_R_PC)
        line=horizon_line(up,r_bc,((1000.,0.,640.),(0.,1100.,360.),(0.,0.,1.)))
        rows.append({'heading_deg':degrees,'yaw_deg':math.degrees(s.yaw),'pitch_deg':math.degrees(s.pitch),
          'horizontal_residual':dot(up,optical_ray(s.yaw,s.pitch)),
          'reticle_angle_deg':None if line is None else math.degrees(reticle_angle(line))})
    out={'synthetic_only':True,'hardware_qualified':False,'path_or_stop_certified':False,
         'up_B':up,'mounting_class':mount_class(up),'rows':rows,
         'note':'Point geometry only; no interval proof, timing, motor simulation or live execution.'}
    p=Path(__file__).resolve().parents[1]/'reports'/'synthetic_geometry.json'
    p.write_text(json.dumps(out,indent=2)+'\n')
    print(p)

if __name__=='__main__':
    main()
