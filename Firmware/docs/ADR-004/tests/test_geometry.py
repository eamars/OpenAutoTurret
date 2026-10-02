from __future__ import annotations
import math
import random
import unittest
from reference.geometry import *


class GeometryTests(unittest.TestCase):
    def assert_vec(self,a,b,tol=1e-9):
        self.assertLess(norm(tuple(x-y for x,y in zip(a,b))),tol)

    def test_rotation_inverse_and_reflection_rejection(self):
        r=mm(rz(.9),mm(ry(-.4),rx(1.2)))
        require_rotation(r)
        for row,ref in zip(mm(r,inverse(r)),I3): self.assert_vec(row,ref)
        with self.assertRaises(ValueError): require_rotation(((-1.,0.,0.),(0.,1.,0.),(0.,0.,1.)))

    def test_matrix_inverse_nonrotation(self):
        k=((1100.,4.,620.),(0.,870.,340.),(0.,0.,1.))
        for row,ref in zip(mm(k,inverse(k)),I3): self.assert_vec(row,ref)

    def test_invalid_numbers_and_zero_directions(self):
        for v in [(0.,0.,0.),(math.nan,0.,1.),(math.inf,0.,1.),(True,0.,1.)]:
            with self.assertRaises(ValueError): unit(v)
        with self.assertRaises(ValueError): inverse(((0.,0.,0.),(0.,0.,0.),(0.,0.,0.)))

    def test_quaternion_sign_equivalence(self):
        q=(.1,.2,-.3,.9)
        for row,ref in zip(quat_matrix(q),quat_matrix(tuple(-x for x in q))): self.assert_vec(row,ref)
        with self.assertRaises(ValueError): quat_matrix((0.,0.,0.,0.))

    def test_quaternion_known_axis(self):
        q=(0.,0.,math.sin(.4),math.cos(.4))
        for row,ref in zip(quat_matrix(q),rz(.8)): self.assert_vec(row,ref)

    def test_base_gravity_removes_both_joint_rotations(self):
        rng=random.Random(4004)
        for _ in range(200):
            r_nb=mm(rz(rng.uniform(-3,3)),mm(ry(rng.uniform(-3,3)),rx(rng.uniform(-3,3))))
            r_ps=mm(rx(.17),rz(-.29))
            yaw,pitch=rng.uniform(-6,6),rng.uniform(-1.4,1.4)
            r_ns=mm(mm(r_nb,mm(rz(yaw),ry(pitch))),r_ps)
            expected=mv(transpose(r_nb),Z)
            self.assert_vec(base_up(r_ns,r_ps,yaw,pitch),expected)

    def test_game_yaw_drift_does_not_rotate_gravity(self):
        r_ps=rx(.3); r_ns=mm(ry(.5),rx(-.7))
        a=base_up(r_ns,r_ps,.8,-.4)
        b=base_up(mm(rz(2.1),r_ns),r_ps,.8,-.4)
        self.assert_vec(a,b)

    def test_relative_tare_would_erase_tilt(self):
        raw=ry(.4)
        correct=base_up(raw,I3,0.,0.)
        relative=base_up(I3,I3,0.,0.)
        self.assertGreater(norm(tuple(x-y for x,y in zip(correct,relative))),.3)

    def test_wrong_time_joint_pose_is_not_equivalent(self):
        r_ns=mm(ry(.4),mm(rz(.8),ry(.2)))
        correct=base_up(r_ns,I3,.8,.2)
        wrong=base_up(r_ns,I3,1.1,.2)
        self.assertGreater(norm(tuple(x-y for x,y in zip(correct,wrong))),.05)

    def test_horizontal_basis_poles_and_orientation(self):
        for u0 in [(0.,0.,1.),(0.,0.,-1.),(1.,0.,0.),(-1.,0.,0.),(.2,-.5,.7)]:
            u=unit(u0); e1,e2=horizontal_basis(u)
            self.assertAlmostEqual(dot(e1,u),0.,places=12)
            self.assertAlmostEqual(dot(e2,u),0.,places=12)
            self.assert_vec(cross(e1,e2),u)

    def test_level_and_constant_elevation_rays(self):
        rng=random.Random(4005)
        for _ in range(300):
            u=unit(tuple(rng.uniform(-1,1) for _ in range(3)))
            phi=rng.uniform(-10,10); beta=rng.uniform(-1,1)
            self.assertAlmostEqual(dot(u,level_ray(u,phi)),0.,places=12)
            self.assertAlmostEqual(dot(u,level_ray(u,phi,beta)),math.sin(beta),places=12)
            self.assertAlmostEqual(norm(level_ray(u,phi,beta)),1.,places=12)

    def test_inverted_heading_has_correct_physical_sign(self):
        upright=level_ray((0.,0.,1.),.3)
        inverted=level_ray((0.,0.,-1.),.3)
        self.assertGreater(upright[1],0)
        self.assertLess(inverted[1],0)

    def test_mounting_classes_and_hysteresis(self):
        self.assertEqual(mount_class((0,0,1)),'UPRIGHT')
        self.assertEqual(mount_class((0,0,-1)),'INVERTED')
        self.assertEqual(mount_class((1,0,0)),'SIDEWAYS')
        self.assertEqual(mount_class(None),'UNKNOWN')
        u=(math.cos(math.radians(5)),0,math.sin(math.radians(5)))
        self.assertEqual(mount_class(u),'SIDEWAYS')
        self.assertEqual(mount_class(u,'UPRIGHT'),'UPRIGHT')
        u2=(math.cos(math.radians(2)),0,math.sin(math.radians(2)))
        self.assertEqual(mount_class(u2,'UPRIGHT'),'SIDEWAYS')

    def test_nominal_forward_pitch_sign(self):
        self.assert_vec(optical_ray(0,0),(1.,0.,0.))
        self.assertLess(optical_ray(0,.1)[2],0)

    def test_fk_ik_general_extrinsics(self):
        rng=random.Random(4006)
        r_pc=mm(rx(.08),mm(ry(-.04),NOMINAL_R_PC))
        for _ in range(200):
            y=rng.uniform(-5.,5.); p=rng.uniform(-1.1,1.1)
            d=optical_ray(y,p,r_pc)
            candidates=ik_candidates(d,(-1.3,1.3),(-7.,7.),(y+.01,p+.01),r_pc)
            self.assertTrue(candidates)
            self.assert_vec(optical_ray(candidates[0].yaw,candidates[0].pitch,r_pc),d)
            self.assertAlmostEqual(candidates[0].yaw,y,places=8)
            self.assertAlmostEqual(candidates[0].pitch,p,places=8)

    def test_pitch_limit_can_refuse_level_ray(self):
        d=unit((1.,0.,1.))
        self.assertFalse(ik_candidates(d,(-.1,.1)))
        self.assertTrue(ik_candidates(d,(-1.,1.)))

    def test_pole_does_not_invent_yaw(self):
        self.assertEqual(ik_candidates((0,0,1),(-math.pi,math.pi)),[])

    def test_unwrapped_yaw_crosses_pi_continuously(self):
        seed=(math.pi-.03,0.)
        angles=[math.pi-.02, math.pi-.01,math.pi,math.pi+.01,math.pi+.02]
        previous=seed[0]
        for angle in angles:
            d=(math.cos(angle),math.sin(angle),0.)
            c=ik_candidates(d,(-.4,.4),seed=seed)[0]
            self.assertLess(abs(c.yaw-previous),.03)
            previous=c.yaw; seed=(c.yaw,c.pitch)
        self.assertGreater(previous,math.pi)

    def test_tilted_scan_requires_pitch_motion(self):
        u=unit((.25,-.15,.96)); ps=[]; seed=(0.,0.)
        for d in range(-70,71,2):
            target=level_ray(u,math.radians(d))
            c=ik_candidates(target,(-1.,1.),seed=seed)[0]
            seed=(c.yaw,c.pitch); ps.append(c.pitch)
            self.assertAlmostEqual(dot(u,optical_ray(c.yaw,c.pitch)),0.,places=10)
        self.assertGreater(max(ps)-min(ps),.2)

    def test_endpoints_alone_miss_internal_pitch_limit(self):
        u=unit((.5,0,math.sqrt(.75)))
        a=level_ray(u,-math.pi/2); b=level_ray(u,math.pi/2); mid=level_ray(u,0.)
        self.assertTrue(ik_candidates(a,(-.2,.2)))
        self.assertTrue(ik_candidates(b,(-.2,.2)))
        self.assertFalse(ik_candidates(mid,(-.2,.2)))

    def test_horizon_projection_matches_two_world_rays(self):
        u=unit((.2,-.1,1.)); r_bc=mm(mm(rz(.2),ry(-.1)),NOMINAL_R_PC)
        k=((1000.,0.,640.),(0.,800.,360.),(0.,0.,1.))
        l=horizon_line(u,r_bc,k)
        self.assertIsNotNone(l)
        for phi in [-.2,.1,.5]:
            c=mv(transpose(r_bc),level_ray(u,phi))
            p=mv(k,c); p=tuple(x/p[2] for x in p)
            self.assertAlmostEqual(dot(l,p),0.,places=8)

    def test_display_affine_inverse_transpose(self):
        u=unit((.2,-.3,1.)); k=((900.,0.,610.),(0.,1100.,330.),(0.,0.,1.))
        a=((-1.2,0.,1536.),(0.,.8,20.),(0.,0.,1.))
        l=horizon_line(u,NOMINAL_R_PC,k,a)
        for phi in [-.3,0.,.3]:
            c=mv(transpose(NOMINAL_R_PC),level_ray(u,phi))
            p=mv(k,c); p=tuple(v/p[2] for v in p); dp=mv(a,p)
            self.assertAlmostEqual(dot(l,dp),0.,places=8)

    def test_rotation_180_preserves_line_not_inverted_badge(self):
        k=((900.,0.,640.),(0.,900.,360.),(0.,0.,1.))
        u=unit((.1,.3,1.)); a=((-1.,0.,1280.),(0.,-1.,720.),(0.,0.,1.))
        l=horizon_line(u,NOMINAL_R_PC,k); l2=horizon_line(u,NOMINAL_R_PC,k,a)
        self.assertAlmostEqual(nearest_line_angle(reticle_angle(l2)-reticle_angle(l)),0.,places=12)
        self.assertNotEqual(mount_class(u),mount_class(tuple(-x for x in u)))

    def test_horizontal_mirror_reverses_cant_once(self):
        k=((900.,0.,640.),(0.,900.,360.),(0.,0.,1.))
        u=unit((.1,.3,1.)); a=((-1.,0.,1280.),(0.,1.,0.),(0.,0.,1.))
        one=reticle_angle(horizon_line(u,NOMINAL_R_PC,k))
        mirrored=reticle_angle(horizon_line(u,NOMINAL_R_PC,k,a))
        self.assertAlmostEqual(nearest_line_angle(one+mirrored),0.,places=12)

    def test_vertical_horizon_is_hidden(self):
        k=((900.,0.,640.),(0.,900.,360.),(0.,0.,1.))
        self.assertIsNone(horizon_line((1,0,0),NOMINAL_R_PC,k))
        self.assertIsNone(horizon_line((-1,0,0),NOMINAL_R_PC,k))
        self.assertIsNotNone(horizon_line((0,0,1),NOMINAL_R_PC,k))

    def test_modulo_pi_avoids_180_degree_jump(self):
        previous=math.radians(89)
        value=nearest_line_angle(math.radians(-89),previous)
        self.assertAlmostEqual(math.degrees(value),91.)

    def test_reticle_is_collinear_and_center_stays_fixed(self):
        center=(612.,347.); theta=.35
        segments=reticle_segments(center,theta,70.,16.)
        pts=segments[0]+segments[1]
        for x,y in pts:
            self.assertAlmostEqual((x-center[0])*math.sin(theta)-(y-center[1])*math.cos(theta),0.,places=10)
        self.assertAlmostEqual((pts[0][0]+pts[3][0])/2,center[0])
        self.assertAlmostEqual((pts[0][1]+pts[3][1])/2,center[1])
        self.assertAlmostEqual(math.hypot(pts[1][0]-pts[2][0],pts[1][1]-pts[2][1]),32.)

    def test_path_chain_rule_against_finite_difference(self):
        # q = (sin(phi), cos(phi)); phi = .3 + .4*t + .1*t*t + .02*t**3
        t=.6
        def phase(t): return .3+.4*t+.1*t*t+.02*t**3
        def q(t): return (math.sin(phase(t)),math.cos(phase(t)))
        p=phase(t); rate=.4+.2*t+.06*t*t; acc=.2+.12*t; jerk=.12
        d1=(math.cos(p),-math.sin(p)); d2=(-math.sin(p),-math.cos(p)); d3=(-math.cos(p),math.sin(p))
        v,a,j=path_chain_rule(d1,d2,d3,rate,acc,jerk)
        h=1e-3; minus,mid,plus=q(t-h),q(t),q(t+h)
        for i in range(2):
            self.assertAlmostEqual(v[i],(plus[i]-minus[i])/(2*h),places=6)
            self.assertAlmostEqual(a[i],(plus[i]-2*mid[i]+minus[i])/h**2,places=6)
            approx=(q(t+2*h)[i]-2*q(t+h)[i]+2*q(t-h)[i]-q(t-2*h)[i])/(2*h**3)
            self.assertAlmostEqual(j[i],approx,places=5)

    def test_stopping_interval_and_extra_clearance(self):
        self.assertTrue(stop_interval_inside(.5,(-.05,.2),(-1.,1.),.1))
        self.assertFalse(stop_interval_inside(.8,(-.05,.2),(-1.,1.),.1))
        # Current pose is inside, but braking may cross the reserve.
        self.assertFalse(stop_interval_inside(.85,(0.,.08),(-1.,1.),.1))
        self.assertFalse(stop_interval_inside(.95,(0.,0.),(-1.,1.),.1))

    def test_unbounded_is_explicit_and_bad_intervals_fail(self):
        self.assertTrue(stop_interval_inside(20.,(-2.,3.),None,.1))
        with self.assertRaises(ValueError): stop_interval_inside(0.,(.1,.2),(-1.,1.),.1)
        with self.assertRaises(ValueError): stop_interval_inside(0.,(-.1,.2),(-1.,1.),-.1)
        with self.assertRaises(ValueError): stop_interval_inside(0.,(-.1,.2),(1.,-1.),.1)

if __name__=='__main__':
    unittest.main()
