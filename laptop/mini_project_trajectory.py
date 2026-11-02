#!/usr/bin/env python3
"""Method 2: Lab 5 CSV playback with Method 1 camera, localization and records.

Run: python laptop/mini_project_trajectory.py --drone-id 2 --trajectory trajectory/trajectory_room_2x2.csv
Camera starts automatically. Press M when READY. No separate flight/viewer script.
"""
# PROJECT ADDITION: this entire file is new; upstream flight code stays unchanged.
import csv
import math
import random
import time
from pathlib import Path
import numpy as np

from mini_project_mission import Flight, main
from mini_project_support import MissionAbort, require_home
# REUSED FROM GITHUB: the Lab 5 resampler, upload pacing and packet encoders.
from send_trajectory import load_trajectory as lab_load_trajectory, UPLOAD_GAP_S
from protocol import build_traj_packets, build_traj_start, TRAJ_MAX_PTS, NAV_TRAJ, NAV_ARRIVED


class TrajectoryFlight(Flight):
    def status(self):
        try:
            st = self.altitude_monitor.snapshot()
        except (AttributeError, RuntimeError) as exc:
            raise MissionAbort(f'PX4 altitude feed unavailable: {exc}') from exc
        now=time.monotonic()
        if st.sample_ms is None or now-st.position_time>.5 or now-st.heartbeat_time>2.:
            raise MissionAbort('Waiting for fresh PX4 altitude + heartbeat: enable QGC forwarding to 127.0.0.1:14551')
        pose=self.poses.latest()
        if pose is None or now-pose.received_at>.5:
            raise MissionAbort('Waiting for fresh ESP32 position telemetry')
        if math.hypot(pose.x-st.x,pose.y-st.y)>.15:
            raise MissionAbort('QGC and ESP32 positions disagree; check vehicle and local frame')
        return st

    def ready_extra(self):
        try:
            st=self.status()
            if st.armed:return False, 'Vehicle must be disarmed before starting'
            if abs(st.z)>.05:return False, f'Ground local z={st.z:.2f}; Lab 5 assumes near zero. Ask GA to check origin.'
            return True,''
        except MissionAbort as exc:return False,str(exc)

    def prepare_takeoff(self):
        ok,reason=self.ready_extra()
        if not ok:raise MissionAbort(reason)
        st=self.status()
        self.ground_z=st.z
        self.height_verified=False
        self.rec.event('TAKEOFF_BASELINE',ground_ned_z=st.z,target_ned_z=-.5,
                       note='Lab 5 takeoff target; measured altitude is an estimator reading')

    def healthy(self):
        pose=super().healthy()
        if self.started.is_set():
            st=self.status()
            if not (-.20<=pose.x<=1.25 and abs(pose.y)<=.55):
                raise MissionAbort('2x2 test position envelope exceeded; LAND in place requested')
            if getattr(self,'height_verified',False):
                if not st.armed or not st.offboard:
                    raise MissionAbort('Armed/OFFBOARD state lost during mission')
                if not .30<=self.ground_z-st.z<=.70:
                    raise MissionAbort('Measured height left 0.30..0.70 m band; LAND requested')
        return pose

    def wait_takeoff(self):
        start=time.monotonic();deadline=start+30.
        stable_since=None;last_sample=None;armed_seen=False;next_report=0.
        while time.monotonic()<deadline:
            self.healthy();st=self.status();now=time.monotonic()
            rise=self.ground_z-st.z
            if now>=next_report:
                self.rec.event('TAKEOFF_PROGRESS',rise_m=round(rise,3),ned_z=round(st.z,3),
                    target_m=.5,armed=st.armed,offboard=st.offboard,
                    stage='climbing/settling' if st.armed else 'waiting for arming')
                next_report=now+1.
            if armed_seen and not st.armed:raise MissionAbort('Disarmed before hover confirmation')
            armed_seen=armed_seen or st.armed
            if rise>.70:raise MissionAbort('Takeoff height overshoot; LAND requested')
            if st.armed and not st.offboard:raise MissionAbort('OFFBOARD lost during takeoff')
            if st.sample_ms!=last_sample:
                last_sample=st.sample_ms
                stable=(st.armed and st.offboard and .40<=rise<=.60
                    and abs(st.z+.5)<=.10 and abs(st.vz)<=.10 and math.hypot(st.vx,st.vy)<=.10)
                if not stable:stable_since=None
                else:
                    if stable_since is None:stable_since=now
                    # Preserve Lab 5 settling time and require measured stable hover.
                    if now-stable_since>=1.5 and now-start>=self.args.takeoff_wait:
                        self.height_verified=True
                        self.rec.event('HOVER_CONFIRMED',rise_m=round(rise,3),ned_z=round(st.z,3))
                        return
            time.sleep(.05)
        raise MissionAbort('No verified 0.5 m hover within 30 s; trajectory NOT started')

    # PROJECT ADDITION: reject malformed rows instead of silently dropping them.
    # This version is a horizontal sweep at the stock takeoff height (~0.5 m).
    @staticmethod
    def load_trajectory(args):
        if not math.isfinite(args.dt) or not .05<=args.dt<=.2 or abs(args.dt*1000-round(args.dt*1000))>1e-8:
            raise ValueError('--dt must be 0.05..0.2 s in whole milliseconds')
        rows=[]
        for line in Path(args.trajectory).read_text(encoding='utf-8-sig').splitlines():
            line=line.split('#',1)[0].strip()
            if not line:continue
            fields=next(csv.reader([line]))
            if not rows and [v.strip().lower() for v in fields]==['t','x','y','z']:continue
            if len(fields)!=4:raise ValueError('Trajectory needs exactly t,x,y,z columns')
            row=list(map(float,fields))
            if not all(math.isfinite(v) for v in row):raise ValueError('Trajectory contains non-finite values')
            rows.append(row)
        if len(rows)<2:raise ValueError('Trajectory needs at least two samples')
        data=np.array(rows)
        if np.any(np.diff(data[:,0])<=0):raise ValueError('Trajectory times must increase')
        if abs(data[0,0])>1e-8 or np.linalg.norm(data[0,1:])>1e-8 or np.linalg.norm(data[-1,1:])>1e-8:
            raise ValueError('CSV must start at t=0, x=y=z=0 and finish at x=y=z=0')
        if np.max(np.abs(data[:,3]))>1e-8:raise ValueError('This horizontal mission requires z=0 offsets throughout')
        if (data[:,1].min() < -1e-8 or data[:,1].max() > 1.+1e-8
                or np.max(np.abs(data[:,2])) > .30+1e-8):
            raise ValueError('This 2x2 TEST version only accepts x=0..1 m and y=-0.30..0.30 m')
        duration=data[-1,0]
        if abs(duration/args.dt-round(duration/args.dt))>1e-6:
            raise ValueError('Final time must align with --dt so resampling retains the home endpoint')
        # REUSED FROM GITHUB: resample CSV and subtract the first point.
        pts=lab_load_trajectory(str(args.trajectory),args.dt)
        if not 2<=len(pts)<=TRAJ_MAX_PTS:raise ValueError(f'Trajectory exceeds {TRAJ_MAX_PTS} samples; use a valid larger --dt')
        if not np.isfinite(pts).all() or np.linalg.norm(pts[-1])>1e-6:raise ValueError('Resampled route must finish at zero')
        speed=np.linalg.norm(np.diff(pts,axis=0),axis=1)/args.dt
        if speed.max()>.1+1e-6:raise ValueError('2x2 test reference speed exceeds 0.1 m/s')
        # PROJECT ADDITION: reject sharp velocity jumps, including start/stop.
        velocities=np.vstack((np.zeros(3),np.diff(pts,axis=0)/args.dt,np.zeros(3)))
        if np.linalg.norm(np.diff(velocities,axis=0),axis=1).max()/args.dt>.5+1e-6:
            raise ValueError('Reference acceleration exceeds 0.5 m/s^2')
        print(f'Trajectory: {len(pts)} samples, {duration:.1f} s, peak reference speed {speed.max():.3f} m/s')
        return pts

    def wait_state(self,predicate,timeout):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            self.healthy()
            stamp,nav=self.nav_snapshot()
            if time.monotonic()-stamp<.5 and predicate(stamp,nav):return stamp,nav
            time.sleep(.05)
        raise MissionAbort('Timed out waiting for trajectory state')

    def follow_route(self,pts):
        # CHANGED FROM GITHUB: relative playback starts at the current hover pose.
        # Require it close to the intended home so a shifted sweep is not hidden.
        origin=self.healthy()
        require_home(origin,time.monotonic(),radius=.05,stale_s=.5)
        if self.nav_snapshot()[1]==NAV_TRAJ:raise MissionAbort('Another trajectory is active')
        tid=random.randint(1,255)
        packets=build_traj_packets(tid,pts)
        # PROJECT ADDITION: upload twice before a single START. Never re-upload
        # after START uncertainty: that could restart playback from a new origin.
        for _ in range(2):
            for packet in packets:
                self.healthy();self.comms.send_raw(packet)
                time.sleep(UPLOAD_GAP_S)
        before=self.healthy()
        require_home(before,time.monotonic(),radius=.05,stale_s=.5)
        if math.hypot(before.x-origin.x,before.y-origin.y)>.03:
            raise MissionAbort('Hover moved during upload; trajectory not started')
        sent=time.monotonic()
        self.comms.send_raw(build_traj_start(tid,round(self.args.dt*1000)))
        self.rec.event('TRAJECTORY_START_COMMAND_SENT',trajectory_id=tid,
                       samples=len(pts),dt_s=self.args.dt,hover_x=before.x,hover_y=before.y)
        self.wait_state(lambda stamp,nav:stamp>sent and nav==NAV_TRAJ,3.)
        duration=(len(pts)-1)*self.args.dt
        deadline=sent+duration+10.
        while time.monotonic()<deadline:
            self.healthy()
            stamp,nav=self.nav_snapshot()
            if time.monotonic()-stamp>.5:raise MissionAbort('Trajectory state telemetry stale')
            if nav==NAV_ARRIVED:
                if stamp<sent+duration-.2:raise MissionAbort('Trajectory ended earlier than expected')
                break
            if nav!=NAV_TRAJ:raise MissionAbort('Trajectory interrupted before completion')
            time.sleep(.05)
        else:raise MissionAbort('Trajectory playback timeout')
        self.rec.event('TRAJECTORY_PLAYBACK_FINISHED',note='Playback time is not proof of home arrival')
        # CHANGED FROM GITHUB: require fresh physical home proximity for a dwell,
        # before the shared Method 1 sequence reports home and sends normal LAND.
        deadline=time.monotonic()+self.args.arrival_timeout
        since=None
        while time.monotonic()<deadline:
            pose=self.healthy()
            if pose.received_at>stamp and time.monotonic()-pose.received_at<.5 and math.hypot(pose.x,pose.y)<=.25:
                since=time.monotonic() if since is None else since
                if time.monotonic()-since>=1.:return
            else:since=None
            time.sleep(.05)
        raise MissionAbort('Trajectory finished but home arrival was not verified')


if __name__=='__main__':
    raise SystemExit(main(flight_class=TrajectoryFlight,trajectory_mode=True))
