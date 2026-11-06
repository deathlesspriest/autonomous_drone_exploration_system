#!/usr/bin/env python3
"""Lab 5 trajectory sequence with concurrent camera/localization/recording.
No firmware changes or MAVLink forwarding. Start with M in the camera window.
"""
import math
import csv
from pathlib import Path
import random
import time
import numpy as np
from mini_project_mission import Flight, main
from mini_project_support import MissionAbort
from send_trajectory import load_trajectory, UPLOAD_GAP_S
from protocol import NAV_TRAJ, TRAJ_MAX_PTS, build_traj_packets, build_traj_start

class Lab5Sweep(Flight):
    @staticmethod
    def load_trajectory(args):
        if not math.isfinite(args.dt) or not 0.001 <= args.dt <= 1:
            raise ValueError('dt must be finite, 0.001..1 seconds')
        if not math.isfinite(args.max_speed) or not 0 < args.max_speed <= 0.30:
            raise ValueError('max-speed must be positive and no more than 0.30 m/s')
        dt = round(args.dt * 1000) / 1000
        rows=[]
        for line in Path(args.trajectory).read_text(encoding='utf-8-sig').splitlines():
            line=line.split('#',1)[0].strip()
            if not line:
                continue
            fields=next(csv.reader([line]))
            if not rows and [v.strip().lower() for v in fields]==['t','x','y','z']:
                continue
            if len(fields)!=4:
                raise ValueError('CSV requires four columns: t,x,y,z')
            rows.append([float(v) for v in fields])
        data=np.asarray(rows)
        if len(rows)<2 or not np.isfinite(data).all():
            raise ValueError('CSV requires at least two finite rows')
        if abs(data[0,0])>1e-8 or np.linalg.norm(data[0,1:])>1e-8:
            raise ValueError('CSV must begin at t=0 and zero offsets')
        if abs(data[-1,0]/dt-round(data[-1,0]/dt))>1e-6:
            raise ValueError('CSV end time must align with dt')
        pts = load_trajectory(str(args.trajectory), dt)
        if len(pts) < 2 or len(pts) > TRAJ_MAX_PTS or not np.isfinite(pts).all():
            raise ValueError('Invalid trajectory samples or ESP32 point limit exceeded')
        if np.linalg.norm(np.diff(pts,axis=0),axis=1).max()/dt > args.max_speed+1e-6:
            raise ValueError('Trajectory exceeds speed limit')
        if np.max(np.abs(pts[:,2])) > 1e-6:
            raise ValueError('This sweep must hold takeoff height: z offsets must be zero')
        if (pts[:,0].min() < -1e-6 or pts[:,0].max() > 3.0+1e-6
                or np.abs(pts[:,1]).max() > .90+1e-6):
            raise ValueError('Route exceeds the inset 2.5m-wide x 3.5m-long room envelope')
        if np.linalg.norm(pts[-1]) > 1e-6:
            raise ValueError('Sweep must end at its starting position')
        print(f'ROUTE: {len(pts)} points, {(len(pts)-1)*dt:.1f}s playback')
        return pts

    def healthy(self):
        # Unlike the old integration, camera lag never cancels takeoff.
        if self.stop.is_set():
            raise MissionAbort('Operator or camera-program failure requested abort')
        pose = self.poses.latest()
        if pose is None or time.monotonic()-pose.received_at > 3:
            raise MissionAbort('ESP telemetry missing/stale for more than 3 seconds')
        lost = self.camera_age() > 3
        if lost != getattr(self, '_video_lost', False):
            self._video_lost = lost
            self.rec.event('VIDEO_STALE' if lost else 'VIDEO_RESUMED',
                           note='Flight continues; missing frames cannot produce tag results')
        return pose

    def pause(self, seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            self.healthy()
            time.sleep(min(.05, max(0, end-time.monotonic())))

    def wait_nav(self, timeout):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            self.healthy()
            stamp, nav = self.nav_snapshot()
            if time.monotonic()-stamp <= 3 and nav == NAV_TRAJ:
                return True
            time.sleep(.05)
        return False

    def run(self, pts):
        try:
            self.healthy()
            self.rec.start()
            self.started.set()
            self.send(self.Command(self.START))
            self.rec.event('TAKEOFF_COMMAND_SENT', wait_s=self.args.takeoff_wait,
                           note='Original Lab 5 timed takeoff; height is not verified')
            self.collecting.set()
            self.pause(self.args.takeoff_wait)
            # Same Lab 5 packet builder, 25 ms upload gap, retries and start command.
            dt_ms = round(self.args.dt*1000)
            traj_id = random.randint(1,255)
            packets = build_traj_packets(traj_id, pts)
            start = build_traj_start(traj_id, dt_ms)
            for attempt in range(1,6):
                self.rec.event('TRAJECTORY_UPLOAD', attempt=attempt)
                for packet in packets:
                    self.healthy()
                    if not self.comms.send_raw(self.args.drone_id, packet):
                        raise MissionAbort('Trajectory upload: drone IP unknown')
                    self.pause(UPLOAD_GAP_S)
                if not self.comms.send_raw(self.args.drone_id, start):
                    raise MissionAbort('Trajectory start: drone IP unknown')
                self.rec.event('TRAJECTORY_START_COMMAND_SENT', attempt=attempt)
                if self.wait_nav(1.5):
                    break
            else:
                raise MissionAbort('Drone refused trajectory after Lab 5 upload retries')
            duration = (len(pts)-1)*dt_ms/1000
            self.rec.event('TRAJECTORY_PLAYING', duration_s=duration)
            playback_started = time.monotonic()
            deadline = playback_started+duration+10
            while self.nav_snapshot()[1] == NAV_TRAJ and time.monotonic()<deadline:
                p = self.healthy()
                self.rec.event('FLIGHT_POSITION', x=p.x, y=p.y)
                self.pause(1)
            if self.nav_snapshot()[1] == NAV_TRAJ:
                raise MissionAbort('Trajectory playback timeout')
            if time.monotonic()-playback_started < duration-.2:
                raise MissionAbort('Trajectory ended before its planned duration')
            self.rec.event('TRAJECTORY_ENDED', nav_state=self.nav_snapshot()[1])
            # Verify return AFTER playback; no tight pre-upload drift restrictions.
            deadline = time.monotonic()+self.args.arrival_timeout
            since = None
            while time.monotonic()<deadline:
                p = self.healthy()
                if time.monotonic()-p.received_at < .5 and math.hypot(p.x,p.y)<=.25:
                    since = time.monotonic() if since is None else since
                    if time.monotonic()-since >= 1:
                        break
                else:
                    since = None
                self.pause(.05)
            else:
                raise MissionAbort('Trajectory ended but return to home was not verified')
            self.rec.event('HOME_POSITION_VERIFIED', radius_m=.25)
            self.land('NORMAL_HOME_RETURN')
            self.normal_return = True
            self.exit_code=0
        except BaseException as exc:
            self.recovery_event('MISSION_ABORT', reason=str(exc))
            if self.started.is_set():
                try:
                    self.land('ABORT_LAND_IN_PLACE')
                except Exception as land_error:
                    print(f'LAND FAILED: {land_error}; use lab recovery controls', flush=True)
        finally:
            self.collecting.clear()
            try:
                self.rec.stop_detections()
                self.rec.summary()
            finally:
                self.done.set()

if __name__ == '__main__':
    raise SystemExit(main(Lab5Sweep, trajectory_mode=True))
