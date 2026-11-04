#!/usr/bin/env python3
"""Lab 5 trajectory sequence with concurrent camera/localization/recording.
No firmware changes or MAVLink forwarding. Start with M in the camera window.
"""
import math
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
        pts = load_trajectory(str(args.trajectory), dt)
        if len(pts) < 2 or len(pts) > TRAJ_MAX_PTS or not np.isfinite(pts).all():
            raise ValueError('Invalid trajectory samples or ESP32 point limit exceeded')
        if np.linalg.norm(np.diff(pts,axis=0),axis=1).max()/dt > args.max_speed+1e-6:
            raise ValueError('Trajectory exceeds speed limit')
        if np.max(np.abs(pts[:,2])) > 1e-6:
            raise ValueError('This sweep must hold takeoff height: z offsets must be zero')
        if (pts[:,0].min() < -1e-6 or pts[:,0].max() > 1.5+1e-6
                or np.abs(pts[:,1]).max() > .65+1e-6):
            raise ValueError('Route exceeds the inset 2x2 test-room envelope')
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
            deadline = time.monotonic()+duration+10
            while self.nav_snapshot()[1] == NAV_TRAJ and time.monotonic()<deadline:
                p = self.healthy()
                self.rec.event('FLIGHT_POSITION', x=p.x, y=p.y)
                self.pause(1)
            self.rec.event('TRAJECTORY_ENDED', nav_state=self.nav_snapshot()[1],
                           note='Lab 5 end condition; full path/home/touchdown not verified')
            self.land('LAB5_FINISH_LAND')
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
