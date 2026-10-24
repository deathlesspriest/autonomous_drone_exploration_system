"""CDE1302 additions: level-flight coordinates, pose history and mission records.

No drone/network dependencies: this module can be tested on an ordinary laptop.
"""
# PROJECT ADDITION: this ENTIRE file is new relative to the course GitHub repo.
# Each section below explains the extra project behaviour; no upstream file is overwritten.
# PROJECT ADDITION: standard-library tools for coordinate history and persistent records.
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import csv
import json
import math
import statistics
import threading
import time


# PROJECT ADDITION: convert detector-relative translation to the required fixed local x/y.
# Original tag_stream.py only displays forward/right/down beside the video.
def body_to_local_xy(drone_x, drone_y, yaw_rad, forward, right):
    """Rotate body forward/right into local NED x/y, then translate (metres).

    Assumes negligible roll/pitch. t_body from apriltag_host already includes
    the camera mount rotation/offset: do NOT apply that transform a second time.
    """
    values = (drone_x, drone_y, yaw_rad, forward, right)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Non-finite position or heading')
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return drone_x + c * forward - s * right, drone_y + s * forward + c * right


# PROJECT ADDITION: save x/y/yaw with the laptop telemetry-receipt time.
@dataclass(frozen=True)
class Pose:
    received_at: float
    x: float
    y: float
    yaw: float


# PROJECT ADDITION: keep past poses so an older camera image uses a nearby pose,
# rather than the drone position after detection processing has finished.
class PoseHistory:
    def __init__(self):
        self._poses = deque(maxlen=300)  # about 30 s at 10 Hz
        self._lock = threading.Lock()

    # PROJECT ADDITION: ignore invalid telemetry; use a lock across the two threads.
    def add(self, pose):
        if not all(math.isfinite(v) for v in (pose.received_at, pose.x, pose.y, pose.yaw)):
            return
        with self._lock:
            self._poses.append(pose)

    # PROJECT ADDITION: reject a detection if no pose lies within 0.25 s of its estimate.
    def nearest(self, when, max_difference=0.25):
        with self._lock:
            if not self._poses:
                return None
            p = min(self._poses, key=lambda p: abs(p.received_at - when))
            return p if abs(p.received_at - when) <= max_difference else None

    # PROJECT ADDITION: flight control uses the newest pose, not image-matched history.
    def latest(self):
        with self._lock:
            return self._poses[-1] if self._poses else None


# PROJECT ADDITION: separate an unsuccessful mission from normal home return.
class MissionAbort(RuntimeError):
    pass


# CHANGED FROM GITHUB: independently verify the final home position before normal LAND.
# The original waypoint script sends LAND even when its final waypoint times out.
def require_home(pose, now, radius=0.25, stale_s=3.0):
    """Normal LAND is permitted only after fresh telemetry puts us near home."""
    if pose is None or now - pose.received_at > stale_s:
        raise MissionAbort('Cannot verify home: missing/stale telemetry')
    if math.hypot(pose.x, pose.y) > radius:
        raise MissionAbort('Home not reached; normal home landing refused')


# PROJECT ADDITION: original scripts do not persist the complete required detection/event data.
class Recorder:
    """One lock keeps camera/flight records consistent; flush every event."""
    # PROJECT ADDITION: create a new run folder and CSV/JSONL output headers.
    def __init__(self, root):
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.directory = Path(root) / ('mission_' + stamp)
        self.directory.mkdir(parents=True, exist_ok=False)
        self._lock = threading.RLock()
        self.start_mono = None
        self.start_wall = None
        self.tags = {}
        self.accept_detections = False
        self._json = (self.directory / 'events.jsonl').open('w', encoding='utf-8')
        self._events = (self.directory / 'events.csv').open('w', newline='', encoding='utf-8')
        self._ew = csv.DictWriter(self._events, fieldnames=['timestamp', 'elapsed_s', 'event', 'details_json'])
        self._ew.writeheader()
        self._detections = (self.directory / 'detections.csv').open('w', newline='', encoding='utf-8')
        self._dw = csv.DictWriter(self._detections, fieldnames=[
            'timestamp', 'elapsed_s', 'tag_id', 'local_x_m', 'local_y_m',
            'forward_m', 'right_m', 'down_m', 'R_body_json',
            'drone_x_m', 'drone_y_m', 'yaw_rad', 'frame_id', 'frame_age_ms',
            'capture_est_mono_s', 'pose_received_mono_s', 'pose_time_error_s',
            'processing_ms', 'hamming', 'decision_margin', 'pose_err'])
        self._dw.writeheader()

    # PROJECT ADDITION: start elapsed timing immediately before the takeoff send attempt.
    def start(self):
        with self._lock:
            self.start_mono = time.monotonic()
            self.start_wall = datetime.now().astimezone().isoformat(timespec='milliseconds')
            self.accept_detections = True
            self.event('TAKEOFF_COMMAND_ATTEMPT', note='Elapsed clock starts immediately before sending CMD_START')

    # PROJECT ADDITION: display AND save wall-clock timestamps plus elapsed seconds.
    def event(self, event, **details):
        with self._lock:
            now = datetime.now().astimezone().isoformat(timespec='milliseconds')
            elapsed = None if self.start_mono is None else round(time.monotonic() - self.start_mono, 3)
            row = dict(timestamp=now, elapsed_s=elapsed, event=event, **details)
            self._json.write(json.dumps(row, allow_nan=False) + '\n')
            self._json.flush()
            self._ew.writerow(dict(timestamp=now, elapsed_s=elapsed, event=event,
                                   details_json=json.dumps(details, allow_nan=False)))
            self._events.flush()
            print(f'{now} elapsed={elapsed} {event} {json.dumps(details)}', flush=True)
            return row

    # PROJECT ADDITION: close collection atomically before the final tag report.
    def stop_detections(self):
        with self._lock:
            self.accept_detections = False

    # PROJECT ADDITION: keep every accepted pose observation, not just one ID per tag.
    def detection(self, **data):
        with self._lock:
            if self.start_mono is None or not self.accept_detections:
                return
            row = self.event('TAG_DETECTED', **data)
            self._dw.writerow({k: row.get(k) for k in self._dw.fieldnames})
            self._detections.flush()
            self.tags.setdefault(data['tag_id'], []).append((data['local_x_m'], data['local_y_m']))

    # PROJECT ADDITION: report one median x/y per unique ID; retain raw observations separately.
    # A median reduces isolated outliers but does not remove calibration/timing bias.
    def summary(self):
        with self._lock:
            result = [dict(tag_id=i, local_x_m=statistics.median(v[0] for v in values),
                           local_y_m=statistics.median(v[1] for v in values), observations=len(values))
                      for i, values in sorted(self.tags.items())]
            with (self.directory / 'tag_summary.csv').open('w', newline='', encoding='utf-8') as f:
                w = csv.DictWriter(f, fieldnames=['tag_id', 'local_x_m', 'local_y_m', 'observations'])
                w.writeheader()
                w.writerows(result)
            self.event('TAG_SUMMARY', tags=result, method='Coordinate-wise median; no accuracy guarantee')
            return result

    # PROJECT ADDITION: close records after camera and flight work have stopped.
    def close(self):
        for f in (self._json, self._events, self._detections):
            f.close()


# PROJECT ADDITION: downward camera axes into body forward/right/down.
# Camera x=image right, y=image down, z=optical axis. At zero mounting rotation:
# body forward=-camera y, body right=camera x, body down=camera z.
# Positive mounting rotation turns image top toward body right, viewed from above.
def downward_camera_rotation(degrees):
    if not math.isfinite(degrees):
        raise ValueError('Camera rotation must be finite')
    a=math.radians(degrees);c,s=math.cos(a),math.sin(a)
    return ((-s,-c,0.0),(c,-s,0.0),(0.0,0.0,1.0))
