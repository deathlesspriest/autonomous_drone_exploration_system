#!/usr/bin/env python3
"""CDE1302 sweep + live tag localization. New code; course files stay unchanged.

Run from repository root: python laptop/mini_project_mission.py --drone-id 2
Press M in the camera window only when READY and preflight checks are complete.
ESC/Q/Ctrl+C requests an abort; software recovery requires a working command link.
"""
# PROJECT ADDITION: this ENTIRE file is new relative to the course GitHub repo.
# It combines the lab exercises; imports marked REUSED call unchanged upstream code.
# PROJECT ADDITION: timing, threading and sockets let video and flight run together.
import argparse
import math
import json
from pathlib import Path
import select
import socket
import threading
import time

# PROJECT ADDITION: import the new conversion, timestamping and return-check helpers.
from mini_project_support import (MissionAbort, Pose, PoseHistory, Recorder,
                                  body_to_local_xy, require_home)

# PROJECT ADDITION: resolve default files relative to the repository, not the shell folder.
ROOT = Path(__file__).resolve().parents[1]


# CHANGED FROM GITHUB: validate finite coordinates and require a final (0,0) point.
# x,y parsing follows the lab format; inline # comments are also accepted.
def load_route(path):
    points = []
    for n, line in enumerate(Path(path).read_text(encoding='utf-8').splitlines(), 1):
        line = line.split('#', 1)[0].strip()
        if not line:
            continue
        bits = line.replace(',', ' ').split()
        if len(bits) != 2:
            raise ValueError(f'Waypoint line {n} needs x,y')
        point = tuple(map(float, bits))
        if not all(math.isfinite(v) for v in point):
            raise ValueError(f'Waypoint line {n} contains non-finite values')
        points.append(point)
    if not points or math.hypot(*points[-1]) > 1e-6:
        raise ValueError('Route must end at home (0,0)')
    return points


# PROJECT ADDITION: send flight commands through a checked UDP send. The original
# CommsNode.send_command() catches socket errors internally; here send failures
# must reach the mission's abort handler. UDP send success is still NOT a PX4 ACK.
class CommandLink:
    def __init__(self, drone_id, esp_ip, encode):
        self.drone_id, self.target, self.encode = drone_id, (esp_ip, 5006), encode

    def send_command(self, drone_id, command):
        if drone_id != self.drone_id:
            return False
        payload = self.encode(command)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            if sock.sendto(payload, self.target) != len(payload):
                raise OSError('Incomplete UDP command send')
        return True


# PROJECT ADDITION: one background worker owns flight commands while the main
# thread continuously receives images, detects tags, draws video and records poses.
class Flight:
    """Single flight-command owner. Camera runs on the main thread."""
    # PROJECT ADDITION: shared events coordinate aborts and detection recording.
    def __init__(self, args, comms, poses, recorder, stop, camera_age, commands):
        self.args, self.comms, self.poses, self.rec = args, comms, poses, recorder
        self.stop, self.camera_age = stop, camera_age
        self.Command, self.START, self.GOTO, self.LAND = commands
        self.done = threading.Event()
        self.started = threading.Event()
        self.normal_return = False
        self.collecting = threading.Event()
        self.exit_code = 1

    # PROJECT ADDITION: propagate command-send errors instead of claiming completion.
    def send(self, command):
        if not self.comms.send_command(self.args.drone_id, command):
            raise MissionAbort('Command could not be sent: drone IP unknown')

    # CHANGED FROM GITHUB: monitor camera loss as well as stale telemetry/abort requests.
    def healthy(self):
        if self.stop.is_set():
            raise MissionAbort('Operator/camera requested abort')
        now = time.monotonic()
        pose = self.poses.latest()
        if pose is None or now - pose.received_at > 3.0:
            raise MissionAbort('Telemetry missing/stale for more than 3 s')
        if self.camera_age() > 3.0:
            raise MissionAbort('No usable camera frame for more than 3 s')
        return pose

    # CHANGED FROM GITHUB: continue checking faults during takeoff/waypoint waits.
    def pause(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.healthy()
            time.sleep(0.05)

    # CHANGED FROM GITHUB: keep the 0.25 m arrival radius and fresh-packet check,
    # but abort on timeout instead of skipping a missed waypoint and continuing.
    def goto(self, x, y, index):
        self.healthy()
        self.rec.event('WAYPOINT_COMMAND', index=index, x=x, y=y)
        start = time.monotonic()
        self.send(self.Command(self.GOTO, goal_x=x, goal_y=y))
        self.last_goto_sent = time.monotonic()
        deadline = start + self.args.arrival_timeout
        while time.monotonic() < deadline:
            p = self.healthy()
            if p.received_at > start and math.hypot(p.x-x, p.y-y) <= 0.25:
                self.rec.event('WAYPOINT_REACHED', index=index, actual_x=p.x, actual_y=p.y)
                return
            # CHANGED FROM GITHUB: retry only while idle/arrived; the original uses
            # a latched saw_moving flag. No new GOTO is sent during active flight.
            # Re-send the same goal periodically, as UDP has no delivery guarantee.
            # Limit this to idle/arrived navigation states; do not restart active flight.
            if time.monotonic() - self.last_goto_sent >= 2.0 and self.nav_idle():
                self.send(self.Command(self.GOTO, goal_x=x, goal_y=y))
                self.last_goto_sent = time.monotonic()
            time.sleep(0.1)
        raise MissionAbort(f'Waypoint {index} ({x},{y}) not reached within {self.args.arrival_timeout}s')

    # PROJECT ADDITION: failed disk logging must not prevent an abort LAND attempt.
    def recovery_event(self, event, **details):
        try:
            self.rec.event(event, **details)
        except Exception as exc:
            print(f'LOG ERROR {exc}: {event} {details}', flush=True)

    # CHANGED FROM GITHUB: label normal versus abort landing; log all three sends.
    # This remains LAND IN PLACE and is not an automatic return-to-home command.
    def land(self, reason):
        self.recovery_event('LAND_COMMAND_ATTEMPT', reason=reason,
                       note='Command time is NOT touchdown time')
        # PROJECT ADDITION: one failed UDP send must not cancel the remaining retries.
        accepted = 0
        for i in range(3):
            try:
                self.send(self.Command(self.LAND))
                accepted += 1
                self.recovery_event('LAND_COMMAND_SENT', reason=reason, transmission=i+1)
            except Exception as exc:
                self.recovery_event('LAND_SEND_FAILED', transmission=i+1, reason=str(exc))
            time.sleep(0.1)
        if accepted == 0:
            raise MissionAbort('All LAND send attempts failed')

    # PROJECT ADDITION: connect the full sequence and gate normal LAND on fresh home telemetry.
    def run(self, route):
        try:
            p = self.healthy()
            require_home(p, time.monotonic())
            # PROJECT ADDITION: save the takeoff event and begin elapsed-time recording.
            self.rec.start()
            # Set before send: an exception after a UDP send may leave outcome uncertain.
            self.started.set()
            self.send(self.Command(self.START))
            self.rec.event('TAKEOFF_COMMAND_SENT')
            self.collecting.set()
            self.pause(getattr(self.args, 'takeoff_wait', 10.0))  # same timed takeoff wait as course script; not altitude feedback
            for i, (x, y) in enumerate(route, 1):
                self.goto(x, y, i)
                self.pause(0.5)
            # CHANGED FROM GITHUB: do not call a timed-out home waypoint a successful return.
            require_home(self.healthy(), time.monotonic())
            self.rec.event('HOME_POSITION_VERIFIED', radius_m=0.25)
            self.collecting.clear()
            self.rec.stop_detections()
            self.rec.summary()
            self.land('NORMAL_HOME_RETURN')
            self.normal_return = True
            self.exit_code = 0
            self.rec.event('LANDING_REQUESTED', note='Observe touchdown/disarm in QGC; mission completion is not verified by this program')
        # CHANGED FROM GITHUB: stop the route on failure, label ABORT, and attempt
        # in-place LAND. Never report that this recovery has returned home.
        except BaseException as exc:
            self.collecting.clear()
            self.rec.stop_detections()
            try:
                self.rec.event('MISSION_ABORT', reason=str(exc), returned_home=False)
                self.rec.summary()
            finally:
                if self.started.is_set():
                    try:
                        self.land('ABORT_LAND_IN_PLACE_RETURN_NOT_VERIFIED')
                    except Exception as land_error:
                        print(f'LAND COULD NOT BE SENT: {land_error}. Use the lab recovery procedure.', flush=True)
        finally:
            self.done.set()


# PROJECT ADDITION: unified entry point; camera starts before a single M-key start.
def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--drone-id', type=int, required=True)
    ap.add_argument('--esp-ip')
    ap.add_argument('--waypoints-file', type=Path, default=ROOT/'waypoints/cde1302_sweep_draft.txt')
    # CHANGED FROM GITHUB: 40 s per goal instead of the upstream default 25 s.
    ap.add_argument('--arrival-timeout', type=float, default=40.0)
    # PROJECT ADDITION: record location and measured downward-camera mounting.
    ap.add_argument('--log-dir', type=Path, default=ROOT/'mission_logs')
    ap.add_argument('--tag-size-m', type=float, default=0.12)
    ap.add_argument('--camera-offset', type=float, nargs=3, default=(0.02,0.0,0.0), metavar=('FORWARD','RIGHT','DOWN'))
    ap.add_argument('--camera-rotation-deg', type=float, default=0.0)
    ap.add_argument('--takeoff-wait', type=float, default=10.0)
    args = ap.parse_args()
    if not 0 <= args.drone_id <= 54:
        ap.error('Drone ID must be 0..54 for the lab IP convention')
    if not math.isfinite(args.arrival_timeout) or args.arrival_timeout <= 0:
        ap.error('Arrival timeout must be finite and positive')
    if not all(math.isfinite(v) for v in [args.tag_size_m,args.takeoff_wait,args.camera_rotation_deg,*args.camera_offset]) or args.tag_size_m<=0 or args.takeoff_wait<=0:
        ap.error('Camera values must be finite; tag size and takeoff wait must be positive')
    route = load_route(args.waypoints_file)
    args.esp_ip = args.esp_ip or f'192.168.1.{200+args.drone_id}'
    print(f'{len(route)} waypoints; home=(0,0); ESP={args.esp_ip}')
    print('Check actual PX4 axes and origin against the draft route before flight.')
    # REUSED FROM GITHUB (unchanged): detector, frame assembly, telemetry, packet
    # encoding, and tag drawing. New integration logic surrounds these imports.
    import cv2
    import numpy as np
    import apriltag_host
    from apriltag_host import Detector, passes_gate
    # CHANGED FROM GITHUB: configure the existing pose properties for 90 degrees,
    # before constructing a detector. This also corrects the reused video overlay.
    # No upstream file is edited. Zero image rotation means image top = forward.
    from mini_project_support import downward_camera_rotation
    apriltag_host.R_BODY_CAM = downward_camera_rotation(args.camera_rotation_deg)
    apriltag_host.CAM_POS_M = tuple(args.camera_offset)
    from camera_stream import FrameAssembler, drain_socket, KEEPALIVE_S
    from comms import CommsNode
    from protocol import (CAMERA_STREAM_PORT, CommandPacket, CMD_START, CMD_GOTO,
                          CMD_LAND, NAV_IDLE, NAV_ARRIVED, build_camera_stream_command, build_command)
    from tag_stream import draw_tag

    # PROJECT ADDITION: resolve one camera/command endpoint and allocate run records.
    esp = socket.gethostbyname(args.esp_ip)
    rec = Recorder(args.log_dir)
    print(f'Logs: {rec.directory}')
    poses, stop = PoseHistory(), threading.Event()
    state_lock = threading.Lock()
    state = dict(camera_time=0.0, nav=None, source=None)

    # PROJECT ADDITION: timestamp and buffer this drone's x/y/yaw for tag mapping.
    def on_telemetry(packet, source):
        if packet.drone_id == args.drone_id and source == esp:
            poses.add(Pose(time.monotonic(), packet.ned_x, packet.ned_y, packet.heading_rad))
            with state_lock:
                state['nav'], state['source'] = packet.nav_state, source

    # PROJECT ADDITION: share camera freshness with the flight worker.
    def camera_age():
        with state_lock:
            return time.monotonic() - state['camera_time']

    # PROJECT ADDITION: restrict repeated GOTO sends to non-moving navigation states.
    def nav_idle():
        with state_lock:
            return state['nav'] in (NAV_IDLE, NAV_ARRIVED)

    # REUSED FROM GITHUB: one telemetry receiver and the same zero-offset configuration.
    # PROJECT ADDITION: flight sends use CommandLink so UDP errors propagate.
    comms = CommsNode(listen_port=5005, cmd_port=5006, config_path=None)
    comms.set_nav_tags([], {})  # zero map offset; does NOT reset PX4's local origin
    comms.on_telemetry(on_telemetry)
    flight = Flight(args, CommandLink(args.drone_id, esp, build_command), poses, rec, stop, camera_age,
                    (CommandPacket, CMD_START, CMD_GOTO, CMD_LAND))
    flight.nav_idle = nav_idle
    worker = None
    rx = tx = detector = None
    stream_target = None
    try:
        stream_target = (esp, 5006)
        # REUSED FROM GITHUB: same detector, camera port, assembler and keepalive packet.
        detector = Detector(tag_size_m=args.tag_size_m)
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # CHANGED FROM GITHUB: do not enable SO_REUSEADDR for the camera socket.
        # Exclusive bind: a second camera program must fail rather than steal frames.
        rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        rx.bind(('0.0.0.0', CAMERA_STREAM_PORT))
        rx.setblocking(False)
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        assembler = FrameAssembler()
        start_stream = build_camera_stream_command(True, 0, 0)
        comms.start()
        # PROJECT ADDITION: one mission window provides READY/start/abort status.
        window = 'CDE1302 mission: M start, Q/ESC abort'
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        next_keepalive = 0.0
        view = np.zeros((480, 640, 3), dtype=np.uint8)
        rec.event('PROGRAM_READY', model='Level-flight yaw-only conversion; approximate image/telemetry timing',
                  tag_size_m=detector.tag_size_m, camera_pitch_deg=90,
                  camera_offset_m=args.camera_offset, image_rotation_deg=args.camera_rotation_deg)
        print('Camera starts automatically. Press M in its window when READY; Q/ESC aborts.')
        # CHANGED FROM GITHUB: keep the camera loop alive concurrently with flight.
        while not flight.done.is_set():
            now = time.monotonic()
            # REUSED FROM GITHUB: keep asking ESP32 for video and assemble UDP frames.
            if now >= next_keepalive:
                tx.sendto(start_stream, stream_target)
                next_keepalive = now + KEEPALIVE_S
            select.select([rx], [], [], 0.01)
            frame, _, _ = drain_socket(rx, assembler, esp)
            assembler.expire(time.monotonic())
            if frame is not None:
                received = time.monotonic()
                gray = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
                # PROJECT ADDITION: reject frames whose ESP age exceeds 500 ms.
                # Drop old frames. ESP age is capture -> end-send, not network delay.
                if gray is not None and 0 <= frame.age_ms <= 500:
                    detected_at = time.monotonic()
                    # REUSED FROM GITHUB: same detection and hamming/margin quality gate.
                    tags = [d for d in detector.detect(gray) if passes_gate(d)]
                    processing_ms = (time.monotonic()-detected_at)*1000
                    # PROJECT ADDITION: associate the image with buffered telemetry.
                    # Use an approximate capture time in the laptop's monotonic clock.
                    # Unknown WiFi/socket delay remains; this is NOT clock synchronization.
                    capture_est = received - frame.age_ms/1000.0
                    pose = poses.nearest(capture_est, max_difference=0.25)
                    with state_lock:
                        state['camera_time'] = received
                    view = cv2.resize(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), (640,480))
                    for d in tags:
                        draw_tag(view, d, 2.0)
                        # PROJECT ADDITION: only record mission detections with valid pose/time data.
                        if not flight.collecting.is_set():
                            continue
                        if flight.rec.start_mono is None or capture_est < flight.rec.start_mono:
                            continue  # do not count buffered pre-takeoff images
                        # PROJECT ADDITION: retain the ID even when localization is unavailable;
                        # never invent coordinates from an invalid pose or stale telemetry.
                        if pose is None or not d.pose_valid:
                            rec.event('TAG_ID_ONLY', tag_id=d.id, frame_id=frame.frame_id,
                                      reason='No time-matched telemetry' if pose is None else 'No valid relative pose')
                            continue
                        body, rotation = d.t_body, d.R_body
                        if body is None or rotation is None:
                            continue
                        if d.pose_err is None or not all(math.isfinite(v) for v in (*body, *rotation, d.decision_margin, d.pose_err)):
                            continue
                        # PROJECT ADDITION (LOCALIZATION): rotate forward/right by yaw,
                        # then add local drone x/y; save translation AND relative rotation.
                        x, y = body_to_local_xy(pose.x, pose.y, pose.yaw, body[0], body[1])
                        rec.detection(tag_id=d.id, local_x_m=x, local_y_m=y,
                                      forward_m=body[0], right_m=body[1], down_m=body[2],
                                      R_body_json=json.dumps(rotation), drone_x_m=pose.x,
                                      drone_y_m=pose.y, yaw_rad=pose.yaw, frame_id=frame.frame_id,
                                      frame_age_ms=frame.age_ms, capture_est_mono_s=capture_est,
                                      pose_received_mono_s=pose.received_at,
                                      pose_time_error_s=pose.received_at-capture_est,
                                      processing_ms=processing_ms, hamming=d.hamming,
                                      decision_margin=d.decision_margin, pose_err=d.pose_err)
            # PROJECT ADDITION: start only with fresh video/telemetry, matching ESP IP,
            # and a position near home; the original scripts did not combine these checks.
            p = poses.latest()
            with state_lock:
                matching_ip = state['source'] == esp
            ready = (p is not None and time.monotonic()-p.received_at < 0.5
                     and math.hypot(p.x,p.y) <= 0.25 and camera_age() < 0.5 and matching_ip)
            label = ('MISSION ACTIVE - Q/ESC abort' if worker else
                     'READY - press M to take off' if ready else
                     'WAIT: video + telemetry + home (0,0) + matching ESP IP')
            display = view.copy()
            cv2.putText(display, label, (8,465), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,255), 1)
            cv2.imshow(window, display)
            key = cv2.waitKey(1) & 0xff
            if key in (ord('q'), 27) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                stop.set()
                break
            # CHANGED FROM GITHUB: M in the camera window replaces the console MISSION-ID prompt.
            if key in (ord('m'), ord('M')) and worker is None:
                if not ready:
                    print('Not ready: check video, fresh telemetry, ESP IP and home coordinates.', flush=True)
                else:
                    worker = threading.Thread(target=flight.run, args=(route,), daemon=False)
                    worker.start()
    # PROJECT ADDITION: GUI/error/keyboard abort reaches the single flight worker.
    except KeyboardInterrupt:
        stop.set()
        print('Abort requested. Waiting for flight-command recovery.', flush=True)
    except Exception as exc:
        stop.set()
        rec.event('PROGRAM_ERROR', reason=str(exc))
    # PROJECT ADDITION: keep communication alive until any recovery LAND has been attempted.
    finally:
        stop.set()
        # Never close comms while the flight worker is trying to send fallback LAND.
        if worker is not None:
            while worker.is_alive():
                try:
                    worker.join(0.2)
                except KeyboardInterrupt:
                    print('Waiting for LAND attempt; use lab recovery if required.', flush=True)
        if tx is not None:
            if stream_target is not None:
                try:
                    tx.sendto(build_camera_stream_command(False), stream_target)
                except OSError:
                    pass
            tx.close()
        if rx is not None:
            rx.close()
        if detector is not None:
            detector.close()
        comms.stop()
        cv2.destroyAllWindows()
        rec.summary()
        # PROJECT ADDITION: never equate program exit with actual touchdown or mission finish.
        rec.event('PROGRAM_EXIT', normal_home_landing_requested=flight.normal_return,
                  touchdown_verified=False, note='Check actual touchdown/disarm in QGC. This is not a mission-finish timestamp.')
        rec.close()
    return flight.exit_code if worker else 1


if __name__ == '__main__':
    raise SystemExit(main())
