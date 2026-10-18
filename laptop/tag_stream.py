#!/usr/bin/env python3
"""Live drone camera with AprilTags drawn on it.

    python3 laptop/tag_stream.py --esp-ip 192.168.1.222

Each frame is checked for tags on the laptop, with the same detector code the
drone runs. Every tag the drone would accept gets a green outline, its id and
where it is from the drone centre (forward, right, down, metres; camera mount
in apriltag_host.py). Keys: q or Esc quit, s save a picture.

For tuning (rejected tags, the drone's own detections, HUD) use
tag_stream_legacy.py.
"""

import argparse
import select
import socket
import time

import cv2
import numpy as np

from apriltag_host import Detector, passes_gate
from camera_stream import COMMAND_PORT, KEEPALIVE_S, FrameAssembler, drain_socket
from protocol import CAMERA_STREAM_PORT, build_camera_stream_command

GREEN = (0, 255, 0)     # OpenCV colours are BGR
BLACK = (0, 0, 0)
FONT = cv2.FONT_HERSHEY_SIMPLEX


def to_view(x, y, scale):
    """AprilTag pixel (x, y) -> pixel in the scaled window.

    AprilTag puts (0, 0) at the corner of the top-left pixel, OpenCV at its
    centre, hence the - 0.5.
    """
    return round(x * scale - 0.5), round(y * scale - 0.5)


def draw_tag(view, det, scale):
    """Green outline, plus e.g. "5  fwd 1.02 right +0.10 down 0.50 m" above it."""
    pts = [to_view(x, y, scale) for x, y in det.corners]
    cv2.polylines(view, [np.array(pts, np.int32)], True, GREEN, 2, cv2.LINE_AA)

    label = str(det.id)
    if det.t_body is not None:
        fwd, right, down = det.t_body
        label += f"  fwd {fwd:.2f} right {right:+.2f} down {down:.2f} m"
    # Keep the whole label inside the window.
    (text_w, _), _ = cv2.getTextSize(label, FONT, 0.5, 3)
    x = max(0, min(min(p[0] for p in pts), view.shape[1] - text_w))
    y = max(15, min(p[1] for p in pts) - 6)
    cv2.putText(view, label, (x, y), FONT, 0.5, BLACK, 3, cv2.LINE_AA)  # outline
    cv2.putText(view, label, (x, y), FONT, 0.5, GREEN, 1, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description="Live drone camera with AprilTags.")
    parser.add_argument("--esp-ip", required=True,
                        help="drone IP, 192.168.1.(200 + drone id)")
    parser.add_argument("--fps", type=int, default=0,
                        help="1-15 (default 0 = firmware's 10)")
    parser.add_argument("--quality", type=int, default=0,
                        help="JPEG quality 10-90 (default 0 = firmware's 60)")
    parser.add_argument("--scale", type=float, default=2.0,
                        help="window size multiplier (default 2)")
    args = parser.parse_args()
    try:
        start_cmd = build_camera_stream_command(True, args.fps, args.quality)
    except ValueError as exc:
        parser.error(str(exc))
    esp_ip = socket.gethostbyname(args.esp_ip)  # packets carry numeric IPs

    # Frames arrive on one UDP port; start/stop commands go out on another.
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)  # 1 MB
    rx.bind(("0.0.0.0", CAMERA_STREAM_PORT))
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    drone = (esp_ip, COMMAND_PORT)

    detector = Detector()           # compiles the drone's C code on first run
    assembler = FrameAssembler()    # joins UDP pieces back into whole JPEGs
    window = "Drone camera + AprilTags"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    print(f"Asking {esp_ip} for video. q or Esc quits, s saves.")

    next_keepalive = 0.0
    view = None
    try:
        while True:
            # The drone only streams while it keeps hearing from us.
            if time.monotonic() >= next_keepalive:
                next_keepalive = time.monotonic() + KEEPALIVE_S
                try:
                    tx.sendto(start_cmd, drone)
                except OSError as exc:      # e.g. not on the drone Wi-Fi
                    print(f"cannot reach drone: {exc}")

            select.select([rx], [], [], 0.01)   # wait up to 10 ms for data
            frame, _, _ = drain_socket(rx, assembler, esp_ip)
            assembler.expire(time.monotonic())  # forget frames missing a piece

            if frame is not None:
                gray = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8),
                                    cv2.IMREAD_GRAYSCALE)   # (240, 320) uint8
                if gray is not None:
                    tags = [d for d in detector.detect(gray) if passes_gate(d)]
                    h, w = gray.shape
                    size = (round(w * args.scale), round(h * args.scale))
                    view = cv2.resize(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR),
                                      size, interpolation=cv2.INTER_NEAREST)
                    for det in tags:
                        draw_tag(view, det, args.scale)
                    cv2.imshow(window, view)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):           # 27 = Esc
                break
            if key == ord("s") and view is not None:
                path = time.strftime("tag_stream_%Y%m%d_%H%M%S.png")
                cv2.imwrite(path, view)
                print(f"saved {path}")
    except KeyboardInterrupt:
        pass
    finally:
        try:
            tx.sendto(build_camera_stream_command(False), drone)  # stop stream
        except OSError:
            pass
        detector.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
