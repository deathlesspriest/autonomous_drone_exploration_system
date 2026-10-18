#!/usr/bin/env python3
"""Live camera view with AprilTags drawn on it — camera_stream.py plus detection.

    python3 laptop/tag_stream_legacy.py --esp-ip 192.168.1.222 [--fps 10] [--quality 60]

Detects every frame with the drone's own detector (apriltag_host.py), so
margin, gate and pose match main/at_detect.c. Plain view: green outline, id
and camera-to-tag range per accepted tag.

--detail (key d) adds full labels, grey gate rejects (r), magenta crosses for
the drone's own :5008 detections, ~1 s behind the video (e), and the HUD (h).
If tag_debug.py holds :5008, the magenta layer is skipped.

Keys: q/Esc quit, s save (raw + annotated). --headless --frames N --save-dir
DIR writes annotated PNGs and one JSON line per frame; the log holds every
detection, whatever the toggles show.
"""

from __future__ import annotations

import argparse
import json
import re
import select
import socket
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

from apriltag_host import (
    GATE_MAX_HAMMING,
    GATE_MIN_MARGIN,
    PIXEL_CENTRE_OFFSET,
    POSE_ERR_LIMIT,
    ApriltagHostError,
    Detection,
    Detector,
    DetectorParams,
    gate_reason,
)
from camera_stream import (
    COMMAND_PORT,
    KEEPALIVE_S,
    STATS_S,
    FrameAssembler,
    drain_socket,
)
from protocol import (
    AT_DEBUG_PORT,
    CAMERA_STREAM_MAX_FPS,
    CAMERA_STREAM_MAX_QUALITY,
    CAMERA_STREAM_MIN_QUALITY,
    CAMERA_STREAM_PORT,
    CAMERA_VERSION,
    build_camera_stream_command,
    parse_at_debug,
)
from tag_debug import FRAME_STALL_S, LINK_TIMEOUT_S, load_nav_tag_ids

try:
    import cv2
    import numpy as np
except ImportError:                 # main() reports it
    cv2 = np = None

FRAME_W, FRAME_H = 320, 240         # QVGA, as at_detect.c and the stream use

# Colours, BGR.
COL_GOOD    = (0, 255, 0)
COL_REJECT  = (150, 150, 150)
COL_CORNER0 = (0, 200, 255)
COL_NAV     = (255, 255, 0)
COL_ESP     = (255, 0, 255)
COL_HUD     = (0, 255, 0)

FONT = 0            # cv2.FONT_HERSHEY_SIMPLEX, so tests import without cv2
LABEL_SCALE = 0.40  # at --scale 2; smaller windows shrink it
HUD_SCALE   = 0.45
MIN_TEXT_SCALE = 0.28


# --- ESP overlay state -------------------------------------------------------

@dataclass(frozen=True)
class EspMarker:
    """One detection the drone reported, with its own numbers for logging."""
    id:       int
    cx:       float     # image pixels, AprilTag convention
    cy:       float
    age_s:    float     # lag behind the live video
    gated:    bool      # passed the drone's gate (has a pose)
    margin:   float = 0.0
    hamming:  int   = 0
    pose_err: float = -1.0      # < 0 = no pose


class EspOverlay:
    """Latest AprilTag debug packet from the drone, and how old it is.

    Results lag the video ~1 s (500-650 ms per detection, then up to 100 ms to
    the next packet). Markers hide once the link is quiet for LINK_TIMEOUT_S or
    frame_ms is frozen for FRAME_STALL_S: a stale marker is worse than none.
    """

    def __init__(self) -> None:
        self.pkt = None             # latest AtDebugPacket
        self.rx_t = 0.0             # when it arrived
        self.frame_rx_t = 0.0       # when frame_ms last changed
        self.count = 0              # packets accepted
        self.frame_times: deque = deque(maxlen=12)
        self._last_frame_ms = None

    def update(self, pkt, now: float) -> bool:
        """Feed one parsed packet; True when it carried a new detector frame."""
        self.pkt = pkt
        self.rx_t = now
        self.count += 1
        new_frame = pkt.frame_ms != self._last_frame_ms
        if new_frame:
            self._last_frame_ms = pkt.frame_ms
            self.frame_rx_t = now
            if pkt.frame_ms != 0:
                self.frame_times.append(now)
        return new_frame

    def link_up(self, now: float) -> bool:
        return self.pkt is not None and (now - self.rx_t) < LINK_TIMEOUT_S

    def stalled(self, now: float) -> bool:
        """Packets arriving but the detector has not finished a new frame."""
        return (self.link_up(now) and self.pkt.frame_ms != 0
                and (now - self.frame_rx_t) > FRAME_STALL_S)

    def fresh(self, now: float) -> bool:
        return (self.link_up(now) and self.pkt.frame_ms != 0
                and not self.stalled(now))

    def result_age_s(self, now: float) -> float:
        """Newest result's lag behind live video (s): time since it arrived
        plus proc_ms, since the frame was captured before processing."""
        if self.pkt is None:
            return float("inf")
        return (now - self.frame_rx_t) + self.pkt.proc_ms / 1000.0

    def detector_fps(self, now: float):
        ft = self.frame_times
        if len(ft) < 2 or (now - ft[-1]) > 3.0:
            return None
        span = ft[-1] - ft[0]
        return (len(ft) - 1) / span if span > 0 else None

    def markers(self, now: float, show_rejected: bool = True) -> list[EspMarker]:
        if not self.fresh(now):
            return []
        age = self.result_age_s(now)
        out = []
        for d in self.pkt.detections:
            if not d.has_pose and not show_rejected:
                continue
            out.append(EspMarker(d.id, d.cx, d.cy, age, d.has_pose,
                                 d.margin, d.hamming, d.pose_err))
        return out

    def status(self, now: float) -> str:
        """One HUD line describing the drone's own detector."""
        if self.pkt is None:
            return f"esp :{AT_DEBUG_PORT} waiting"
        if not self.link_up(now):
            return f"esp LINK DOWN {now - self.rx_t:.1f}s (markers hidden)"
        if self.pkt.frame_ms == 0:
            return "esp waiting for first camera frame"
        if self.stalled(now):
            return (f"esp STALLED {now - self.frame_rx_t:.1f}s "
                    f"(markers hidden)")
        fps = self.detector_fps(now)
        fps_s = f"{fps:.1f} fps" if fps else "-- fps"
        latched = self.pkt.latched_id
        return (f"esp {fps_s} {self.pkt.proc_ms} ms/frame  "
                f"lag {self.result_age_s(now):.1f}s  "
                f"latched {'none' if latched < 0 else latched}")


# --- Drawing (no sockets, no window) -----------------------------------------

def to_view(x: float, y: float, scale: float) -> tuple[int, int]:
    """AprilTag pixel coordinates → pixel indices in the scaled view.

    AprilTag's (0, 0) is the top-left pixel's corner, OpenCV's its centre; this
    is the only place that half-pixel shift happens. Scale first: the shift is
    half a VIEW pixel.
    """
    return (int(round(x * scale - PIXEL_CENTRE_OFFSET)),
            int(round(y * scale - PIXEL_CENTRE_OFFSET)))


def view_geometry(shape, scale: float) -> tuple[int, int, float]:
    """(width, height, drawing scale) for a decoded frame of shape (H, W, ...).

    Uses the decoded size, not FRAME_W/FRAME_H, so overlays stay put if the
    drone ever sends something other than QVGA.
    """
    src_h, src_w = shape[:2]
    view_w = max(1, round(src_w * scale))
    view_h = max(1, round(src_h * scale))
    return view_w, view_h, view_w / src_w


def det_label(det: Detection, nav_ids, *, detail: bool = False) -> str:
    """Label for one laptop detection: id and camera-to-tag range (|t|).

    `detail` adds margin, hamming and [nav], and marks HIGH a pose error the
    firmware would discard (>= POSE_ERR_LIMIT).
    """
    if not detail:
        dist = f"  {det.range_m:.2f} m" if det.t is not None else ""
        return f"{det.id}{dist}"

    nav = " [nav]" if det.id in nav_ids else ""
    reason = gate_reason(det)
    if reason is not None:
        return (f"tag {det.id}?{nav} m{det.decision_margin:.1f} "
                f"h{det.hamming} {reason}")
    dist = ""
    if det.t is not None:
        dist = f" {det.range_m:.2f}m"
        if det.pose_err is not None and det.pose_err >= POSE_ERR_LIMIT:
            dist += f" err{det.pose_err:.2f} HIGH"
    return (f"tag {det.id}{nav} m{det.decision_margin:.0f} "
            f"h{det.hamming}{dist}")


def esp_label(m: EspMarker) -> str:
    """Label for one drone detection, e.g. "esp 5 m206 1.3s old"."""
    return (f"esp {m.id}{'' if m.gated else '?'} m{m.margin:.0f} "
            f"{m.age_s:.1f}s old")


def _text(view, s: str, x: int, y: int, colour, scale: float = LABEL_SCALE,
          thickness: int = 1, taken: list | None = None) -> None:
    """Outlined text, moved so the whole string fits in the view.

    OpenCV silently cuts an overhanging tail, and a cut label reads as another
    tag. `taken` holds boxes drawn this frame; on overlap, step down a row
    (3 tries at most).
    """
    h, w = view.shape[:2]
    (tw, th), _ = cv2.getTextSize(s, FONT, scale, thickness + 2)
    x = max(2, min(x, w - 2 - tw))
    y = max(th + 2, min(y, h - 3))
    if taken is not None:
        step = th + 3
        for _ in range(3):
            box = (x, y - th, x + tw, y)
            if not any(box[0] < o[2] and o[0] < box[2]
                       and box[1] < o[3] and o[1] < box[3] for o in taken):
                break
            y = min(y + step, h - 3)
        taken.append((x, y - th, x + tw, y))
    cv2.putText(view, s, (x, y), FONT, scale, (0, 0, 0), thickness + 2,
                cv2.LINE_AA)
    cv2.putText(view, s, (x, y), FONT, scale, colour, thickness, cv2.LINE_AA)


def _label_scale(scale: float) -> float:
    """Labels follow the window scale, so --scale 1 is not all text."""
    return max(MIN_TEXT_SCALE, min(0.45, LABEL_SCALE * scale / 2.0))


def _hud_scale(lines: list[str], width: int) -> float:
    """Shrink the HUD to fit the view, down to MIN_TEXT_SCALE; _hud_wrap()
    handles what still overflows."""
    if not lines:
        return HUD_SCALE
    longest = max(lines, key=len)
    text_w = cv2.getTextSize(longest, FONT, HUD_SCALE, 1)[0][0]
    if text_w <= width - 12:
        return HUD_SCALE
    return max(MIN_TEXT_SCALE, HUD_SCALE * (width - 12) / text_w)


def _hud_wrap(lines: list[str], width: int, scale: float) -> list[str]:
    """Wrap HUD lines still too wide at `scale`, on spaces, so none is cut."""
    limit = width - 12
    out: list[str] = []
    for line in lines:
        cur = ""
        # Keep separators: splitting on " " drops half of each double space.
        for sep, word in re.findall(r"( *)(\S+)", line):
            trial = f"{cur}{sep}{word}" if cur else word
            if cur and cv2.getTextSize(trial, FONT, scale, 1)[0][0] > limit:
                out.append(cur)
                cur = word              # the break replaces that separator
            else:
                cur = trial
        out.append(cur)
    return out


def annotate(view, pc_dets: list[Detection], esp_markers: list[EspMarker],
             hud: list[str], *, scale: float = 1.0, nav_ids=frozenset(),
             show_rejected: bool = True, detail: bool = False) -> None:
    """Draw overlays and HUD onto a BGR view, in place.

    `view` is already resized by `scale`; detections are in decoded-frame
    pixels. Empty `hud`/`esp_markers` give the plain view.
    """
    label_scale = _label_scale(scale) * (1.0 if detail else 1.3)
    taken: list = []            # label boxes so far, so none buries another
    # Rejected first, so a good detection on top stays readable.
    ordered = sorted(pc_dets,
                     key=lambda d: gate_reason(d) is None)
    for det in ordered:
        reason = gate_reason(det)
        if reason is not None and not show_rejected:
            continue
        colour = COL_REJECT if reason else COL_GOOD
        pts = [to_view(x, y, scale) for x, y in det.corners]
        cv2.polylines(view, [np.array(pts, np.int32)], True, colour,
                      1 if reason else 2, cv2.LINE_AA)
        if reason is None and detail:
            # Corner 0 shows the tag's orientation.
            cv2.circle(view, pts[0], max(3, int(2 * scale)), COL_CORNER0, -1,
                       cv2.LINE_AA)
        label = det_label(det, nav_ids, detail=detail)
        top = min(pts, key=lambda p: p[1])
        _text(view, label, min(p[0] for p in pts), top[1] - 5,
              COL_NAV if (reason is None and det.id in nav_ids) else colour,
              label_scale, taken=taken)

    text_h = cv2.getTextSize("0", FONT, label_scale, 1)[0][1]
    for m in esp_markers:
        x, y = to_view(m.cx, m.cy, scale)
        # Hollow cross (four ticks), so the tag underneath stays readable.
        r = max(5, int(4 * scale))
        gap = max(2, r // 2)
        for sx in (-1, 1):
            for sy in (-1, 1):
                cv2.line(view, (x + sx * gap, y + sy * gap),
                         (x + sx * r, y + sy * r), COL_ESP, 1, cv2.LINE_AA)
        # Below the marker: the laptop's label is above the tag.
        _text(view, esp_label(m), x + gap, y + r + 2 + text_h, COL_ESP,
              label_scale, taken=taken)

    hud_scale = _hud_scale(hud, view.shape[1])
    step = int(round(hud_scale * 30)) + 2
    for i, line in enumerate(_hud_wrap(hud, view.shape[1], hud_scale)):
        _text(view, line, 6, step + i * step, COL_HUD, hud_scale)


def hud_lines(*, frame_id: int, fps: float, esp_age_ms: int, latency_ms: float,
              drops: int, esp_drops: int, detect_ms: float,
              params: DetectorParams,
              n_good: int, n_rejected: int, esp_status: str,
              truncated: int = 0) -> list[str]:
    """The HUD text as plain strings. `truncated`: detections past
    MAX_DETECTIONS, counted but not returned."""
    extra = ([f"+{n_rejected} rejected"] if n_rejected else []) + \
            ([f"+{truncated} truncated"] if truncated else [])
    return [
        f"#{frame_id} {fps:4.1f} fps  esp {esp_age_ms} ms  "
        f"lat {latency_ms:.0f} ms  drops {drops}/{esp_drops}  "
        f"det {detect_ms:.1f} ms",
        f"{params.summary()}  gate h<={GATE_MAX_HAMMING} m>{GATE_MIN_MARGIN:g}  "
        f"{n_good} tag{'' if n_good == 1 else 's'}"
        + (f" ({', '.join(extra)})" if extra else ""),
        esp_status,
    ]


def det_record(det: Detection) -> dict:
    """One detection as JSON-friendly data for --save-dir."""
    return {
        "id":       det.id,
        "hamming":  det.hamming,
        "margin":   round(det.decision_margin, 3),
        "centre":   [round(v, 3) for v in det.centre],
        "corners":  [[round(v, 3) for v in c] for c in det.corners],
        "rejected": gate_reason(det),
        "pose_err": det.pose_err,
        "t":        None if det.t is None else [round(v, 5) for v in det.t],
    }


def visible_markers(markers: list[EspMarker], show_esp: bool,
                    show_rejected: bool) -> list[EspMarker]:
    """The drone's markers to draw; --save-dir logs them all."""
    if not show_esp:
        return []
    return [m for m in markers if show_rejected or m.gated]


def png_name(shown: int) -> str:
    """--save-dir name, numbered per run: drone frame ids restart on reboot."""
    return f"tag_stream_{shown:06d}.png"


def esp_record(m: EspMarker) -> dict:
    """One drone detection as JSON-friendly data for --save-dir."""
    return {
        "id":       m.id,
        "cx":       round(m.cx, 2),
        "cy":       round(m.cy, 2),
        "age_s":    round(m.age_s, 2),
        "gated":    m.gated,
        "margin":   round(m.margin, 2),
        "hamming":  m.hamming,
        # Significant figures: pose errors are ~4e-07, round(x, 6) gives 0.0.
        "pose_err": float(f"{m.pose_err:.3g}"),
    }


# --- CLI ---------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--esp-ip", required=True,
                        help="drone IP, 192.168.1.(200 + drone id)")
    parser.add_argument("--fps", type=int, default=0,
                        help=f"1-{CAMERA_STREAM_MAX_FPS} (default: firmware, 10)")
    parser.add_argument("--quality", type=int, default=0,
                        help=f"JPEG quality {CAMERA_STREAM_MIN_QUALITY}-"
                             f"{CAMERA_STREAM_MAX_QUALITY} (default: firmware, 60)")
    parser.add_argument("--scale", type=float, default=2.0,
                        help="window scale (default: 2)")
    parser.add_argument("--decimate", type=float,
                        default=DetectorParams.quad_decimate,
                        help="quad_decimate (default: firmware, 1.5)")
    parser.add_argument("--sigma", type=float, default=DetectorParams.quad_sigma,
                        help="quad_sigma (default: firmware, 1.0)")
    parser.add_argument("--detail", action="store_true",
                        help="tuning view: HUD, the gate's rejects, the "
                             "drone's own detections, full labels "
                             "(--save-dir records all of it either way)")
    parser.add_argument("--config", type=Path,
                        default=Path(__file__).resolve().parent / "setup.yaml",
                        help="setup.yaml used to label [nav] tags")
    parser.add_argument("--headless", action="store_true",
                        help="no window (for automated runs)")
    parser.add_argument("--frames", type=int, default=0,
                        help="exit after N shown frames (0 = run until quit)")
    parser.add_argument("--save-dir", type=Path, default=None,
                        help="write annotated PNGs + detections.jsonl here")
    return parser


def parse_args(argv=None) -> argparse.Namespace:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        args.esp_ip = socket.gethostbyname(args.esp_ip)  # datagrams carry numeric IPs
    except OSError:
        parser.error(f"cannot resolve --esp-ip {args.esp_ip}")
    try:
        build_camera_stream_command(True, args.fps, args.quality)
    except ValueError as exc:
        parser.error(str(exc))
    if args.scale <= 0:
        parser.error("--scale must be positive")
    if args.decimate < 1.0:
        parser.error("--decimate must be at least 1.0")
    if args.sigma < 0:
        parser.error("--sigma must not be negative")
    if args.frames < 0:
        parser.error("--frames must not be negative")
    if args.save_dir is not None:
        try:
            args.save_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            parser.error(f"cannot use --save-dir {args.save_dir}: {exc}")
    return args


def detector_params(args: argparse.Namespace) -> DetectorParams:
    """The firmware's parameters, with quad_decimate/quad_sigma overridable."""
    return DetectorParams(quad_decimate=args.decimate, quad_sigma=args.sigma)


# --- Main --------------------------------------------------------------------

def open_esp_socket() -> tuple:
    """Bind :5008 for the drone's own detections; returns (socket, warning).

    No SO_REUSEADDR: if tag_debug.py holds the port, fail cleanly rather than
    split the stream between two readers.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("", AT_DEBUG_PORT))
    except OSError as exc:
        sock.close()
        return None, (f"UDP :{AT_DEBUG_PORT} busy ({exc}) — the drone's own "
                      "detections are hidden; is tag_debug.py running?")
    sock.setblocking(False)
    return sock, None


@dataclass
class _Stats:
    shown: int = 0
    ages: float = 0.0
    latencies: float = 0.0
    detect_ms: float = 0.0
    start: float = field(default_factory=time.monotonic)


def main(argv=None) -> int:
    args = parse_args(argv)
    if cv2 is None:
        sys.exit("Missing dependencies: python3 -m pip install -r "
                 "laptop/requirements.txt")
    try:
        detector = Detector(detector_params(args))
    except ApriltagHostError as exc:
        sys.exit(str(exc))

    nav_ids = load_nav_tag_ids(args.config)

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    try:
        rx.bind(("0.0.0.0", CAMERA_STREAM_PORT))
    except OSError as exc:
        sys.exit(f"Cannot listen on UDP {CAMERA_STREAM_PORT}: {exc}")
    rx.setblocking(False)

    at_sock, warning = open_esp_socket()
    if warning and args.detail:         # the plain view does not draw them
        print(warning, file=sys.stderr)

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.esp_ip, COMMAND_PORT)
    keepalive = build_camera_stream_command(True, args.fps, args.quality)

    asm = FrameAssembler()
    esp = EspOverlay()
    # Placeholder size; real frames are sized from what they decode to.
    size = view_geometry((FRAME_H, FRAME_W), args.scale)[:2]
    window = "ESP32 camera + AprilTags"
    if not args.headless:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.imshow(window, np.zeros((size[1], size[0], 3), np.uint8))
    print(f"Requesting {args.esp_ip}; detector {detector.params.summary()}, "
          f"gate h<={GATE_MAX_HAMMING} m>{GATE_MIN_MARGIN:g}."
          + ("" if args.headless else
             "  q/Esc quits, s saves, d detail, h HUD, e drone layer, "
             "r rejects."))

    jsonl = None
    if args.save_dir is not None:
        jsonl = open(args.save_dir / "detections.jsonl", "w")

    socks = [rx] + ([at_sock] if at_sock else [])
    next_keepalive = time.monotonic()
    last_warning = float("-inf")
    warned_version = False
    stats = _Stats()
    shown_times: deque = deque(maxlen=20)    # rolling fps for the HUD
    # Plain by default; each layer also has its own key (d/h/e/r).
    show_esp = show_rejected = show_hud = detail = args.detail
    gray = view = None
    total_shown = 0
    try:
        while True:
            now = time.monotonic()
            if now >= next_keepalive:
                next_keepalive = now + KEEPALIVE_S
                try:
                    tx.sendto(keepalive, target)
                except OSError as exc:        # e.g. drone off: keep trying
                    if now - last_warning > 5:
                        print(f"keepalive failed: {exc}", file=sys.stderr)
                        last_warning = now

            select.select(socks, [], [], 0.005)
            if at_sock is not None:
                while True:
                    try:
                        data, (ip, _) = at_sock.recvfrom(2048)
                    except (BlockingIOError, InterruptedError):
                        break
                    if ip != args.esp_ip:
                        continue
                    pkt = parse_at_debug(data)
                    if pkt is not None:
                        esp.update(pkt, time.monotonic())

            frame, _, bad_version = drain_socket(rx, asm, args.esp_ip)
            if bad_version is not None and not warned_version:
                warned_version = True
                print(f"drone sends ECAM v{bad_version}, viewer expects "
                      f"v{CAMERA_VERSION}: reflash the drone", file=sys.stderr)

            if frame is not None:
                decoded = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8),
                                       cv2.IMREAD_GRAYSCALE)
                if decoded is None:
                    asm.dropped += 1
                else:
                    gray = np.ascontiguousarray(decoded)
                    t0 = time.perf_counter()
                    dets = detector.detect(gray)
                    detect_ms = (time.perf_counter() - t0) * 1000

                    now = time.monotonic()
                    good = [d for d in dets if gate_reason(d) is None]
                    # --save-dir logs all_markers, whatever is drawn.
                    all_markers = esp.markers(now) if at_sock else []
                    markers = visible_markers(all_markers, show_esp,
                                              show_rejected)
                    latency_ms = (now - frame.first_rx) * 1000
                    stats.shown += 1
                    stats.ages += frame.age_ms
                    stats.latencies += latency_ms
                    stats.detect_ms += detect_ms
                    # Rolling, so the HUD fps doesn't jump at each stats reset.
                    shown_times.append(now)
                    span = shown_times[-1] - shown_times[0]
                    hud = hud_lines(
                        frame_id=frame.frame_id,
                        fps=(len(shown_times) - 1) / span if span > 0 else 0.0,
                        esp_age_ms=frame.age_ms, latency_ms=latency_ms,
                        drops=asm.dropped, esp_drops=frame.esp_drops,
                        detect_ms=detect_ms, params=detector.params,
                        n_good=len(good), n_rejected=len(dets) - len(good),
                        truncated=detector.truncated,
                        esp_status=("esp layer off (e)" if not show_esp else
                                    f"esp :{AT_DEBUG_PORT} unavailable"
                                    if at_sock is None else esp.status(now)))
                    view_w, view_h, draw_scale = view_geometry(gray.shape,
                                                               args.scale)
                    view = cv2.resize(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR),
                                      (view_w, view_h),
                                      interpolation=cv2.INTER_NEAREST)
                    annotate(view, dets, markers, hud if show_hud else [],
                             scale=draw_scale, detail=detail, nav_ids=nav_ids,
                             show_rejected=show_rejected)
                    if not args.headless:
                        cv2.imshow(window, view)
                    total_shown += 1
                    if args.save_dir is not None:
                        png = png_name(total_shown)
                        cv2.imwrite(str(args.save_dir / png), view)
                        jsonl.write(json.dumps({
                            "png": png,
                            "frame_id": frame.frame_id,
                            "esp_age_ms": frame.age_ms,
                            "latency_ms": round(latency_ms, 1),
                            "detect_ms": round(detect_ms, 3),
                            "truncated": detector.truncated,
                            "dets": [det_record(d) for d in dets],
                            "esp": [esp_record(m) for m in all_markers],
                        }) + "\n")
                        jsonl.flush()

            now = time.monotonic()
            asm.expire(now)
            if now - stats.start >= STATS_S:
                n = max(stats.shown, 1)
                print(f"{stats.shown / (now - stats.start):4.1f} fps  "
                      f"esp age {stats.ages / n:.0f} ms  "
                      f"laptop {stats.latencies / n:.0f} ms  "
                      f"detect {stats.detect_ms / n:.1f} ms  "
                      f"drops {asm.dropped}")
                stats = _Stats(start=now)

            quit_now = False
            if not args.headless:
                # waitKey also presents the frame, so it runs before any exit.
                key = cv2.waitKey(1) & 0xFF
                quit_now = key in (ord("q"), 27)
                if key == ord("d"):     # every layer at once, as --detail
                    detail = not detail
                    show_esp = show_rejected = show_hud = detail
                if key == ord("h"):
                    show_hud = not show_hud
                if key == ord("e"):
                    show_esp = not show_esp
                if key == ord("r"):
                    show_rejected = not show_rejected
                if key == ord("s") and view is not None:
                    stamp = time.strftime("%Y%m%d_%H%M%S")
                    raw_path = f"tag_stream_{stamp}_raw.png"
                    ann_path = f"tag_stream_{stamp}.png"
                    ok = (cv2.imwrite(raw_path, gray)
                          and cv2.imwrite(ann_path, view))
                    print(f"saved {raw_path} + {ann_path}" if ok
                          else f"could not save {ann_path}")
            if quit_now or (args.frames and total_shown >= args.frames):
                break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            tx.sendto(build_camera_stream_command(False), target)
        except OSError:
            pass
        if jsonl is not None:
            jsonl.close()
        detector.close()
        rx.close()
        if at_sock is not None:
            at_sock.close()
        tx.close()
        if not args.headless:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
