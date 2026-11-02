"""Read-only PX4 telemetry from QGC forwarding on localhost. Sends no MAVLink.

QGC: enable MAVLink forwarding to 127.0.0.1:14551. Keep only the intended
vehicle connected: course aircraft all use MAVLink system 1/component 1.
"""
from dataclasses import dataclass, replace
import math
import socket
import threading
import time

@dataclass(frozen=True)
class FlightStatus:
    position_time: float = 0.
    heartbeat_time: float = 0.
    sample_ms: int | None = None
    x: float = 0.
    y: float = 0.
    z: float = 0.
    vx: float = 0.
    vy: float = 0.
    vz: float = 0.
    armed: bool = False
    offboard: bool = False

class AltitudeMonitor:
    def __init__(self, port=14551, system_id=1):
        try:
            from pymavlink.dialects.v20 import common
        except ImportError as exc:
            raise RuntimeError('Install telemetry reader: python -m pip install pymavlink') from exc
        self._parser = common.MAVLink(None)
        self._parser.robust_parsing = True
        self._system_id = system_id
        self._lock = threading.Lock()
        self._state = FlightStatus()
        self._error = None
        self._stop = threading.Event()
        self._thread = None
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._sock.bind(('127.0.0.1',port))
            self._sock.settimeout(.2)
        except BaseException:
            self._sock.close()
            raise

    def start(self):
        self._thread = threading.Thread(target=self._receive, name='px4-altitude-reader', daemon=True)
        self._thread.start()

    def feed(self, data, now=None):
        now = time.monotonic() if now is None else now
        for msg in self._parser.parse_buffer(data) or []:
            if msg.get_srcSystem()!=self._system_id or msg.get_srcComponent()!=1:
                continue
            kind=msg.get_type()
            with self._lock:
                if kind=='HEARTBEAT' and msg.autopilot==12:  # MAV_AUTOPILOT_PX4
                    self._state=replace(self._state, heartbeat_time=now,
                        armed=bool(msg.base_mode & 128),
                        offboard=bool(msg.base_mode & 1) and ((msg.custom_mode>>16)&255)==6)
                elif kind=='LOCAL_POSITION_NED':
                    values=(msg.x,msg.y,msg.z,msg.vx,msg.vy,msg.vz)
                    if not all(math.isfinite(v) for v in values):
                        continue
                    previous=self._state.sample_ms
                    if previous is not None:
                        delta=(msg.time_boot_ms-previous)&0xffffffff
                        if delta==0: continue  # duplicates do not refresh position age
                        if delta>=0x80000000: continue  # reordered samples do not refresh age
                    self._state=replace(self._state, position_time=now,
                        sample_ms=msg.time_boot_ms,x=msg.x,y=msg.y,z=msg.z,
                        vx=msg.vx,vy=msg.vy,vz=msg.vz)

    def snapshot(self):
        with self._lock:
            if self._error: raise RuntimeError(self._error)
            return self._state

    def _receive(self):
        try:
            while not self._stop.is_set():
                try:data,_=self._sock.recvfrom(65535)
                except socket.timeout:continue
                self.feed(data)
        except Exception as exc:
            if not self._stop.is_set():
                with self._lock:self._error=f'MAVLink reader stopped: {exc}'

    def close(self):
        self._stop.set()
        if self._thread:self._thread.join(1.)
        self._sock.close()
