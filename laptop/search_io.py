"""Laptop-only adapters for stock ESP32 packets and its existing MAVLink bridge."""
from __future__ import annotations
import json
import math
import socket
import threading
import time
from collections import deque
from datetime import datetime,timezone
from pathlib import Path

from protocol import (CMD_START,CMD_LAND,CommandPacket,build_command,
    build_traj_packets,build_traj_start,parse_telemetry,parse_camera_chunk,
    build_camera_stream_command,NAV_TRAJ,NAV_ARRIVED)
from camera_stream import FrameAssembler
from search_core import History,inside,local_to_arena,trajectory,leg_time,tag_local


class Recorder:
    def __init__(self,out):
        self.out=Path(out);self.out.mkdir(parents=True,exist_ok=False)
        self.file=(self.out/'events.jsonl').open('w',buffering=1)
        self.lock=threading.Lock();self.start=time.monotonic()
    def event(self,event,**fields):
        row=dict(event=event,utc=datetime.now(timezone.utc).isoformat(),host_monotonic_s=time.monotonic(),**fields)
        with self.lock:self.file.write(json.dumps(row,allow_nan=False)+'\n')
        if event not in ('PX4_POSITION','PX4_ATTITUDE','CUSTOM_TELEMETRY','FRAME','DETECTION_REJECTED'):
            print(json.dumps(row),flush=True)
    def save(self,name,data):
        (self.out/name).write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
    def close(self):
        with self.lock:self.file.close()


class Link:
    """One aircraft, explicit IP, exclusive local UDP ports. No peer broadcasting."""
    def __init__(self,c,rec):
        from pymavlink.dialects.v20 import common as mav
        self.c=c;self.rec=rec;self.mav=mav
        self.ip=f"192.168.1.{200+c['drone_id']}";self.sockets=[]
        self.parser=mav.MAVLink(None);self.parser.robust_parsing=True
        self.encoder=mav.MAVLink(None,srcSystem=255,srcComponent=191)
        self.running=False;self.error=None;self.lock=threading.RLock()
        self.positions=History();self.attitudes=History(angles=True)
        self.position=None;self.attitude=None;self.heartbeat=None;self.landed=None;self.custom=None
        self.boot_offsets=deque(maxlen=400);self.boot_last={};self.clock_offset=None
    def bind(self,port):
        s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        try:s.bind(('0.0.0.0',port));s.settimeout(.1)
        except Exception:s.close();raise
        self.sockets.append(s);return s
    def start(self):
        try:
            self.raw=self.bind(self.c['ports']['mavlink_in'])
            self.telem=self.bind(self.c['ports']['telemetry'])
            self.tx=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.sockets.append(self.tx)
        except Exception:
            for s in self.sockets:s.close()
            raise
        self.running=True
        self.threads=[threading.Thread(target=self.receive,args=(self.raw,True),daemon=True),threading.Thread(target=self.receive,args=(self.telem,False),daemon=True)]
        for t in self.threads:t.start()
        # Readback subscriptions only: no mode/arming/parameter/setpoint commands.
        for msg_id,hz in ((32,20),(30,20),(245,2)):
            packet=self.encoder.command_long_encode(1,1,511,0,msg_id,1e6/hz,0,0,0,0,0)
            self.tx.sendto(packet.pack(self.encoder),(self.ip,self.c['ports']['mavlink_bridge']))
    def receive(self,sock,raw):
        try:
            while self.running:
                try:data,addr=sock.recvfrom(65535)
                except socket.timeout:continue
                if addr[0]!=self.ip:continue
                now=time.monotonic()
                if raw:
                    for msg in self.parser.parse_buffer(data) or []:
                        if msg.get_srcSystem()==1 and msg.get_srcComponent()==1:self.ingest(msg,now)
                else:
                    p=parse_telemetry(data)
                    if p and p.drone_id==self.c['drone_id']:
                        with self.lock:self.custom=(now,p)
                        self.rec.event('CUSTOM_TELEMETRY',nav_state=p.nav_state,map_x=p.ned_x,map_y=p.ned_y)
        except Exception as exc:
            if self.running:self.error=exc
    def ingest(self,msg,now):
        kind=msg.get_type()
        with self.lock:
            capture=now
            if kind in ('LOCAL_POSITION_NED','ATTITUDE'):
                boot=msg.time_boot_ms/1000.
                last=self.boot_last.get(kind)
                if last is not None and boot<last-.2:
                    raise RuntimeError('PX4 timestamp reset; mission coordinates no longer trustworthy')
                self.boot_last[kind]=boot;self.boot_offsets.append(now-boot)
                self.clock_offset=min(self.boot_offsets);capture=boot+self.clock_offset
            if kind=='LOCAL_POSITION_NED':
                xyz=[msg.x,msg.y,msg.z];vel=[msg.vx,msg.vy,msg.vz]
                if not all(math.isfinite(v) for v in xyz+vel):raise RuntimeError('Non-finite PX4 position')
                if self.position:
                    old_t,old,old_v=self.position
                    if math.dist(xyz,old)>max(.6,2*math.sqrt(sum(v*v for v in vel))*(now-old_t)+.2):
                        raise RuntimeError('Discontinuous PX4 position; stop and re-establish the frame')
                self.position=(now,xyz,vel);self.positions.add(capture,xyz)
                self.rec.event('PX4_POSITION',px4_boot_ms=msg.time_boot_ms,estimated_capture_host_s=capture,position=xyz,velocity=vel)
            elif kind=='ATTITUDE':
                angles=[msg.roll,msg.pitch,msg.yaw]
                if not all(math.isfinite(v) for v in angles):raise RuntimeError('Non-finite attitude')
                self.attitude=(now,angles);self.attitudes.add(capture,angles)
                self.rec.event('PX4_ATTITUDE',px4_boot_ms=msg.time_boot_ms,rpy=angles)
            elif kind=='HEARTBEAT':
                self.heartbeat=(now,bool(msg.base_mode&128),(msg.custom_mode>>16)&255)
            elif kind=='EXTENDED_SYS_STATE':self.landed=(now,msg.landed_state==1)
    def state(self):
        with self.lock:
            if self.error:raise RuntimeError(f'Telemetry reader failed: {self.error}')
            return dict(position=self.position,attitude=self.attitude,heartbeat=self.heartbeat,landed=self.landed,custom=self.custom)
    def pose_at(self,t):
        with self.lock:return self.positions.at(t),self.attitudes.at(t)
    def raw_command(self,data):self.tx.sendto(data,(self.ip,self.c['ports']['commands']))
    def command(self,kind):self.raw_command(build_command(CommandPacket(kind)))
    def close(self):
        self.running=False
        for t in getattr(self,'threads',[]):t.join(timeout=1.)
        for s in self.sockets:s.close()


class Vision:
    def __init__(self,c,link,rec,registry,home):
        import cv2
        import numpy as np
        from apriltag_host import Detector,Intrinsics
        self.cv2=cv2;self.np=np;self.c=c;self.link=link;self.rec=rec;self.registry=registry;self.home=home
        cam=c['camera']
        self.detector=Detector(intrinsics=Intrinsics(cam['fx'],cam['fy'],cam['cx'],cam['cy']),tag_size_m=cam['tag_size_m'])
        self.assembler=FrameAssembler();self.running=False;self.error=None;self.last_frame=0.;self.last_usable_frame=0.;self.active=False
        self.lock=threading.Lock()
    def start(self):
        self.rx=self.link.bind(self.c['ports']['camera'])
        self.running=True;self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def run(self):
        try:
            keepalive=0.
            while self.running:
                now=time.monotonic()
                if now>=keepalive:
                    self.link.raw_command(build_camera_stream_command(True,10,60));keepalive=now+1
                try:data,addr=self.rx.recvfrom(65535)
                except socket.timeout:continue
                if addr[0]!=self.link.ip:continue
                chunk=parse_camera_chunk(data)
                if not chunk or chunk.drone_id!=self.c['drone_id']:continue
                now=time.monotonic();frame=self.assembler.add(chunk,now)
                self.assembler.expire(now)
                if frame is None:continue
                self.last_frame=now
                gray=self.cv2.imdecode(self.np.frombuffer(frame.jpeg,self.np.uint8),self.cv2.IMREAD_GRAYSCALE)
                if gray is None or gray.shape!=(240,320):raise RuntimeError('Camera resolution differs from calibration')
                if frame.age_ms/1000>self.c['camera']['max_frame_age_s']:continue
                # age_ms ends at ESP transmit, not laptop receipt. Network correction
                # is configurable; this is explicitly an approximate clock alignment.
                capture=now-frame.age_ms/1000-self.c['camera']['network_delay_estimate_s']
                position,rpy=self.link.pose_at(capture)
                self.rec.event('FRAME',frame_id=frame.frame_id,esp_age_ms=frame.age_ms,estimated_capture_host_s=capture)
                if position is None or rpy is None:continue
                tilt=math.acos(max(-1.,min(1.,math.cos(rpy[0])*math.cos(rpy[1]))))
                if tilt>math.radians(self.c['camera']['max_tilt_deg']):continue
                self.last_usable_frame=now
                if not self.active:continue
                for d in self.detector.detect(gray):
                    if d.hamming>1 or d.decision_margin<=55 or not d.pose_valid or d.t is None or d.pose_err is None or not 0<=d.pose_err<.5:continue
                    xyz=tag_local(d.t,position,rpy,self.c['camera'])
                    arena=local_to_arena(xyz,self.home,self.c['arena_heading_deg'])
                    area=self.c['search_area'];pad=self.c['tag_outlier_radius_m']
                    in_area=inside(arena,[area['x_min_m']-pad,area['y_min_m']-pad,area['x_min_m']+area['length_x_m']+pad,area['y_min_m']+area['width_y_m']+pad])
                    if abs(xyz[2]-self.home[2])>self.c['tag_floor_tolerance_m'] or not in_area:continue
                    fid=(self.assembler.nonce,frame.frame_id)
                    with self.lock:
                        accepted=self.registry.add(d.id,fid,capture,xyz)
                        results=self.registry.results()
                    self.rec.event('TAG_DETECTION',id=d.id,frame_id=frame.frame_id,position_local_ned_m=xyz,
                        camera_translation_m=d.t,camera_rotation=d.R,vehicle_position=position,vehicle_rpy=rpy,
                        estimated_capture_host_s=capture,px4_boot_estimate_s=capture-self.link.clock_offset,
                        timestamp_method='receipt-minus-ESP-age-minus-configured-network-delay; approximate',
                        decision_margin=d.decision_margin,pose_error=d.pose_err,accepted=accepted)
                    if accepted:
                        self.rec.save('tags.json',results)
        except Exception as exc:self.error=exc
    def close(self):
        self.running=False
        if hasattr(self,'thread'):self.thread.join(timeout=3.)
        try:self.link.raw_command(build_camera_stream_command(False))
        finally:
            # Never free the native detector while its worker is still using it.
            if not hasattr(self,'thread') or not self.thread.is_alive():self.detector.close()


class Flight:
    def __init__(self,c,p,link,rec,home,vision):
        self.c=c;self.plan=p;self.link=link;self.rec=rec;self.home=home;self.vision=vision
        self.start_time=time.monotonic();self.tid=0
        self.last_goal=list(home)
    @staticmethod
    def now():return time.monotonic()
    def event(self,name,**kw):self.rec.event(name,**kw)
    def check(self,flying=True,landing=False):
        s=self.link.state();now=self.now()
        for key,limit in (('position',self.c['position_stale_s']),('attitude',self.c['position_stale_s']),('heartbeat',2.5),('custom',self.c['custom_stale_s'])):
            if s[key] is None or now-s[key][0]>limit:raise RuntimeError(f'Stale or absent {key}')
        if flying and not landing and (not s['heartbeat'][1] or s['heartbeat'][2]!=6):
            raise RuntimeError('Vehicle left armed Offboard control; mission stops')
        p=local_to_arena(s['position'][1],self.home,self.c['arena_heading_deg'])
        margin=self.c['vehicle_radius_m']+self.c['boundary_buffer_m']
        b=self.plan['flight_bounds'];bounds=[b[0]+margin,b[1]+margin,b[2]-margin,b[3]-margin]
        if not inside(p,bounds):raise RuntimeError('Vehicle reached the flight boundary margin')
        if self.home[2]-s['position'][1][2]>2.1:raise RuntimeError('Operating-height limit exceeded')
        if flying and not landing:
            if self.vision.error:raise RuntimeError(f'Detection worker failed: {self.vision.error}')
            if now-self.vision.last_usable_frame>1.:raise RuntimeError('Fresh camera frames aligned with vehicle pose are unavailable')
            r,pitch,_=s['attitude'][1]
            if math.acos(max(-1.,min(1.,math.cos(r)*math.cos(pitch))))>math.radians(self.c['camera']['max_tilt_deg']):
                raise RuntimeError('Tilt exceeds planned coverage allowance')
            if now-self.start_time>self.c['mission_timeout_s']:raise RuntimeError('Mission deadline reached')
        return s
    def wait(self,predicate,timeout,flying=True,landing=False,dwell=0.):
        deadline=self.now()+timeout;since=None
        while self.now()<deadline:
            s=self.check(flying,landing)
            if predicate(s):
                since=self.now() if since is None else since
                if self.now()-since>=dwell:return s
            else:since=None
            time.sleep(.05)
        raise RuntimeError('Timed out waiting for verified vehicle state')
    def takeoff(self):
        self.start_time=self.now()
        for _ in range(3):self.link.command(CMD_START);time.sleep(.1)
        self.wait(lambda s:s['heartbeat'][1] and s['heartbeat'][2]==6 and
                  abs(s['position'][1][2]-self.c['firmware_takeoff_local_z_m'])<.15 and
                  abs(s['position'][2][2])<self.c['settled_speed_m_s'],self.c['takeoff_timeout_s'],flying=False,dwell=1.5)
        # Stock mission_task has an additional one-second transition to FLYING.
        time.sleep(1.2)
        self.vision.active=True
        self.move(self.home[:2],self.home[2]-self.c['altitude_m'])
        self.event('TAKEOFF_CONFIRMED')
    def return_cost(self,via=None):
        s=self.check();points=[s['position'][1][:2]]+([via] if via is not None else [])+[self.home[:2]]
        return sum(leg_time(math.dist(a,b),self.c['speed_m_s'],self.c['acceleration_m_s2'])+self.c['settle_s']+2.
                   for a,b in zip(points,points[1:]))+self.c['landing_timeout_s']+5.
    def move(self,xy,z,returning=False):
        s=self.check();origin=list(s['position'][1]);goal=[*xy,z]
        if not inside(local_to_arena(goal,self.home,self.c['arena_heading_deg']),self.plan['allowed_center_bounds']):
            raise RuntimeError('Goal violates permitted center boundary')
        delta=[a-b for a,b in zip(goal,origin)]
        if math.dist(goal,origin)>.02:
            pts=trajectory(delta,self.c['speed_m_s'],self.c['acceleration_m_s2'],self.c['sample_period_s'])
            self.tid=self.tid%255+1
            packets=build_traj_packets(self.tid,pts)
            if s['custom'][1].nav_state==NAV_TRAJ:raise RuntimeError('Previous trajectory still active')
            for _ in range(2):
                for packet in packets:self.link.raw_command(packet);time.sleep(.03)
            self.check()
            # Correct relative-to-hover reference after upload: origin must not drift.
            before=self.link.state()['position'][1]
            if math.dist(before,origin)>self.c['tracking_error_m']:
                raise RuntimeError('Hover moved while uploading; replan from the new position')
            start_packet=build_traj_start(self.tid,round(1000*self.c['sample_period_s']))
            sent=self.now();self.link.raw_command(start_packet)
            def began(s):return s['custom'][0]>sent and s['custom'][1].nav_state==NAV_TRAJ
            self.wait(began,3.)
            duration=(len(pts)-1)*self.c['sample_period_s']
            self.event('TRAJECTORY_STARTED',id=self.tid,origin=origin,goal=goal,duration_s=duration,reference_speed_m_s=self.c['speed_m_s'])
            deadline=self.now()+duration+8.
            while self.now()<deadline:
                state=self.check()
                # Measure cross-track error, allowing along-track controller lag.
                ideal=self.last_goal;v=[g-o for g,o in zip(goal,ideal)];den=sum(a*a for a in v)
                t=0. if den<1e-12 else max(0.,min(1.,sum((p-o)*a for p,o,a in zip(state['position'][1],ideal,v))/den))
                closest=[o+t*a for o,a in zip(ideal,v)]
                if math.dist(state['position'][1],closest)>self.c['max_tracking_error_m']:
                    raise RuntimeError('Flight deviated from the planned segment')
                if state['custom'][1].nav_state==NAV_ARRIVED:break
                if state['custom'][1].nav_state!=NAV_TRAJ:raise RuntimeError('Trajectory interrupted')
                time.sleep(.05)
            else:raise RuntimeError('Trajectory playback timeout')
        self.wait(lambda s:math.dist(s['position'][1],goal)<=self.c['arrival_radius_m'] and
                  math.sqrt(sum(v*v for v in s['position'][2]))<=self.c['settled_speed_m_s'],10.,dwell=self.c['settle_s'])
        self.event('GOAL_REACHED',goal=goal,returning=returning)
        self.last_goal=goal
    def land(self):
        self.vision.active=False
        for _ in range(3):self.link.command(CMD_LAND);time.sleep(.1)
        self.wait(lambda s:not s['heartbeat'][1] and s['landed'] is not None and self.now()-s['landed'][0]<2.5 and s['landed'][1],
                  self.c['landing_timeout_s'],flying=False,landing=True,dwell=.5)
        state=self.link.state()
        if math.dist(state['position'][1][:2],self.home[:2])>.45-self.c['vehicle_radius_m']:
            raise RuntimeError('Landed outside conservative launch-area acceptance circle')
        self.event('LANDING_CONFIRMED',position_local_ned_m=state['position'][1])
    def abort_land(self):
        self.vision.active=False
        self.event('ABORT_LAND_COMMAND')
        for _ in range(3):self.link.command(CMD_LAND);time.sleep(.1)
        # Never report confirmed landing merely because a command was sent.
        self.event('ABORT_LAND_UNCONFIRMED',reason='Inspect aircraft/RC; no automatic recovery or restart')
