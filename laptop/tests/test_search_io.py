"""Adapter checks with encoded MAVLink and a fake vehicle; no flight network."""
import math
import struct
import sys
from pathlib import Path
from types import SimpleNamespace
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from search_core import example_config,geometry,trajectory,TagRegistry,execute
from search_io import Link,Flight
from protocol import build_traj_packets,build_traj_start,CMD_LAND,CMD_START,NAV_ARRIVED,NAV_TRAJ


class Record:
    def __init__(self):self.rows=[]
    def event(self,name,**kw):self.rows.append((name,kw))


def test_real_mavlink_decode_position_attitude_arming_and_landed():
    link=Link(example_config(),Record());m=link.mav.MAVLink(None,srcSystem=1,srcComponent=1)
    messages=[m.local_position_ned_encode(1000,2,3,-1,.1,0,0),
              m.attitude_encode(1000,0,0,math.pi/2,0,0,0),
              m.heartbeat_encode(2,12,128,6<<16,4),m.extended_sys_state_encode(0,2)]
    for msg in messages:
        for decoded in link.parser.parse_buffer(msg.pack(m)):link.ingest(decoded,10.)
    s=link.state()
    assert s['position'][1]==[2,3,-1] and s['heartbeat'][1:] == (True,6)
    assert not s['landed'][1]
    p,rpy=link.pose_at(10.)
    assert p==(2,3,-1) and rpy[2]==pytest.approx(math.pi/2)
    link.ingest(m.extended_sys_state_encode(0,1),10.5)
    assert link.state()['landed'][1]


def test_estimator_reset_and_nonfinite_data_rejected():
    link=Link(example_config(),Record());m=link.encoder
    link.ingest(m.local_position_ned_encode(2000,0,0,0,0,0,0),10.)
    with pytest.raises(RuntimeError,match='timestamp reset'):
        link.ingest(m.local_position_ned_encode(100,0,0,0,0,0,0),11.)
    with pytest.raises(RuntimeError,match='Non-finite'):
        link.ingest(m.local_position_ned_encode(3000,float('nan'),0,0,0,0,0),12.)


def test_generated_trajectory_matches_stock_wire_format():
    points=trajectory((3,-2,-.5),.3,.3,.05);restored=[]
    for packet in build_traj_packets(7,points):
        kind,cmd,tid,total,off,n=struct.unpack('<BBBHHB',packet[:8])
        assert (kind,cmd,tid,total,off)==(2,8,7,len(points),len(restored))
        assert len(packet)<=1024
        restored.extend(struct.iter_unpack('<fff',packet[8:]))
        assert len(packet)==8+n*12
    assert restored[-1]==pytest.approx(points[-1]) and len(restored)==len(points)
    assert struct.unpack('<BBBH',build_traj_start(7,50))==(2,9,7,50)


class Vehicle:
    def __init__(self):
        self.commands=[]
        self.s=dict(position=(10.,[0.,0.,-1.],[0.,0.,0.]),attitude=(10.,[0.,0.,0.]),
                    heartbeat=(10.,True,6),custom=(10.,SimpleNamespace(nav_state=NAV_ARRIVED)),landed=(10.,False))
    def state(self):return self.s
    def command(self,cmd):self.commands.append(cmd)


def flight_fixture():
    c=example_config();link=Vehicle();rec=Record()
    f=Flight(c,geometry(c),link,rec,[0.,0.,0.],SimpleNamespace(error=None,last_usable_frame=10.,active=True))
    f.now=lambda:10.;f.start_time=0.
    return f,link,rec


def test_stale_telemetry_and_camera_never_allow_flight():
    f,link,_=flight_fixture();assert f.check()
    link.s['position']=(8.,[0.,0.,-1.],[0,0,0])
    with pytest.raises(RuntimeError,match='Stale'):f.check()
    link.s['position']=(10.,[0.,0.,-1.],[0,0,0]);f.vision.last_usable_frame=8.
    with pytest.raises(RuntimeError,match='camera'):f.check()


def test_reserve_includes_next_leg_and_return():
    f,_,_=flight_fixture()
    assert f.return_cost(via=[4.,0.])>f.return_cost()+25


@pytest.mark.parametrize('armed,ground',[(True,True),(False,False)])
def test_land_command_alone_does_not_confirm_landing(monkeypatch,armed,ground):
    f,link,rec=flight_fixture();link.s['heartbeat']=(10.,armed,6);link.s['landed']=(10.,ground)
    monkeypatch.setattr('search_io.time.sleep',lambda _:None)
    def wait(predicate,*args,**kw):
        assert not predicate(link.state())
        raise RuntimeError('unconfirmed')
    f.wait=wait
    with pytest.raises(RuntimeError,match='unconfirmed'):f.land()
    assert link.commands==[CMD_LAND]*3
    assert not any(name=='LANDING_CONFIRMED' for name,_ in rec.rows)


def test_land_requires_fresh_ground_state_inside_launch_area(monkeypatch):
    f,link,rec=flight_fixture();link.s['heartbeat']=(10.,False,6);link.s['landed']=(10.,True)
    monkeypatch.setattr('search_io.time.sleep',lambda _:None)
    def wait(predicate,*args,**kw):assert predicate(link.state());return link.state()
    f.wait=wait;f.land()
    assert rec.rows[-1][0]=='LANDING_CONFIRMED'
    link.s['position']=(10.,[.4,0,0],[0,0,0])
    with pytest.raises(RuntimeError,match='outside'):f.land()


class PacketVehicle(Vehicle):
    """Stock-packet playback stub; intentionally not a flight dynamics model."""
    def __init__(self,clock,vision,accept_start):
        super().__init__();self.clock=clock;self.vision=vision;self.accept_start=accept_start
        self.xyz=[0.,0.,0.];self.velocity=[0.,0.,0.];self.armed=False;self.ground=True
        self.upload=[];self.playing=None;self.nav=NAV_ARRIVED
    def state(self):
        now=self.clock[0]
        if self.playing:
            start,dt,origin=self.playing;k=(now-start)/dt;idx=min(int(k),len(self.upload)-1)
            if idx==len(self.upload)-1:
                self.xyz=[a+b for a,b in zip(origin,self.upload[-1])]
                self.velocity=[0,0,0];self.nav=NAV_ARRIVED;self.playing=None
            else:
                a,b=self.upload[idx:idx+2];f=k-idx
                self.xyz=[o+x+f*(y-x) for o,x,y in zip(origin,a,b)]
                self.velocity=[(y-x)/dt for x,y in zip(a,b)]
        self.vision.last_usable_frame=now
        return dict(position=(now,list(self.xyz),self.velocity),attitude=(now,[0,0,0]),
                    heartbeat=(now,self.armed,6),landed=(now,self.ground),custom=(now,SimpleNamespace(nav_state=self.nav)))
    def command(self,cmd):
        self.commands.append(cmd)
        if cmd==CMD_START:self.armed=True;self.ground=False;self.xyz[2]=-.5
        if cmd==CMD_LAND:self.armed=False;self.ground=True;self.xyz[2]=0.
    def raw_command(self,packet):
        if packet[1]==8:
            _,_,tid,total,off,n=struct.unpack('<BBBHHB',packet[:8])
            if off==0:self.upload=[None]*total
            self.upload[off:off+n]=list(struct.iter_unpack('<fff',packet[8:]))
        elif packet[1]==9 and self.accept_start:
            _,_,tid,dt=struct.unpack('<BBBH',packet)
            assert all(p is not None for p in self.upload)
            self.playing=(self.clock[0],dt/1000,list(self.xyz));self.nav=NAV_TRAJ


@pytest.mark.parametrize('accept_start',[True,False])
def test_full_flight_adapter_through_stock_packets(monkeypatch,accept_start):
    c=example_config();clock=[10.];rec=Record();vision=SimpleNamespace(error=None,last_usable_frame=10.,active=False)
    link=PacketVehicle(clock,vision,accept_start);flight=Flight(c,geometry(c),link,rec,[0,0,0],vision)
    flight.now=lambda:clock[0]
    def advance(seconds):clock[0]+=seconds
    monkeypatch.setattr('search_io.time.sleep',advance)
    result=execute(flight,c,[[0,0],[1,0],[1,.3],[0,0]],[0,0,0],TagRegistry(c))
    if accept_start:
        assert result.status=='partial' and result.returned and result.landed
        assert rec.rows[-1][0]=='MISSION_FINISHED'
        assert link.xyz==pytest.approx([0,0,0])
    else:
        assert result.status=='aborted' and not result.landed
        assert rec.rows[-1][0]=='ABORT_LAND_UNCONFIRMED'
    assert CMD_START in link.commands and CMD_LAND in link.commands
