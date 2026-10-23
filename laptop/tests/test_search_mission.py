import copy
import json
import math
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from search_core import (example_config,validate,geometry,trajectory,arena_to_local,local_to_arena,
                        tag_local,History,TagRegistry,execute)
from search_mission import main


def distance_segment(p,a,b):
    v=[b[i]-a[i] for i in range(2)];d=sum(x*x for x in v)
    t=0 if d==0 else max(0,min(1,sum((p[i]-a[i])*v[i] for i in range(2))/d))
    return math.dist(p,[a[i]+t*v[i] for i in range(2)])


def test_document_geometry_full_coverage_and_home():
    c=example_config();p=geometry(c)
    assert c['search_area']['width_y_m']==3.15
    assert p['route'][0]==p['route'][-1]==[0.,0.]
    # Independently sample all edges and interior against circular footprint.
    for i in range(31):
        for j in range(31):
            xy=(.9+3.6*i/30,-1.8+3.15*j/30)
            assert min(distance_segment(xy,a,b) for a,b in zip(p['route'],p['route'][1:]))<=p['effective_radius_m']+1e-9
    assert p['estimated_route_time_s']+c['takeoff_timeout_s']+c['return_reserve_s']<c['mission_timeout_s']


@pytest.mark.parametrize('delta',[(3.,0,0),(.01,.02,0),(-3,2,-.5),(0,0,.5)])
def test_trajectory_bounded_speed_and_acceleration(delta):
    dt=.05;pts=trajectory(delta,.3,.3,dt)
    assert pts[0]==(0.,0.,0.) and pts[-1]==delta
    vel=[[ (b[i]-a[i])/dt for i in range(3)] for a,b in zip(pts,pts[1:])]
    assert max(math.sqrt(sum(v*v for v in row)) for row in vel)<=.3+1e-8
    assert max(math.dist(a,b)/dt for a,b in zip(vel,vel[1:]))<=.3+1e-8
    assert math.sqrt(sum(v*v for v in vel[0]))<.02
    assert math.sqrt(sum(v*v for v in vel[-1]))<.02


def test_buffer_limit():
    with pytest.raises(ValueError,match='buffer'):trajectory((100,0,0),.3,.3,.05)


@pytest.mark.parametrize('heading',[0,90,180,-45,37])
def test_frame_roundtrip_and_nonzero_home(heading):
    home=[12,-7,0];point=[2,1]
    local=arena_to_local(point,home,heading)
    assert local_to_arena(local,home,heading)==pytest.approx(point)
    assert arena_to_local([0,0],home,heading)==home[:2]


def test_downward_camera_and_yaw():
    cam=example_config()['camera'];cam['position_body_m']=[.02,0,0]
    assert tag_local([0,0,1],[2,3,-1],[0,0,0],cam)==pytest.approx([2.02,3,0])
    assert tag_local([0,0,1],[2,3,-1],[0,0,math.pi/2],cam)==pytest.approx([2,3.02,0])
    # Camera pixel right maps to body right with the documented image orientation.
    assert tag_local([.1,0,1],[0,0,-1],[0,0,0],cam)==pytest.approx([.02,.1,0])


def test_clock_history_interpolation_and_wrap():
    h=History();h.add(1,[1,2,3]);h.add(1.1,[2,3,4])
    assert h.at(1.05)==pytest.approx([1.5,2.5,3.5]);assert h.at(2) is None
    h=History(angles=True);h.add(1,[math.radians(179)]);h.add(1.1,[math.radians(-179)])
    assert abs(abs(h.at(1.05)[0])-math.pi)<1e-8


def test_unique_frames_outliers_and_three_tags():
    c=example_config();r=TagRegistry(c)
    for tid in range(3):
        for i in range(5):assert r.add(tid,i,i*.2,[tid,1,0])
    assert r.complete()
    assert not r.add(0,4,2,[0,1,0])
    assert not r.add(0,99,3,[4,1,0])
    assert r.results()['0']['observations']==5


@pytest.mark.parametrize('change',[{'drone_id':None},{'arena_heading_deg':None},{'flight_y_min_m':None},{'expected_tags':True},{'speed_m_s':5},{'max_tracking_error_m':.3}])
def test_invalid_config(change):
    c=example_config();c.update(change)
    with pytest.raises((ValueError,TypeError)):validate(c)


def test_live_rejects_example_and_missing_tag_size():
    c=example_config()
    with pytest.raises(ValueError):validate(c,live=True)
    c.update(example_only=False,geometry_verified=True);c['camera']['tag_size_m']=None
    with pytest.raises(ValueError):validate(c,live=True)


class FakeIO:
    def __init__(self,fail=None,elapsed=0):self.calls=[];self.start_time=0;self.elapsed=elapsed;self.fail=fail
    def now(self):return self.elapsed
    def event(self,name,**kw):self.calls.append(name)
    def takeoff(self):
        self.calls.append('takeoff')
        if self.fail=='takeoff':raise RuntimeError('takeoff failed')
    def move(self,*args,**kw):
        self.calls.append('return' if kw.get('returning') else 'move')
        if self.fail=='move':raise RuntimeError('missed waypoint')
    def return_cost(self,via=None):return 30
    def land(self):
        self.calls.append('land')
        if self.fail=='land':raise RuntimeError('landing unconfirmed')
    def abort_land(self):self.calls.append('abort_land')


def test_unconfirmed_tags_produce_partial_result_and_return():
    c=example_config();io=FakeIO();r=execute(io,c,[[0,0],[1,0],[0,0]],[4,5,0],TagRegistry(c))
    assert r.status=='partial' and r.returned and r.landed
    assert io.calls.index('return')<io.calls.index('land')


@pytest.mark.parametrize('failure',['takeoff','move','land'])
def test_failed_state_never_reports_completion(failure):
    c=example_config();io=FakeIO(failure);r=execute(io,c,[[0,0],[1,0],[0,0]],[4,5,0],TagRegistry(c))
    assert r.status=='aborted' and not r.landed
    assert 'abort_land' in io.calls and 'MISSION_FINISHED' not in io.calls


def test_time_reserve_returns_before_new_search_leg():
    c=example_config();io=FakeIO(elapsed=500)
    execute(io,c,[[0,0],[1,0],[0,0]],[0,0,0],TagRegistry(c))
    assert 'move' not in io.calls and 'return' in io.calls


def test_offline_plan_never_opens_a_socket(tmp_path,monkeypatch):
    import socket
    monkeypatch.setattr(socket,'socket',lambda *a,**kw:pytest.fail('Offline planning used network'))
    assert main(['--plan','--example','--out',str(tmp_path/'plan')])==0


def test_simulated_end_to_end(tmp_path):
    out=tmp_path/'sim'
    assert main(['--simulate','--example','--out',str(out)])==0
    result=json.loads((out/'result.json').read_text())
    assert result['status']=='complete' and result['returned'] and result['landed']
    assert len(json.loads((out/'tags.json').read_text()))==3


def test_disarmed_inspection_has_no_flight_adapter(tmp_path,monkeypatch):
    import time
    import search_io
    from types import SimpleNamespace
    c=example_config();c['example_only']=False
    config=tmp_path/'config.json';config.write_text(json.dumps(c))
    class Readback:
        def __init__(self,*args):pass
        def start(self):pass
        def close(self):pass
        def state(self):
            t=time.monotonic()
            return dict(position=(t,[0,0,0],[0,0,0]),attitude=(t,[0,0,0]),heartbeat=(t,False,0),landed=(t,True))
        # Deliberately no flight-command or raw-command methods.
    monkeypatch.setattr(search_io,'Link',Readback)
    monkeypatch.setattr(search_io,'Vision',lambda *a:SimpleNamespace(start=lambda:None,close=lambda:None,error=None,active=False))
    monkeypatch.setattr(search_io,'Flight',lambda *a:pytest.fail('Inspection constructed a flight controller'))
    out=tmp_path/'inspect'
    assert main(['--inspect-tags','--config',str(config),'--seconds','.01','--out',str(out)])==0
    events=[json.loads(line)['event'] for line in (out/'events.jsonl').read_text().splitlines()]
    assert events==['INSPECTION_STARTED','INSPECTION_FINISHED']
