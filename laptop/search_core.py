"""Pure mission geometry, trajectories and estimation; no sockets or flight I/O."""
from __future__ import annotations
import bisect
import copy
import json
import math
import statistics
import threading
from dataclasses import dataclass
from pathlib import Path

from search_geometry import plan, inside, number


def load_config(path):
    return json.loads(Path(path).read_text())


def example_config():
    c = load_config(Path(__file__).with_name('search_configs')/'mission.json')
    c.update(example_only=True, geometry_verified=False, drone_id=2,
             arena_heading_deg=0., flight_y_min_m=-2.4)
    c['camera'].update(tag_size_m=.12, position_body_m=[.02,0.,0.])
    return c


def validate(c, live=False):
    if live and (c['example_only'] or not c['geometry_verified']):
        raise ValueError('Live flight needs a measured configuration: example_only=false, geometry_verified=true. See SEARCH_MISSION.md.')
    for k in ('drone_id', 'expected_tags', 'min_tag_observations'):
        if isinstance(c[k],bool) or not isinstance(c[k],int) or c[k]<1:
            raise ValueError(f'{k} must be a positive integer')
    if c['drone_id']>54:
        raise ValueError('Drone ID must fit the repository IP scheme: 1..54')
    for k in ('arena_heading_deg','flight_y_min_m','flight_x_min_m','firmware_takeoff_local_z_m'):
        number(c[k],k,-math.inf)
    for k in ('flight_width_y_m','flight_length_x_m','altitude_m','speed_m_s','acceleration_m_s2','sample_period_s',
              'arrival_radius_m','position_stale_s','custom_stale_s','settle_s','settled_speed_m_s','max_tracking_error_m',
              'mission_timeout_s','return_reserve_s','takeoff_timeout_s','landing_timeout_s','min_tag_span_s',
              'tag_outlier_radius_m','tag_max_spread_m','tag_floor_tolerance_m'):
        number(c[k],k,strictly_positive=True)
    if c['altitude_m']>2.0 or c['speed_m_s']>.3 or c['acceleration_m_s2']>.5:
        raise ValueError('This initial implementation supports height <=2 m, reference speed <=0.3 m/s and acceleration <=0.5 m/s²')
    if not .05<=c['sample_period_s']<=.2 or abs(c['sample_period_s']*1000-round(c['sample_period_s']*1000))>1e-6:
        raise ValueError('sample_period_s must be 0.05..0.2 in integer milliseconds')
    if c['return_reserve_s']>=c['mission_timeout_s']:
        raise ValueError('Return reserve must be less than mission timeout')
    if c['max_tracking_error_m']>c['tracking_error_m']+c['arrival_radius_m']+1e-9:
        raise ValueError('Runtime tracking threshold must fit the geometric tracking + arrival allowance')
    if c['firmware_takeoff_local_z_m'] != -.5:
        raise ValueError('Pinned stock firmware takeoff target is local z=-0.5 m')
    if c['expected_tags']>22:
        raise ValueError('The stock tag16h5 family has only 22 IDs')
    ids=c.get('allowed_tag_ids')
    if ids is not None and (not isinstance(ids,list) or len(set(ids))!=len(ids) or len(ids)<c['expected_tags'] or any(type(i)!=int or not 0<=i<22 for i in ids)):
        raise ValueError('allowed_tag_ids must be unique IDs in 0..21 and include at least expected_tags IDs')
    cam=c['camera']
    for k in ('fx','fy','tag_size_m','usable_fraction','max_frame_age_s','max_tilt_deg'):
        number(cam[k],'camera.'+k,strictly_positive=True)
    for k in ('cx','cy','network_delay_estimate_s'):
        number(cam[k],'camera.'+k)
    for k in ('pitch_down_deg','image_rotation_deg'):
        number(cam[k],'camera.'+k,-math.inf)
    if cam['pitch_down_deg']!=90 or cam['usable_fraction']>1 or cam['max_tilt_deg']>15:
        raise ValueError('Planner requires downward camera, usable_fraction<=1 and max_tilt_deg<=15')
    if (cam['width_px'],cam['height_px'])!=(320,240) or not 0<cam['cx']<320 or not 0<cam['cy']<240:
        raise ValueError('Intrinsics must match stock 320x240 stream')
    if len(cam['position_body_m'])!=3:
        raise ValueError('Camera offset must have three body-frame components')
    for value in cam['position_body_m']: number(value,'camera.position_body_m',-math.inf)
    for value in c['ports'].values():
        if type(value)!=int or not 1<=value<=65535: raise ValueError('Invalid UDP port')
    if len(set(c['ports'].values()))!=len(c['ports']): raise ValueError('UDP ports must be distinct')


def geometry(c):
    validate(c)
    cam=c['camera']
    # Centered optical ray's nearest image edge, not a symmetric-FOV assumption.
    angle=math.atan(min(cam['cx']/cam['fx'],(320-cam['cx'])/cam['fx'],cam['cy']/cam['fy'],(240-cam['cy'])/cam['fy']))
    angle-=math.radians(cam['max_tilt_deg'])
    radius=cam['measured_reliable_radius_m']
    if radius is None:
        # Whole tag must fit, including at the lowest permitted camera height.
        low_height=c['altitude_m']-c['max_tracking_error_m']-math.sqrt(sum(v*v for v in cam['position_body_m']))
        radius=low_height*math.tan(angle)*cam['usable_fraction']-cam['tag_size_m']/math.sqrt(2)
    else:
        number(radius,'camera.measured_reliable_radius_m',strictly_positive=True)
    radius-=math.hypot(*cam['position_body_m'][:2])
    raw=dict(search_area=c['search_area'],flight_area=dict(x_min_m=c['flight_x_min_m'],y_min_m=c['flight_y_min_m'],
             length_x_m=c['flight_length_x_m'],width_y_m=c['flight_width_y_m']),expected_tags=c['expected_tags'],
             altitude_m=c['altitude_m'],max_altitude_m=2.,camera=dict(horizontal_fov_deg=90.,vertical_fov_deg=70.,
             usable_fraction=1.,measured_reliable_radius_m=radius),overlap_fraction=c['overlap_fraction'],
             tracking_error_m=c['tracking_error_m'],arrival_radius_m=c['arrival_radius_m'],vehicle_radius_m=c['vehicle_radius_m'],
             boundary_buffer_m=c['boundary_buffer_m'],estimated_speed_m_s=c['speed_m_s'],estimated_waypoint_delay_s=c['settle_s'],
             max_segment_m=100.,sweep_axis='auto')
    # Score both axis/end/order variants using actual acceleration-limited legs.
    candidates=[]
    for axis in ('x','y'):
        p=plan(dict(raw,sweep_axis=axis))
        segments=p['scan_segments']
        for reverse in (False,True):
            for flip in (False,True):
                ordered=list(reversed(segments)) if reverse else segments
                route=[[0.,0.]]
                for a,b in ordered:
                    pair=(b,a) if flip else (a,b)
                    for v in pair:
                        if math.dist(route[-1],v)>1e-8: route.append(list(v))
                if math.dist(route[-1],(0,0))>1e-8: route.append([0.,0.])
                duration=sum(leg_time(math.dist(a,b),c['speed_m_s'],c['acceleration_m_s2'])+c['settle_s']+2.
                             for a,b in zip(route,route[1:]))
                q=copy.deepcopy(p)
                q.update(route=route,estimated_route_time_s=duration,distance_m=sum(math.dist(a,b) for a,b in zip(route,route[1:])))
                candidates.append(q)
    p=min(candidates,key=lambda p:p['estimated_route_time_s'])
    p['footprint_basis']='Conservative geometric estimate; measured detection performance still required'
    p['mission_config']=c
    p['coordinate_frame']='Arena x/y metres relative to launch center; rotated into captured PX4 local frame at runtime'
    p['tag_count_behavior']='Return after required unique tags are confirmed, full scan ends, or return reserve is reached'
    return p


def arena_to_local(p,home,heading_deg):
    a=math.radians(heading_deg); co,si=math.cos(a),math.sin(a)
    return [home[0]+co*p[0]-si*p[1],home[1]+si*p[0]+co*p[1]]


def local_to_arena(p,home,heading_deg):
    return arena_to_local([p[0]-home[0],p[1]-home[1]],[0,0],-heading_deg)


def leg_time(distance,speed,accel):
    if distance<=0:return 0.
    return 2*math.sqrt(distance/accel) if distance<speed*speed/accel else distance/speed+speed/accel


def trajectory(delta,speed,accel,dt):
    """Trapezoidal/triangular reference, rest-to-rest; offsets from actual hover."""
    distance=math.sqrt(sum(x*x for x in delta))
    if distance<1e-8:return [(0.,0.,0.),(0.,0.,0.)]
    duration=leg_time(distance,speed,accel)
    steps=math.ceil(duration/dt)
    if steps+1>2400:raise ValueError('Leg exceeds stock 2400-point buffer; split the route')
    stretch=steps*dt/duration
    peak=min(speed,math.sqrt(distance*accel)); ta=peak/accel
    def s(t):
        if t<ta:return .5*accel*t*t
        if t<=duration-ta:return .5*accel*ta*ta+peak*(t-ta)
        return distance-.5*accel*(duration-t)**2
    pts=[tuple(d*s(i*dt/stretch)/distance for d in delta) for i in range(steps+1)]
    pts[0]=(0.,0.,0.);pts[-1]=tuple(delta)
    return pts


def rotation(roll,pitch,yaw):
    cr,sr=math.cos(roll),math.sin(roll); cp,sp=math.cos(pitch),math.sin(pitch); cy,sy=math.cos(yaw),math.sin(yaw)
    return ((cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr),(sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr),(-sp,cp*sr,cp*cr))


def mul(R,v):return [sum(a*b for a,b in zip(row,v)) for row in R]


def tag_local(t,position,rpy,camera):
    p=math.radians(camera['pitch_down_deg'])
    mount=((0.,-math.sin(p),math.cos(p)),(1.,0.,0.),(0.,math.cos(p),math.sin(p)))
    rotated=mul(rotation(0,0,math.radians(camera['image_rotation_deg'])),t)
    body=[a+b for a,b in zip(mul(mount,rotated),camera['position_body_m'])]
    return [a+b for a,b in zip(position,mul(rotation(*rpy),body))]


class History:
    """Time-sorted vector observations. Reject stale extrapolation; wrap angles."""
    def __init__(self,angles=False): self.values=[];self.angles=angles
    def add(self,t,v):
        if not math.isfinite(t) or not all(math.isfinite(x) for x in v):return
        self.values.append((t,tuple(v)));self.values.sort(key=lambda p:p[0]);self.values=self.values[-600:]
    def at(self,t,max_gap=.2):
        if not self.values:return None
        times=[p[0] for p in self.values];i=bisect.bisect_left(times,t)
        if i==0 or i==len(times):
            s=self.values[min(i,len(times)-1)]
            return s[1] if abs(s[0]-t)<=max_gap else None
        a,b=self.values[i-1],self.values[i]
        if max(t-a[0],b[0]-t)>max_gap:return None
        f=(t-a[0])/(b[0]-a[0]) if b[0]!=a[0] else 0.
        return tuple(x+f*((y-x+math.pi)%(2*math.pi)-math.pi if self.angles else y-x) for x,y in zip(a[1],b[1]))


class TagRegistry:
    def __init__(self,c):self.c=c;self.observations={};self.frames=set();self.lock=threading.RLock()
    def add(self,tag_id,frame_id,t,xyz):
        with self.lock:return self._add(tag_id,frame_id,t,xyz)
    def _add(self,tag_id,frame_id,t,xyz):
        key=(tag_id,frame_id)
        if key in self.frames:return False
        self.frames.add(key)
        ids=self.c.get('allowed_tag_ids')
        if not 0<=tag_id<22 or (ids is not None and tag_id not in ids) or not all(math.isfinite(x) for x in xyz):return False
        rows=self.observations.setdefault(tag_id,[])
        if len(rows)>=3:
            center=[statistics.median(r[1][i] for r in rows) for i in range(2)]
            if math.dist(center,xyz[:2])>self.c['tag_outlier_radius_m']:return False
        rows.append((t,list(xyz)));self.observations[tag_id]=rows[-200:]
        return True
    def results(self):
        with self.lock:return self._results()
    def _results(self):
        result={}
        for tid,rows in self.observations.items():
            pos=[statistics.median(r[1][i] for r in rows) for i in range(3)]
            spread=max(math.dist(r[1][:2],pos[:2]) for r in rows)
            confirmed=len(rows)>=self.c['min_tag_observations'] and max(r[0] for r in rows)-min(r[0] for r in rows)>=self.c['min_tag_span_s'] and spread<=self.c['tag_max_spread_m']
            result[str(tid)]=dict(position_local_ned_m=pos,observations=len(rows),horizontal_spread_m=spread,confirmed=confirmed)
        return result
    def complete(self):return sum(r['confirmed'] for r in self.results().values())>=self.c['expected_tags']


@dataclass
class MissionResult:
    status:str
    returned:bool=False
    landed:bool=False
    reason:str=''


def execute(io,c,route,home,registry):
    """Testable mission sequencing; transport owns fresh-state and physical checks."""
    result=MissionResult('not_started'); started=False
    try:
        io.event('TAKEOFF_COMMAND',home_local_ned_m=home)
        started=True;io.takeoff()
        for point in route[1:-1]:
            if registry.complete():
                io.event('SEARCH_FINISHED',reason='all_tags_confirmed');break
            remaining=c['mission_timeout_s']-(io.now()-io.start_time)
            goal=arena_to_local(point,home,c['arena_heading_deg'])
            if remaining<io.return_cost(via=goal)+c['return_reserve_s']:
                io.event('SEARCH_FINISHED',reason='return_reserve');break
            io.event('SEARCH_GOAL',arena_xy_m=point)
            io.move(goal,home[2]-c['altitude_m'])
        io.event('RETURN_STARTED')
        io.move(home[:2],home[2]-c['altitude_m'],returning=True)
        result.returned=True;io.event('HOME_REACHED')
        io.event('LAND_COMMAND');io.land();result.landed=True
        result.status='complete' if registry.complete() else 'partial'
        io.event('MISSION_FINISHED',status=result.status,returned=True,landed=True,tags=registry.results())
    except (Exception,KeyboardInterrupt) as exc:
        result.status='aborted';result.reason=str(exc) or 'interrupted'
        io.event('MISSION_ABORTED',reason=result.reason)
        if started:
            # Same laptop-side error response as upstream; no new onboard Wi-Fi watchdog.
            try:io.abort_land()
            except Exception as err:io.event('ABORT_LAND_UNCONFIRMED',reason=str(err))
    return result
