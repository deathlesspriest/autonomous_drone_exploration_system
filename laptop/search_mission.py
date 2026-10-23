#!/usr/bin/env python3
"""Plan, simulate, monitor or fly the single-UAV course search mission.

Default mode is --plan (no networking). Only --fly can issue flight commands.
"""
from __future__ import annotations
import argparse
from dataclasses import asdict
from datetime import datetime
import json
import math
from pathlib import Path
import time

from search_core import (load_config,example_config,validate,geometry,TagRegistry,
                        execute,local_to_arena,trajectory,leg_time)
from search_geometry import write_outputs


class SimulatedFlight:
    """Deterministic kinematic stub, not PX4/SITL or a physical flight model."""
    def __init__(self,c,rec,home,registry,targets):
        self.c=c;self.rec=rec;self.home=home;self.position=list(home);self.registry=registry
        self.targets=targets;self.clock=0.;self.start_time=0.;self.frame=0
    def now(self):return self.clock
    def event(self,name,**kw):self.rec.event(name,simulation_time_s=self.clock,**kw)
    def takeoff(self):self.clock+=12.;self.position[2]=self.home[2]-self.c['altitude_m'];self.event('TAKEOFF_CONFIRMED')
    def return_cost(self,via=None):
        points=[self.position[:2]]+([via] if via is not None else [])+[self.home[:2]]
        return sum(leg_time(math.dist(a,b),self.c['speed_m_s'],self.c['acceleration_m_s2'])+self.c['settle_s']+2.
                   for a,b in zip(points,points[1:]))+self.c['landing_timeout_s']+5.
    def move(self,xy,z,returning=False):
        origin=list(self.position);delta=[xy[0]-origin[0],xy[1]-origin[1],z-origin[2]]
        for p in trajectory(delta,self.c['speed_m_s'],self.c['acceleration_m_s2'],self.c['sample_period_s']):
            self.clock+=self.c['sample_period_s'];self.position=[a+b for a,b in zip(origin,p)];self.frame+=1
            self.rec.event('PX4_POSITION',simulation=True,simulation_time_s=self.clock,position=self.position,velocity=[0,0,0])
            for tid,xyz in self.targets.items():
                if math.dist(self.position[:2],xyz[:2])<.25:
                    self.registry.add(tid,self.frame,self.clock,xyz)
        self.clock+=self.c['settle_s'];self.event('GOAL_REACHED',goal=self.position,returning=returning)
    def land(self):self.clock+=8.;self.position[2]=self.home[2];self.event('LANDING_CONFIRMED',position_local_ned_m=self.position)
    def abort_land(self):self.event('ABORT_LAND_UNCONFIRMED')


def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    mode=ap.add_mutually_exclusive_group()
    mode.add_argument('--plan',action='store_true');mode.add_argument('--simulate',action='store_true')
    mode.add_argument('--monitor',action='store_true');mode.add_argument('--fly',action='store_true')
    mode.add_argument('--inspect-tags',action='store_true',help='Disarmed tag/local-frame inspection; no flight commands')
    ap.add_argument('--config',type=Path,default=Path(__file__).with_name('search_configs')/'mission.json')
    ap.add_argument('--example',action='store_true',help='Illustrative values for offline plan/simulation only')
    ap.add_argument('--drone-id',type=int,help='Override ID, useful with --monitor')
    ap.add_argument('--out',type=Path)
    ap.add_argument('--seconds',type=float,default=15.,help='Read-only monitoring duration')
    args=ap.parse_args(argv)
    c=example_config() if args.example else load_config(args.config)
    if args.drone_id is not None:c['drone_id']=args.drone_id
    if args.example and (args.fly or args.monitor or args.inspect_tags):ap.error('--example is offline-only')
    from search_io import Recorder
    out=args.out or Path('search_runs')/datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    if args.monitor:
        if not math.isfinite(args.seconds) or not 0<args.seconds<=300:ap.error('--seconds must be in (0,300]')
        if not isinstance(c['drone_id'],int) or not 1<=c['drone_id']<=54:ap.error('--monitor needs --drone-id 1..54')
        from search_io import Link
        rec=Recorder(out);link=Link(c,rec)
        try:
            link.start();end=time.monotonic()+args.seconds
            while time.monotonic()<end:
                s=link.state()
                print({k:(None if v is None else str(v[1:])) for k,v in s.items()},flush=True);time.sleep(1.)
        finally:link.close();rec.close()
        return 0
    if args.inspect_tags:
        try:validate(c)
        except (ValueError,KeyError,TypeError) as e:ap.error(str(e))
        if not math.isfinite(args.seconds) or not 0<args.seconds<=300:ap.error('--seconds must be in (0,300]')
        from search_io import Link,Vision
        rec=Recorder(out);link=Link(c,rec);vision=None;registry=TagRegistry(c)
        try:
            link.start();until=time.monotonic()+20
            while True:
                s=link.state();now=time.monotonic()
                ready=all(s[k] is not None and now-s[k][0]<limit for k,limit in
                          (('position',c['position_stale_s']),('attitude',c['position_stale_s']),('heartbeat',2.5),('landed',2.5)))
                if ready:break
                if now>until:raise RuntimeError('Inspection needs fresh PX4 pose, heartbeat and landed state')
                time.sleep(.1)
            if s['heartbeat'][1] or not s['landed'][1]:raise RuntimeError('Begin inspection on the floor and disarmed')
            home=list(s['position'][1]);rec.save('home.json',dict(position_local_ned_m=home,arena_heading_deg=c['arena_heading_deg']))
            rec.save('config.json',c);vision=Vision(c,link,rec,registry,home);vision.start();vision.active=True
            rec.event('INSPECTION_STARTED',seconds=args.seconds,home_local_ned_m=home)
            end=time.monotonic()+args.seconds
            while time.monotonic()<end:
                s=link.state()
                if not s['heartbeat'] or time.monotonic()-s['heartbeat'][0]>2.5 or s['heartbeat'][1]:
                    raise RuntimeError('Inspection stopped: heartbeat stale or vehicle armed')
                if vision.error:raise RuntimeError(str(vision.error))
                time.sleep(.1)
            rec.save('tags.json',registry.results());rec.event('INSPECTION_FINISHED',tags=registry.results())
            return 0
        except (Exception,KeyboardInterrupt) as e:
            rec.event('INSPECTION_FAILED',reason=str(e));return 2
        finally:
            if vision:vision.close()
            link.close();rec.close()
    try:validate(c,live=args.fly);p=geometry(c)
    except (ValueError,KeyError,TypeError) as e:ap.error(str(e))
    rec=Recorder(out)
    rec.save('config.json',c);write_outputs(p,out/'plan')
    rec.event('PLAN_READY',simulation=bool(args.simulate),example_only=c['example_only'],lanes=p['lanes'],distance_m=p['distance_m'],
              estimated_route_time_s=p['estimated_route_time_s'],reference_speed_m_s=c['speed_m_s'])
    if p['estimated_route_time_s']+c['takeoff_timeout_s']+c['return_reserve_s']>c['mission_timeout_s']:
        rec.event('PLAN_TIME_WARNING',reason='Full scan may exceed available time; mission may return with partial results. Calibrate footprint or improve validated settings.')
    if not args.fly and not args.simulate:
        rec.close();print(f'Offline plan saved to {out}. No UAV connection.');return 0
    registry=TagRegistry(c);link=vision=None;result=None
    try:
        if args.simulate:
            from search_core import arena_to_local
            home=[1.2,-.7,0.]
            area=c['search_area']
            targets={i:[*arena_to_local([area['x_min_m']+f*area['length_x_m'],area['y_min_m']+g*area['width_y_m']],home,c['arena_heading_deg']),0.]
                     for i,(f,g) in enumerate(((.1,.15),(.55,.55),(.9,.85)))}
            io=SimulatedFlight(c,rec,home,registry,targets)
            rec.save('simulation_truth.json',targets)
        else:
            from search_io import Link,Vision,Flight
            link=Link(c,rec);link.start()
            deadline=time.monotonic()+20
            while True:
                s=link.state();now=time.monotonic()
                ready=all(s[k] is not None and now-s[k][0]<limit for k,limit in (('position',c['position_stale_s']),('attitude',c['position_stale_s']),('heartbeat',2.5),('custom',c['custom_stale_s']),('landed',2.5)))
                if ready:break
                if now>deadline:raise RuntimeError('Preflight needs position, attitude, heartbeat, landed-state and ESP telemetry. Close QGC/viewers and check IP/bridge.')
                time.sleep(.1)
            if s['heartbeat'][1] or not s['landed'][1]:raise RuntimeError('Start with UAV landed and disarmed')
            home=list(s['position'][1])
            if abs(home[2])>.1:raise RuntimeError('Stock takeoff uses absolute z=-0.5: local floor z must be near 0. Reinitialize on the launch floor.')
            vision=Vision(c,link,rec,registry,home);vision.start()
            until=time.monotonic()+20
            while not vision.last_usable_frame:
                if vision.error:raise RuntimeError(str(vision.error))
                if time.monotonic()>until:raise RuntimeError('No calibrated camera frames received')
                time.sleep(.1)
            io=Flight(c,p,link,rec,home,vision)
            io.check(flying=False)
            answer=input(f"Type SEARCH-{c['drone_id']} to take off and search ({c['expected_tags']} tags): ")
            if answer!=f"SEARCH-{c['drone_id']}":
                rec.event('CANCELLED_BEFORE_TAKEOFF');return 0
            s=io.check(flying=False)
            if s['heartbeat'][1] or not s['landed'][1] or math.dist(s['position'][1],home)>.03:
                raise RuntimeError('Vehicle moved or armed during confirmation; restart preflight')
        rec.save('home.json',dict(position_local_ned_m=home,arena_heading_deg=c['arena_heading_deg']))
        result=execute(io,c,p['route'],home,registry)
        rec.save('result.json',asdict(result));rec.save('tags.json',registry.results())
        return 0 if result.status=='complete' else 2
    except (Exception,KeyboardInterrupt) as e:
        rec.event('PREFLIGHT_FAILED',reason=str(e));return 2
    finally:
        if vision:vision.close()
        if link:link.close()
        rec.close()
        print(f'Records: {out}')


if __name__=='__main__':raise SystemExit(main())
