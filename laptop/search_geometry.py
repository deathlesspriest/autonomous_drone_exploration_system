"""Offline rectangular coverage planner. Standard-library Python 3.10+ only.

Coordinates: arena x/y relative to launch center, in metres.
No network operations or flight commands. The course executor controls altitude,
speed, yaw and landing. Its actual behaviour must match the planning assumptions.
"""
from __future__ import annotations

import copy
import html
import json
import math
from pathlib import Path

EPS = 1e-9
MAX_POINTS = 10000


def number(value, name, minimum=0.0, strictly_positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    if value < minimum or (strictly_positive and value <= minimum):
        raise ValueError(f"{name} must be {'greater than' if strictly_positive else 'at least'} {minimum}")
    return float(value)


def rectangle(data, name):
    x = number(data['x_min_m'], name + '.x_min_m', -math.inf)
    y = number(data['y_min_m'], name + '.y_min_m', -math.inf)
    w = number(data['length_x_m'], name + '.length_x_m', strictly_positive=True)
    h = number(data['width_y_m'], name + '.width_y_m', strictly_positive=True)
    if not all(math.isfinite(v) for v in (x+w, y+h)):
        raise ValueError(name + ' bounds overflow')
    return [x, y, x+w, y+h]


def inside(p, rect):
    return rect[0]-EPS <= p[0] <= rect[2]+EPS and rect[1]-EPS <= p[1] <= rect[3]+EPS


def axis_limits(lo, hi, allowed_lo, allowed_hi, half):
    # Centre locations cover the full requested interval, including its edges.
    if hi-lo <= 2*half:
        a = max(allowed_lo, hi-half)
        b = min(allowed_hi, lo+half)
        if a > b+EPS:
            raise ValueError('Search area cannot be covered from inside the permitted flight boundary')
        c = min(max((lo+hi)/2, a), b)
        return c, c
    first = max(lo+half, allowed_lo)
    last = min(hi-half, allowed_hi)
    if first > last+EPS or first-half > lo+EPS or last+half < hi-EPS:
        raise ValueError('Search area cannot be covered with the configured footprint and boundary clearance')
    return first, last


def positions(first, last, max_spacing):
    if abs(last-first) <= EPS:
        return [first]
    count = math.ceil((last-first)/max_spacing)+1
    if count > MAX_POINTS//2:
        raise ValueError('Too many search lanes; check footprint, overlap and dimensions')
    return [first+(last-first)*i/(count-1) for i in range(count)]


def interpolate_route(points, max_segment):
    result = [list(points[0])]
    for p in points[1:]:
        a = result[-1]
        distance = math.dist(a, p)
        if distance <= EPS:
            continue
        n = max(1, math.ceil(distance/max_segment))
        if len(result)+n > MAX_POINTS:
            raise ValueError('Route exceeds 10000 waypoints; increase max_segment_m or check dimensions')
        result.extend([[a[0]+(p[0]-a[0])*i/n, a[1]+(p[1]-a[1])*i/n] for i in range(1, n+1)])
        result[-1] = list(p)  # Preserve exact goals, especially the final home coordinate.
    # Even when the entire search is directly over home, retain one hold/home goal.
    return result if len(result)>1 else [[0.0, 0.0], [0.0, 0.0]]


def plan(config):
    c = copy.deepcopy(config)
    search = rectangle(c['search_area'], 'search_area')
    flight = rectangle(c['flight_area'], 'flight_area')
    tags = c['expected_tags']
    if isinstance(tags, bool) or not isinstance(tags, int) or tags < 1:
        raise ValueError('expected_tags must be a positive integer')
    for key in ('altitude_m', 'max_altitude_m', 'estimated_speed_m_s', 'max_segment_m', 'arrival_radius_m'):
        number(c[key], key, strictly_positive=True)
    for key in ('tracking_error_m', 'vehicle_radius_m', 'boundary_buffer_m', 'estimated_waypoint_delay_s'):
        number(c[key], key)
    if c['altitude_m'] > c['max_altitude_m']:
        raise ValueError('altitude_m exceeds max_altitude_m')
    overlap = number(c['overlap_fraction'], 'overlap_fraction')
    if overlap >= 1:
        raise ValueError('overlap_fraction must be less than 1')
    if c['sweep_axis'] not in ('auto', 'x', 'y'):
        raise ValueError('sweep_axis must be auto, x or y')
    cam = c['camera']
    usable = number(cam['usable_fraction'], 'camera.usable_fraction', strictly_positive=True)
    if usable > 1:
        raise ValueError('camera.usable_fraction must not exceed 1')
    for key in ('horizontal_fov_deg', 'vertical_fov_deg'):
        angle = number(cam[key], 'camera.'+key, strictly_positive=True)
        if angle >= 179:
            raise ValueError('Camera FOV must be below 179 degrees')
    measured = cam.get('measured_reliable_radius_m')
    if measured is None:
        radius = c['altitude_m']*math.tan(math.radians(min(cam['horizontal_fov_deg'], cam['vertical_fov_deg'])/2))*usable
        basis = 'FOV estimate: uncalibrated, verify with real tags'
    else:
        radius = number(measured, 'camera.measured_reliable_radius_m', strictly_positive=True)
        basis = 'Measured reliable radius at the configured altitude'
    # A disk inside the downward-camera footprint is invariant to yaw. An
    # inscribed square inside the error-eroded disk gives simple conservative
    # continuous-strip coverage, even if the course firmware turns at waypoints.
    error = c['tracking_error_m']+c['arrival_radius_m']
    effective_radius = radius-error
    if effective_radius <= 0:
        raise ValueError('Reliable camera radius must exceed tracking_error_m + arrival_radius_m')
    half = effective_radius/math.sqrt(2)
    spacing = 2*half*(1-overlap)
    if spacing < 1e-6:
        raise ValueError('Resulting lane spacing is too small')
    clearance = c['vehicle_radius_m']+c['boundary_buffer_m']+error
    allowed = [flight[0]+clearance, flight[1]+clearance, flight[2]-clearance, flight[3]-clearance]
    if allowed[0] > allowed[2] or allowed[1] > allowed[3] or not inside((0, 0), allowed):
        raise ValueError('Takeoff (0,0) must lie inside the flight area with the configured clearance')
    if not inside(search[:2], flight) or not inside(search[2:], flight):
        raise ValueError('The search area must be inside the flight area')
    x0, x1 = axis_limits(search[0], search[2], allowed[0], allowed[2], half)
    y0, y1 = axis_limits(search[1], search[3], allowed[1], allowed[3], half)
    candidates = []
    for axis in ('x', 'y') if c['sweep_axis']=='auto' else (c['sweep_axis'],):
        lane_values = positions(y0, y1, spacing) if axis=='x' else positions(x0, x1, spacing)
        for reverse_lanes in (False, True):
            for reverse_first in (False, True):
                values = list(reversed(lane_values)) if reverse_lanes else lane_values
                scan, strips = [], []
                for i, value in enumerate(values):
                    a, b = ([x0, value], [x1, value]) if axis=='x' else ([value, y0], [value, y1])
                    if bool(i%2) != reverse_first:
                        a, b = b, a
                    scan.extend([a, b])
                    strips.append([a, b])
                route = interpolate_route([[0., 0.]]+scan+[[0., 0.]], c['max_segment_m'])
                if not all(inside(p, allowed) for p in route):
                    raise ValueError('A generated point violates the flight boundary')
                distance = sum(math.dist(a,b) for a,b in zip(route, route[1:]))
                estimate = distance/c['estimated_speed_m_s']+(len(route)-1)*c['estimated_waypoint_delay_s']
                candidates.append({'axis':axis, 'reverse_lanes':reverse_lanes, 'reverse_first':reverse_first,
                    'lanes':len(values), 'route':route, 'scan_segments':strips, 'distance_m':distance,
                    'estimated_route_time_s':estimate})
    best = min(candidates, key=lambda p:(p['estimated_route_time_s'], p['distance_m'], p['lanes']))
    best.update({'config':c, 'search_bounds':search, 'flight_bounds':flight, 'allowed_center_bounds':allowed,
                 'reliable_radius_m':radius, 'effective_radius_m':effective_radius,
                 'coverage_half_width_m':half, 'max_lane_spacing_m':spacing, 'footprint_basis':basis,
                 'coordinate_frame':'arena x/y relative to launch center, metres',
                 'expected_tags':tags, 'tag_count_behavior':'metadata only; full coverage, no automatic early return',
                 'candidate_scores':[{k:v for k,v in p.items() if k in ('axis','reverse_lanes','reverse_first','distance_m','estimated_route_time_s','lanes')} for p in candidates]})
    return best


def preview(p):
    bounds = p['flight_bounds']
    scale = min(820/(bounds[2]-bounds[0]), 530/(bounds[3]-bounds[1]))
    def xy(point):
        return 70+(point[0]-bounds[0])*scale, 610-(point[1]-bounds[1])*scale
    def rect(b, fill, stroke, dash=''):
        x,y = xy((b[0],b[3]))
        return f'<rect x="{x:.2f}" y="{y:.2f}" width="{(b[2]-b[0])*scale:.2f}" height="{(b[3]-b[1])*scale:.2f}" fill="{fill}" stroke="{stroke}" stroke-dasharray="{dash}"/>'
    parts = [rect(bounds,'#f1f5f9','#94a3b8'),rect(p['allowed_center_bounds'],'none','#94a3b8','5 5')]
    half = p['coverage_half_width_m']
    for a,b in p['scan_segments']:
        parts.append(rect([min(a[0],b[0])-half,min(a[1],b[1])-half,max(a[0],b[0])+half,max(a[1],b[1])+half],'#bfdbfe66','none'))
    parts.append(rect(p['search_bounds'],'none','#059669','7 4'))
    points = ' '.join(f'{x:.2f},{y:.2f}' for x,y in map(xy,p['route']))
    parts.append(f'<polyline points="{points}" fill="none" stroke="#2563eb" stroke-width="2.5"/>')
    for i,point in enumerate(p['route'][1:],1):
        x,y = xy(point)
        parts.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="3" fill="#2563eb"/><title>Waypoint {i}: {point[0]:.3f}, {point[1]:.3f}</title>')
    x,y = xy((0,0))
    parts.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="7" fill="#ea580c"/><text x="{x+12:.2f}" y="{y-12:.2f}" font-size="15">Start / return (0, 0)</text>')
    for point in (bounds[:2],bounds[2:]):
        x,y=xy(point)
        parts.append(f'<text x="{x:.2f}" y="{y-9:.2f}" text-anchor="middle" font-size="12">({point[0]:g}, {point[1]:g})</text>')
    rows=''.join(f'<tr><td>{i}</td><td>{a:.3f}</td><td>{b:.3f}</td></tr>' for i,(a,b) in enumerate(p['route'][1:],1))
    return f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>UAV search route</title>
<style>body{{font:16px system-ui;color:#172033;background:#f8fafc;margin:0;padding:32px}}main{{max-width:1000px;margin:auto}}h1{{margin-bottom:8px}}.sub{{color:#526175}}.cards{{display:flex;gap:12px;flex-wrap:wrap;margin:24px 0}}.card,section{{background:white;border:1px solid #dce3ed;border-radius:12px;padding:18px}}.card strong{{display:block;font-size:26px}}svg{{width:100%;height:auto}}table{{border-collapse:collapse;width:100%;margin-top:18px}}td,th{{text-align:left;border-bottom:1px solid #e2e8f0;padding:8px}}.note{{border-left:4px solid #d97706;padding:12px 18px;background:#fffbeb;line-height:1.6}}summary{{cursor:pointer}}code{{background:#eef2f7;padding:2px 5px}}</style>
<main><h1>Rectangular search route</h1><div class="sub">Downward camera · arena x/y coordinates relative to launch center · offline preview</div>
<div class="cards"><div class="card"><strong>{p['lanes']}</strong>scan lanes · {p['axis'].upper()} direction</div><div class="card"><strong>{p['distance_m']:.1f} m</strong>route including return</div><div class="card"><strong>{p['estimated_route_time_s']:.0f} s</strong>rough route estimate</div><div class="card"><strong>{p['expected_tags']}</strong>expected tags</div></div>
<section><svg viewBox="0 0 1000 670" role="img" aria-label="Search rectangle, coverage strips, flight boundary and route">{''.join(parts)}<text x="70" y="650" font-size="14">arena x →     arena y ↑     Coordinates in metres (plot orientation is intentional)</text></svg>
<p>Blue: route and modeled coverage · Green dashed: search rectangle · Gray outer: flight boundary · Gray inner: permitted waypoint centers · Orange: takeoff and return.</p></section>
<p class="note">{html.escape(p['footprint_basis'])}. Coverage is geometric and assumes continuous imaging, near-level flight, the configured downward-camera offset and error bounds. This is not a measured detection result. The mission runner converts arena coordinates to the captured PX4 local frame and commands the configured height and reference speed. Time excludes takeoff/landing and is not a battery prediction.</p>
<p>Planning altitude: <b>{p['config']['altitude_m']:g} m</b> · Maximum lane spacing: <b>{p['max_lane_spacing_m']:.3f} m</b> · {len(p['route'])-1} exported waypoints. The mission runner may return early after enough unique tags are confirmed or when its time reserve is reached.</p>
<details><summary>View waypoints in execution order</summary><table><tr><th>Waypoint</th><th>Arena x (m)</th><th>Arena y (m)</th></tr>{rows}</table></details>
<p>Edit <code>mission.json</code>, then run <code>python laptop/search_mission.py --config laptop/search_configs/mission.json --plan --out search_runs/plan</code> to regenerate.</p></main></html>'''


def write_outputs(p, out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    (out/'plan.json').write_text(json.dumps(p, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    header = '# Preview only: arena x,y metres relative to launch center.\n# Do not feed this file to the stock waypoint runner. Use search_mission.py, which captures home and rotates coordinates.\n'
    (out/'waypoints.txt').write_text(header+''.join(f'{x:.9f},{y:.9f}\n' for x,y in p['route'][1:]), encoding='utf-8')
    (out/'preview.html').write_text(preview(p), encoding='utf-8')
