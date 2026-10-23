#!/usr/bin/env python3
"""Plot recorded mission data, optionally combined with the actual PX4 ULog.

Default plot is telemetry-only and labeled as such; --ulog is required for the
project's post-flight ULog deliverable. Never invent a missing flight log.
"""
import argparse
import csv
import json
import math
from pathlib import Path


def read_ulog(path):
    from pyulog import ULog
    import numpy as np
    log=ULog(str(path),['vehicle_local_position'])
    ds=log.get_dataset('vehicle_local_position').data
    for key in ('xy_reset_counter','z_reset_counter'):
        if key in ds and np.any(np.diff(ds[key].astype(int))!=0):
            raise ValueError('Ulog contains local position resets. Resolve frame changes before combining tag locations.')
    t=ds['timestamp']/1e6
    mask=np.isfinite(ds['x'])&np.isfinite(ds['y'])&np.isfinite(ds['z'])
    if 'xy_valid' in ds:mask &= ds['xy_valid'].astype(bool)
    if 'z_valid' in ds:mask &= ds['z_valid'].astype(bool)
    return t[mask],np.column_stack([ds[k][mask] for k in ('x','y','z')])


def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('run',type=Path);ap.add_argument('--ulog',type=Path)
    ap.add_argument('--references',type=Path,help='Optional CSV: id,x,y in the same PX4 local frame')
    args=ap.parse_args(argv)
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows=[json.loads(line) for line in (args.run/'events.jsonl').read_text().splitlines()]
    recorded=[r for r in rows if r['event']=='PX4_POSITION']
    if not recorded:ap.error('No recorded positions to analyze')
    simulated=any(r.get('simulation') for r in recorded)
    tags=json.loads((args.run/'tags.json').read_text())
    home=json.loads((args.run/'home.json').read_text())['position_local_ned_m']
    result=json.loads((args.run/'result.json').read_text())
    if args.ulog:
        if simulated:ap.error('Do not combine synthetic mission data with a real ULog')
        if not all('px4_boot_ms' in r for r in recorded):ap.error('Run lacks PX4 clock references')
        t,xyz=read_ulog(args.ulog)
        lo,hi=recorded[0]['px4_boot_ms']/1000,recorded[-1]['px4_boot_ms']/1000
        keep=(t>=lo)&(t<=hi);t,xyz=t[keep],xyz[keep]
        if len(t)<2:ap.error('No overlapping PX4 timestamps; check the selected ULog')
        # Different boots can share uptime values. Require spatial agreement too.
        residual=[]
        for r in recorded[::max(1,len(recorded)//100)]:
            tt=r['px4_boot_ms']/1000
            if t[0]<=tt<=t[-1]:residual.append(math.dist([np.interp(tt,t,xyz[:,i]) for i in range(3)],r['position']))
        if not residual or np.median(residual)>.15:ap.error('ULog and telemetry positions disagree; check flight identity/frame')
        source='PX4 ULog + recorded AprilTags'
    else:
        xyz=np.asarray([r['position'] for r in recorded]);source='SIMULATION — synthetic data' if simulated else 'Live telemetry only — ULog still required'
    confirmed={k:v for k,v in tags.items() if v['confirmed']}
    fig,ax=plt.subplots(figsize=(9,7))
    ax.plot(xyz[:,0],xyz[:,1],color='#2563eb',lw=1.5,label='Flight trajectory')
    ax.scatter(home[0],home[1],s=90,c='#16a34a',marker='o',label='Start')
    land=[r for r in rows if r['event']=='LANDING_CONFIRMED']
    if land:
        p=land[-1]['position_local_ned_m'];ax.scatter(p[0],p[1],s=100,c='#ea580c',marker='x',label='Confirmed landing')
    for k,v in confirmed.items():
        x,y,_=v['position_local_ned_m'];ax.scatter(x,y,marker='s',s=65,c='#7c3aed');ax.annotate(f'Tag {k}',(x,y),xytext=(6,6),textcoords='offset points')
    ax.set(xlabel='Local x / North (m)',ylabel='Local y / East (m)',title=source)
    ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.25);ax.legend();fig.tight_layout()
    fig.savefig(args.run/'flight_analysis.png',dpi=180);plt.close(fig)
    summary=dict(source=source,status=result['status'],confirmed_tags=len(confirmed),returned=result['returned'],landed=result['landed'],
                 ulog_used=bool(args.ulog),simulation=simulated,timestamp_alignment='PX4 boot clock for ULog; camera timing approximate')
    takeoff=next((r for r in rows if r['event']=='TAKEOFF_COMMAND'),None)
    if takeoff and land:
        key='simulation_time_s' if simulated else 'host_monotonic_s'
        summary['mission_duration_s']=land[-1][key]-takeoff[key]
        summary['landing_error_m']=math.dist(land[-1]['position_local_ned_m'][:2],home[:2])
    if args.references:
        with args.references.open() as f:refs={r['id']:[float(r['x']),float(r['y'])] for r in csv.DictReader(f)}
        errors={k:math.dist(v['position_local_ned_m'][:2],refs[k]) for k,v in confirmed.items() if k in refs}
        summary['tag_horizontal_errors_m']=errors
        if errors:summary['tag_rmse_m']=math.sqrt(sum(e*e for e in errors.values())/len(errors))
    (args.run/'analysis.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
