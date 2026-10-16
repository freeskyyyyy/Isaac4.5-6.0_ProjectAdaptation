#!/usr/bin/env python3
"""Review saved native odom/received Twist timing. Never initializes ROS or sends commands."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def summarize(rows, stamp_key, trigger):
    def metrics(samples):
        if not samples:
            return {'samples': 0}
        t=np.asarray([r[stamp_key] for r in samples],dtype=float)
        w=np.asarray([r['wall_seconds'] for r in samples],dtype=float)
        v=np.asarray([r['xy_wz'] for r in samples],dtype=float)
        if not (np.isfinite(t).all() and np.isfinite(w).all() and np.isfinite(v).all()):
            raise ValueError('nonfinite feedback data')
        dt=np.diff(t); dw=np.diff(w); valid=dt>1e-9
        derivative=np.abs(np.diff(v,axis=0)[valid]/dt[valid,None])
        return {'samples':len(samples),'nonpositive_intervals':int((~valid).sum()),
                'maximum_absolute_xy_wz':np.abs(v).max(axis=0).tolist(),
                'maximum_absolute_sample_derivative_xy_wz':derivative.max(axis=0).tolist() if len(derivative) else [0.,0.,0.],
                'median_sim_dt_s':float(np.median(dt[valid])) if valid.any() else None,
                'median_wall_dt_s':float(np.median(dw[valid])) if valid.any() else None}
    return {'all':metrics(rows),'ordinary':metrics([r for r in rows if r[stamp_key]<trigger])}


def review(report):
    trigger=report['stop_trigger']['sim_seconds']
    commands={topic:summarize([r for r in report['velocity_history'] if r['topic']==topic],
                              'receipt_sim_seconds',trigger)
              for topic in ('/cmd_vel_nav','/cmd_vel')}
    odom=summarize(report['odom_history'],'sim_seconds',trigger)
    start=report['replanning_history'][0]['observed_start_connector_xy']
    grid_start=report['replanning_history'][0]['points_xy'][0]
    native=[r for r in report['odom_history'] if r['sim_seconds']<=trigger]
    newest=native[-1]
    averages={str(window):np.mean([r['xy_wz'] for r in native
               if newest['sim_seconds']-window<=r['sim_seconds']],axis=0).tolist()
              for window in (.05,.1)}
    return {'scope':'offline feedback diagnosis only; no navigation or physical acceleration acceptance',
            'motion_status':report['status'],'cutoff_before_predictive_stop_sim_s':trigger,
            'start_connector_length_m':float(np.linalg.norm(np.subtract(start,grid_start))),
            'first_raw_path_points_xy':report['replanning_history'][0]['points_xy'][:3],
            'odom_native_source_sim_time':odom,'twist_sim_receipt_time':commands,
            'latest_native_odom_before_trigger':newest,
            'native_odom_window_means_before_trigger':averages,
            'limitations':['Ordinary excludes all samples at/after the predictive stop trigger, conservatively separating emergency override.',
                          'Twist receipt stamps cannot establish exact publisher timing or command/physics latency.',
                          'Raw native derivatives include measurement/contact noise; no certified acceleration or braking floor.',
                          'GT source stamp is zero; request/receipt simulation interval remains required.',
                          'Wall/simulation period mismatch is measured, not repaired by use_sim_time or nominal model_dt.']}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motion-report',type=Path,required=True)
    parser.add_argument('--report',type=Path,required=True)
    args=parser.parse_args()
    raw=args.motion_report.read_bytes(); result=review(json.loads(raw))
    result['motion_sha256']=hashlib.sha256(raw).hexdigest()
    args.report.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(result,indent=2,ensure_ascii=False))

if __name__=='__main__':main()
