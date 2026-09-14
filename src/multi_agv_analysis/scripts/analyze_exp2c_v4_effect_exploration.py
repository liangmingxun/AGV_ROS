#!/usr/bin/env python3
"""Independent exploratory analysis. Never selects or passes formal screening."""
import argparse
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import yaml

SPEC = importlib.util.spec_from_file_location('v4', Path(__file__).with_name('analyze_exp2c_v4_transient_yaw.py'))
V4 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V4)
V2 = V4.V2
IDENTITY = 'exp2c_v4_effect_exploration'
MARKERS = ['EFFECT_EXPLORATION_ONLY', 'NOT_FORMAL_PAPER_EVIDENCE']
CANDIDATES = {'E0': (.022, 2.), 'E1': (.022, 2.5), 'E2': (.022, 3.),
              'E3': (.024, 2.5), 'E4': (.026, 2.5)}
# Operational hard-stop definition, not a success or formal screening threshold.
LONG_LIMITER_SECONDS = .5


def save(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False)+'\n')


def prepare(root):
    config = Path(__file__).resolve().parents[2]/'multi_agv_bringup/config'
    (root/'configs').mkdir(parents=True, exist_ok=True)
    save(root/'EXPLORATION_SCOPE.json', {'experiment_id': IDENTITY, 'markers': MARKERS,
         'hardware_authorization': False, 'long_limiter_continuous_seconds': LONG_LIMITER_SECONDS,
         'candidate_order': CANDIDATES, 'E0_baseline_reused': True,
         'success_rule': 'active risk contraction and real reference reduction, plus >=1 measurable resource benefit'})
    for candidate, (amplitude, duration) in CANDIDATES.items():
        for method in ('M1', 'M1b'):
            upper = yaml.safe_load((config/f'exp2c_v4_{method}_candidate_B.yaml').read_text())
            upper['formal_upper']['configuration_status'] = '_'.join(MARKERS)
            runtime = upper['formal_fake_runtime']
            runtime['experiment_id'] = IDENTITY
            runtime['evidence_scope'] = MARKERS
            runtime['transient_yaw_v4'].update(effect_exploration=True, amplitude=amplitude, duration=duration)
            target = root/'configs'/f'{candidate}_{method}.yaml'
            target.write_text(yaml.safe_dump(upper, sort_keys=False))
            auth = yaml.safe_load((config/f'formal_exp2c_v4_{method}_B_authorization.yaml').read_text())
            auth['authorization_scope']['upper_config'] = str(target)
            auth['evidence_scope'] = MARKERS
            (root/'configs'/f'{candidate}_{method}_authorization.yaml').write_text(yaml.safe_dump(auth, sort_keys=False))


def extras(rows):
    dt = V4.durations(rows)
    margin = [V2.num(row, 'transient_yaw_causal_wheel_margin') for row in rows]
    peaks = [max(abs(V2.num(row, 'transient_yaw_left_raw')),
                 abs(V2.num(row, 'transient_yaw_right_raw'))) for row in rows]
    result = {f'J_risk_0p{tag}': sum(max(0., threshold-value)*t for value,t in zip(margin,dt))
              for tag, threshold in ((5,.5),(7,.7))}
    for tag, threshold in (('155',.155),('160',.160)):
        result[f'controller_raw_above_0p{tag}_duration_seconds'] = sum(t for v,t in zip(peaks,dt) if v>threshold)
    applied = [abs(V2.num(row,f'agv2_wheel_{side}_applied')) for row in rows for side in ('left','right')]
    if not all(math.isfinite(v) for v in margin+peaks+applied):
        raise ValueError('nonfinite exploration execution data')
    result['applied_wheel_peak_mps'] = max(applied)
    reduction = [V2.num(row, 'candidate_common_velocity')-V2.num(row, 'common_velocity_reference') for row in rows]
    result['actual_reference_reduction_mean_mps'] = V4.weighted_mean(reduction,dt)
    hold = maximum = 0.
    for row,t in zip(rows,dt):
        limited = any(V2.flag(row,f'agv{i}_wheel_{side}_speed_limit_active')
                      for i in (1,2,3) for side in ('left','right'))
        hold = hold+t if limited else 0.
        maximum = max(maximum,hold)
    result['physical_speed_limiter_max_continuous_seconds'] = maximum
    return result


def analyze(run, candidate, log_path, historical=False):
    manifest = yaml.safe_load((run/'manifest.yaml').read_text())
    params = yaml.safe_load((run/'rosparams.yaml').read_text())
    runtime = params['formal_fake_algorithm']['formal_fake_runtime']
    expected = V4.IDENTITY if historical else IDENTITY
    if manifest['experiment_id'] != expected or runtime['experiment_id'] != expected:
        raise ValueError('wrong experiment provenance')
    if historical and (candidate != 'E0' or manifest['method_id'] != 'M1b_R1'):
        raise ValueError('only existing M1b-B may be reused for E0 exploration')
    if not historical and runtime['transient_yaw_v4'].get('effect_exploration') is not True:
        raise ValueError('missing explicit exploration opt-in')
    if abs(manifest['metrics']['nominal_common_velocity']-.1)>1e-12 or abs(runtime['leader']['velocity']-.1)>1e-12:
        raise ValueError('incorrect task metadata')
    for i in (1,2,3):
        if params[f'agv{i}']['chassis_controller']['transport_type'] != 'fake':
            raise ValueError('serial is forbidden')
    def audit(value):
        if isinstance(value,dict):
            for key,item in value.items():
                if key.endswith('hardware_execution_authorized') and item is not False:
                    raise ValueError('hardware grant is forbidden')
                audit(item)
        elif isinstance(value,list):
            for item in value: audit(item)
    audit(params)
    amplitude,total = CANDIDATES[candidate]
    pulse_config = runtime['transient_yaw_v4']
    if abs(pulse_config['amplitude']-amplitude)>1e-12 or abs(pulse_config['duration']-total)>1e-12 or pulse_config['trigger_progress'] != 2.:
        raise ValueError('incorrect exploratory pulse')
    with (run/'converted/aligned_samples.csv').open(newline='') as stream:
        all_rows = list(csv.DictReader(stream))
    rows=[]; seen=set()
    for row in all_rows:
        if not V2.flag(row,'algorithm_valid') or not V2.flag(row,'transient_yaw_state_available'): continue
        wall=V2.num(row,'transient_yaw_wall_time')
        if not math.isfinite(wall) or wall in seen: continue
        seen.add(wall); rows.append(row)
    triggers = {V2.num(row,'transient_yaw_trigger_wall_time') for row in rows if V2.flag(row,'transient_yaw_triggered')}
    if len(triggers)!=1 or not all(math.isfinite(v) for v in triggers):
        raise ValueError('missing/multiple pulse triggers')
    trigger=next(iter(triggers))
    for row in rows:
        row['relative_wall_time']=V2.num(row,'transient_yaw_wall_time')-trigger
        row['errors']=V4.error_signals(row)
        if not all(math.isfinite(v) for v in row['errors'].values()): raise ValueError('nonfinite geometry')
    rows.sort(key=lambda row: row['relative_wall_time'])
    selected={'baseline':[r for r in rows if -3.<=r['relative_wall_time']<0.],
              'pulse':[r for r in rows if 0.<=r['relative_wall_time']<total],
              'recovery':[r for r in rows if total<=r['relative_wall_time']<total+15.]}
    selected['post_trigger']=selected['pulse']+selected['recovery']
    if len(selected['baseline'])<200 or not selected['pulse'] or not selected['recovery']:
        raise ValueError('insufficient exploratory observation')
    phases={name:dict(V4.phase_metrics(rs),**extras(rs)) for name,rs in selected.items()}
    bounds={key:V4.envelope([r['errors'][key] for r in selected['baseline']],floor) for key,floor in V4.FLOORS.items()}
    recovery={key:V4.recovery_time(selected['recovery'],key,bounds[key],total) for key in bounds}
    log=log_path.read_text(errors='replace')
    # Ignore arming/terminal validity transitions, never motion-time fail-zero.
    motion_log=log.split('Formal execution initialized',1)[-1].split('Formal execution reached',1)[0]
    validation=json.loads((run/'validation.json').read_text())
    summary=json.loads((run/'summary_metrics.json').read_text()) if (run/'summary_metrics.json').exists() else {}
    hard=[]
    for term in ('safety abort latched','held fail-zero:','hard gate rejected'):
        if term in motion_log.lower(): hard.append(term)
    if not validation.get('task_completion',{}).get('complete'): hard.append('task incomplete')
    if selected['recovery'][-1]['relative_wall_time'] < total+15.-.03: hard.append('fake recovery horizon incomplete')
    if phases['post_trigger']['physical_speed_limiter_max_continuous_seconds']>=LONG_LIMITER_SECONDS:
        hard.append('long physical speed limiter dependence')
    waveform=all(V2.num(r,'transient_yaw_disturbance')>=0. and abs(V2.num(r,'transient_yaw_mean_longitudinal_delta'))<1e-12
                 and (r['relative_wall_time']<total or V2.num(r,'transient_yaw_disturbance')==0.) for r in rows)
    if not waveform: hard.append('invalid fake pulse waveform')
    return {'experiment_id':IDENTITY,'markers':MARKERS,'candidate':candidate,'run_dir':str(run),
            'historical_baseline_reused':historical,'method_id':manifest['method_id'],
            'hardware_authorization':False,'exploration_hard_stop_reasons':hard,
            'exploration_run_usable':not hard,'generic_validation_valid':validation.get('valid'),
            'task_completion_time_seconds':summary.get('task',{}).get('completion_time'),
            'phases':phases,'recovery_times_seconds':recovery,'recovery_envelopes':bounds,
            'startup_and_full_run_limiter':V4.limiter_locations(rows),
            'heading_definition':'baseline-relative robot-yaw versus support-tangent proxy'}


def comparison(m1,m1b):
    a=m1['phases']['post_trigger']; b=m1b['phases']['post_trigger']
    baseline=m1['phases']['baseline']
    mechanism={'contraction_positive':a['risk_contraction_peak']>0.,
               'risk_boundary_really_active':a['risk_boundary_active_duration_seconds']>0.,
               'actual_reference_reduction':a['actual_reference_reduction_peak_mps']>max(1e-5,baseline['actual_reference_reduction_peak_mps']),
               'common_reference_really_decreased':a['common_velocity_reference_minimum']<baseline['common_velocity_reference_minimum']-1e-5}
    # Resolution floors prevent rounding jitter being reported as a benefit.
    benefit={key:b[key]-a[key] for key in ('wheel_margin_below_0p5_duration_seconds','J_risk_0p5','J_risk_0p7',
                                         'controller_raw_above_0p150_duration_seconds','controller_raw_wheel_peak_mps')}
    floors={'wheel_margin_below_0p5_duration_seconds':.02,'J_risk_0p5':.001,'J_risk_0p7':.001,
            'controller_raw_above_0p150_duration_seconds':.02,'controller_raw_wheel_peak_mps':.0001}
    clear={key:value>floors[key] for key,value in benefit.items()}
    usable=m1['exploration_run_usable'] and m1b['exploration_run_usable']
    found=usable and all(mechanism.values()) and any(clear.values())
    return {'experiment_id':IDENTITY,'markers':MARKERS,'candidate':m1['candidate'],
            'status':'EFFECT_WORKING_POINT_FOUND' if found else 'EFFECT_INSUFFICIENT' if usable else 'CANDIDATE_HARD_STOP',
            'methods':{'M1':m1,'M1b':m1b},'mechanism_checks':mechanism,
            'resource_improvement_M1b_minus_M1':benefit,'clear_resource_improvement':clear,
            'numeric_resolution_floors':floors,'not_formal_pair':True,
            'baseline_reused':m1b['historical_baseline_reused'],
            'task_time_increase_seconds':m1['task_completion_time_seconds']-m1b['task_completion_time_seconds']}


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--prepare',type=Path); p.add_argument('--root',type=Path)
    p.add_argument('--candidate',choices=tuple(CANDIDATES)); p.add_argument('--baseline',type=Path)
    p.add_argument('--baseline-log',type=Path)
    p.add_argument('--check-run',type=Path); p.add_argument('--check-log',type=Path)
    args=p.parse_args()
    if args.prepare: prepare(args.prepare.resolve()); return 0
    if args.check_run:
        if not args.candidate or not args.check_log: p.error('--candidate and --check-log required')
        value=analyze(args.check_run.resolve(),args.candidate,args.check_log.resolve())
        save(args.check_run/'EFFECT_EXPLORATION_ONLY_NOT_FORMAL_PAPER_EVIDENCE.json',value)
        print('EXPLORATION_RUN_USABLE' if value['exploration_run_usable'] else 'CANDIDATE_HARD_STOP')
        return 0 if value['exploration_run_usable'] else 20
    if not args.root or not args.candidate: p.error('--root and --candidate required')
    root=args.root.resolve(); c=args.candidate
    m1=analyze(root/c/'M1_R1',c,root/'logs'/c/'M1_R1/algorithm.log')
    old=bool(args.baseline)
    baseline=args.baseline.resolve() if old else root/c/'M1b_R1'
    log=args.baseline_log.resolve() if old else root/'logs'/c/'M1b_R1/algorithm.log'
    m1b=analyze(baseline,c,log,historical=old)
    result=comparison(m1,m1b)
    if old:
        result['historical_baseline_aligned_csv_sha256']=hashlib.sha256((baseline/'converted/aligned_samples.csv').read_bytes()).hexdigest()
    save(root/f'{c}_comparison.json',result)
    lines=['# '+' / '.join(MARKERS),'',f"{c}: {result['status']}",'',
           'E0 reuses historical M1b-B only as exploratory reference; no formal selection or screening is changed.','',
           '| phase / metric | M1b | M1 |','| --- | ---: | ---: |']
    with (root/f'{c}_metrics.csv').open('w',newline='') as stream:
        w=csv.writer(stream); w.writerow(['evidence_scope','phase','metric','M1b_R1','M1_R1'])
        for phase in m1['phases']:
            for key in m1['phases'][phase]:
                w.writerow([' / '.join(MARKERS),phase,key,m1b['phases'][phase][key],m1['phases'][phase][key]])
                lines.append(f"| {phase}/{key} | {m1b['phases'][phase][key]} | {m1['phases'][phase][key]} |")
        for key in m1['recovery_times_seconds']:
            w.writerow([' / '.join(MARKERS),'recovery_time',key,m1b['recovery_times_seconds'][key],m1['recovery_times_seconds'][key]])
            lines.append(f"| {key} recovery time / s | {m1b['recovery_times_seconds'][key]} | {m1['recovery_times_seconds'][key]} |")
    (root/f'{c}_comparison.md').write_text('\n'.join(lines)+'\n')
    print(result['status'])
    return 0 if result['status']=='EFFECT_WORKING_POINT_FOUND' else 10 if result['status']=='EFFECT_INSUFFICIENT' else 20


if __name__=='__main__':
    try: sys.exit(main())
    except Exception as exc:
        print('EXPLORATION_HARD_STOP: '+str(exc),file=sys.stderr); sys.exit(20)
