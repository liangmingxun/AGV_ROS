#!/usr/bin/env python3
"""Independent yaw-effectiveness exploration with native tracker errors."""
import argparse
import csv
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
IDENTITY = 'exp2c_v5b_yaw_effectiveness_hold_exploration'
MARKERS = ['EFFECT_EXPLORATION_ONLY', 'NOT_FORMAL_PAPER_EVIDENCE']
CANDIDATES = {'S1_gamma0p20_down0p6_hold2p5_up0p6_R10': (.20,3.7), 'S1R12_gamma0p20_down0p6_hold2p5_up0p6_R12': (.20,3.7)}
FLOORS = dict(V4.FLOORS, lateral=.002, longitudinal=.002)
QUALITY_POLICY = 'effect_exploration_continue_on_quality_exceedance'


def hold_gamma(t):
    if t<0. or t>=3.7: return 1.
    if t<.6: u=t/.6; return 1.-.8*(u*u*u*(10.-15.*u+6.*u*u))
    if t<3.1: return .2
    u=(t-3.1)/.6; return .2+.8*(u*u*u*(10.-15.*u+6.*u*u))


def save(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False)+'\n')


def prepare(root):
    config = Path(__file__).resolve().parents[2]/'multi_agv_bringup/config'
    (root/'configs').mkdir(parents=True, exist_ok=True)
    save(root/'EXPLORATION_SCOPE.json', {'experiment_id': IDENTITY, 'markers': MARKERS,
         'hardware_authorization': False, QUALITY_POLICY: True,
         'rigid_fit_quality_threshold_m': .010, 'fixed_task_timeout_seconds': 95,
         'limiter_policy': 'EXECUTION_LIMIT_ACTIVE: observe, retain actual plant limiting',
         'candidate_order': CANDIDATES, 'baseline_reused': False,
         'success_rule': 'active mechanism, resource benefit, >=10% primary geometry IAE improvement, other primary geometry same direction, no sustained limiter dependence'})
    for candidate, (amplitude, duration) in CANDIDATES.items():
        for method in ('M1', 'M1b'):
            upper = yaml.safe_load((config/f'exp2c_v4_{method}_candidate_B.yaml').read_text())
            upper['formal_upper']['configuration_status'] = '_'.join(MARKERS)
            runtime = upper['formal_fake_runtime']
            runtime['experiment_id'] = IDENTITY
            runtime['evidence_scope'] = MARKERS
            runtime[QUALITY_POLICY] = True
            runtime['transient_yaw_v4'] = {'enabled': False}
            runtime['yaw_effectiveness_v5'] = {'enabled': False}
            runtime['yaw_effectiveness_hold_v5b'] = {'enabled': True, 'trigger_progress': 2., 'gamma_hold': amplitude, 'duration': duration, 'ramp_down': .6, 'hold': 2.5, 'ramp_up': .6}
            if candidate.startswith('S1R12') and method=='M1':
                upper['formal_upper']['agents']['risk_gain_upper']=[.12]*3
            target = root/'configs'/f'{candidate}_{method}.yaml'
            target.write_text(yaml.safe_dump(upper, sort_keys=False))
            auth = yaml.safe_load((config/f'formal_exp2c_v4_{method}_B_authorization.yaml').read_text())
            auth['authorization_scope']['upper_config'] = str(target)
            auth['evidence_scope'] = MARKERS
            auth['yaw_effectiveness_hold_v5b'] = {'hardware_execution_authorized': False}
            (root/'configs'/f'{candidate}_{method}_authorization.yaml').write_text(yaml.safe_dump(auth, sort_keys=False))
    localization = yaml.safe_load((config/'localization_odom_exp2c_fake.yaml').read_text())
    localization.update({QUALITY_POLICY: True, 'experiment_id': IDENTITY,
                         'platform_transport_type': 'fake', 'hardware_execution_authorized': False})
    (root/'configs/localization_v5_quality_observation.yaml').write_text(yaml.safe_dump(localization, sort_keys=False))
    topics = yaml.safe_load((config/'record_topics.yaml').read_text())
    registry = topics['experiment_recording']
    for key in ('topics','required_topics'):
        registry[key].append('/multi_agv/yaw_effectiveness_hold_state')
        registry[key].append('/multi_agv/v5_exploration_quality_state')
    (root/'configs/record_topics_v5.yaml').write_text(yaml.safe_dump(topics,sort_keys=False))


def extras(rows):
    dt = V4.durations(rows)
    margin = [V2.num(row, 'transient_yaw_causal_wheel_margin') for row in rows]
    peaks = [max(abs(V2.num(row, 'transient_yaw_left_raw')),
                 abs(V2.num(row, 'transient_yaw_right_raw'))) for row in rows]
    result = {f'J_risk_0p{tag}': sum(max(0., threshold-value)*t for value,t in zip(margin,dt))
              for tag, threshold in ((5,.5),(7,.7))}
    result['native_aligned_maximum_spacing_seconds']=max(dt)
    result['native_aligned_sample_count']=len(rows)
    for tag, threshold in (('155',.155),('160',.160)):
        result[f'controller_raw_above_0p{tag}_duration_seconds'] = sum(t for v,t in zip(peaks,dt) if v>threshold)
    first = next((r for r,p in zip(rows,peaks) if p>.160),None)
    result['raw_first_above_0p160_relative_seconds'] = first['relative_wall_time'] if first else None
    applied = [abs(V2.num(row,f'agv2_wheel_{side}_applied')) for row in rows for side in ('left','right')]
    if not all(math.isfinite(v) for v in margin+peaks+applied):
        raise ValueError('nonfinite exploration execution data')
    result['applied_wheel_peak_mps'] = max(applied)
    reduction = [V2.num(row, 'candidate_common_velocity')-V2.num(row, 'common_velocity_reference') for row in rows]
    result['actual_reference_reduction_mean_mps'] = V4.weighted_mean(reduction,dt)
    hold = maximum = 0.
    limiter_rows=[]
    for row,t in zip(rows,dt):
        limited = any(V2.flag(row,f'agv{i}_wheel_{side}_speed_limit_active')
                      for i in (1,2,3) for side in ('left','right'))
        hold = hold+t if limited else 0.
        maximum = max(maximum,hold)
        if limited: limiter_rows.append(row)
    result['physical_speed_limiter_max_continuous_seconds'] = maximum
    result['physical_speed_limiter_samples']=len(limiter_rows)
    result['physical_speed_limiter_first_relative_seconds']=limiter_rows[0]['relative_wall_time'] if limiter_rows else None
    result['physical_speed_limiter_first_input_output']={
        f'agv{i}_{side}': {'input_degraded_or_raw':V2.num(limiter_rows[0],
             'yaw_effectiveness_hold_'+('left_degraded' if side=='left' else 'right_degraded')) if i==2 else V2.num(limiter_rows[0],f'agv{i}_wheel_{side}_pre_limit'),
             'applied':V2.num(limiter_rows[0],f'agv{i}_wheel_{side}_applied')}
        for i in (1,2,3) for side in ('left','right')} if limiter_rows else None
    for key in ('lateral','longitudinal'):
        values=[abs(row['errors'][key]) for row in rows]
        result[key+'_error_peak']=max(values)
        result[key+'_error_IAE']=sum(value*t for value,t in zip(values,dt))
        result[key+'_error_rms']=math.sqrt(V4.weighted_mean([v*v for v in values],dt))
    return result


def exposure(rows, values, threshold=None):
    dt=V4.durations(rows)
    signed_values=list(values)
    values=[abs(v) for v in values]
    if not all(math.isfinite(v) for v in values): raise ValueError('NUMERICAL_INVALID: nonfinite quality data')
    first=next((i for i,v in enumerate(values) if threshold is not None and v>threshold),None)
    return {'peak':max(values), 'signed_minimum':min(signed_values),
            'signed_maximum':max(signed_values), 'signed_mean':V4.weighted_mean(signed_values,dt),
            'RMS':math.sqrt(V4.weighted_mean([v*v for v in values],dt)),
            'IAE':sum(v*t for v,t in zip(values,dt)), 'existing_quality_threshold':threshold,
            'quality_exceeded':any(v>threshold for v in values) if threshold is not None else None,
            'duration_above_threshold_seconds':sum(t for v,t in zip(values,dt) if v>threshold) if threshold is not None else None,
            'first_exceedance_relative_wall_seconds':rows[first]['relative_wall_time'] if first is not None else None,
            'first_exceedance_value':values[first] if first is not None else None}


def quality_metrics(rows):
    result={key:exposure(rows,[r['errors'][key] for r in rows],.010 if key=='rigid_fit' else None)
            for key in ('rigid_fit','support','pairwise_side','progress')}
    for i in (1,2,3):
        for key in ('lateral','heading','longitudinal'):
            result[f'agv{i}_{key}']=exposure(rows,[V2.num(r,f'agv{i}_hold_tracker_{key}_error') for r in rows])
        result[f'agv{i}_progress']=exposure(rows,[V2.num(r,f'agv{i}_s_tracking_actual')-V2.num(r,'load_s_reference') for r in rows])
    for name,i,j in (('d12',1,2),('d13',1,3),('d23',2,3)):
        def side(r,kind):
            return math.dist([V2.num(r,f'agv{i}_{kind}_x'),V2.num(r,f'agv{i}_{kind}_y')],
                             [V2.num(r,f'agv{j}_{kind}_x'),V2.num(r,f'agv{j}_{kind}_y')])
        result[name]=exposure(rows,[side(r,'support_pose')-side(r,'support_reference') for r in rows])
    return result


def measured_rigid_quality(run, trigger, task_rows, total):
    """Use the estimator's actual gated fit, not an inferred error proxy."""
    path=run/'converted/v5_exploration_quality_state.csv'
    if not path.exists(): raise ValueError('STATE_CHAIN_INVALID: missing v5 quality diagnostics')
    samples=[];seen=set()
    with path.open(newline='') as stream:
        for r in csv.DictReader(stream):
            v=json.loads(r['data_json'])
            if len(v)!=5 or not all(math.isfinite(x) for x in v):
                raise ValueError('NUMERICAL_INVALID: quality diagnostic')
            if v[0] in seen: continue
            seen.add(v[0]);relative=v[1]-trigger
            if not task_rows[0]['relative_wall_time']<=relative<=task_rows[-1]['relative_wall_time']: continue
            if abs(v[3]-.010)>1e-12 or bool(v[4])!=(v[2]>.010):
                raise ValueError('incorrect unchanged quality threshold/flag')
            samples.append({'relative_wall_time':relative,'value':v[2],'source_stamp':v[0]})
    phases={'full_task':samples,'pulse':[r for r in samples if 0<=r['relative_wall_time']<total],
            'recovery':[r for r in samples if total<=r['relative_wall_time']<total+15],
            'post_trigger':[r for r in samples if 0<=r['relative_wall_time']<total+15]}
    result={name:exposure(rs,[r['value'] for r in rs],.010) for name,rs in phases.items() if rs}
    for name,stats in result.items():
        first=next((r for r in phases[name] if r['value']>.010),None)
        stats['first_exceedance_ros_stamp']=first['source_stamp'] if first else None
        context=next((r for r in reversed(task_rows) if first and r['relative_wall_time']<=first['relative_wall_time']),None)
        stats['first_exceedance_context']={k:V2.num(context,k) for k in (
            'yaw_effectiveness_hold_gamma','yaw_effectiveness_hold_left_raw','yaw_effectiveness_hold_right_raw',
            'common_velocity_reference','agv2_hold_tracker_lateral_error','agv2_hold_tracker_heading_error',
            'agv2_robust_margin','yaw_effectiveness_hold_causal_wheel_margin')} if context else None
    return result


def analyze_or_failure(run, candidate, log):
    """Even corrupted/missing state evidence produces an explicit failed report."""
    try: return analyze(run,candidate,log)
    except Exception as exc:
        text=str(exc)
        category='NUMERICAL_INVALID' if any(s in text.lower() for s in ('nonfinite','nan','inf','arithmetic')) else 'STATE_CHAIN_INVALID'
        manifest=yaml.safe_load((run/'manifest.yaml').read_text()) if (run/'manifest.yaml').exists() else {}
        validation=json.loads((run/'validation.json').read_text()) if (run/'validation.json').exists() else {}
        return {'experiment_id':IDENTITY,'markers':MARKERS,'candidate':candidate,
                'method_id':manifest.get('method_id',run.name),'run_dir':str(run),
                'exploration_run_usable':False,'task_complete':bool(validation.get('task_completion',{}).get('complete')),
                'exploration_hard_stop_reasons':[category+': '+text], 'phases':{},
                'partial_observation':True,'formal_evidence':False,'hardware_authorization':False}


def whole_recorded_execution(all_rows, diagnostic_rows):
    """Startup/full execution comes from standard telemetry, not v5-topic coverage."""
    samples=[]
    for row in all_rows:
        raw=[V2.num(row,f'agv{i}_wheel_{side}_pre_limit') for i in (1,2,3) for side in ('left','right')]
        if not all(math.isfinite(v) for v in raw): continue
        r=dict(row);r['relative_wall_time']=V2.num(row,'stamp');r['raw']=raw
        samples.append(r)
    dt=V4.durations(samples)
    limited=[r for r in samples if any(V2.flag(r,f'agv{i}_wheel_{s}_speed_limit_active') for i in (1,2,3) for s in ('left','right'))]
    trigger=next(V2.num(r,'yaw_effectiveness_hold_trigger_wall_time') for r in diagnostic_rows if V2.flag(r,'yaw_effectiveness_hold_triggered'))
    result={'raw_samples':len(samples),'missing_raw_samples':len(all_rows)-len(samples),
            'missing_raw_while_algorithm_valid_samples':sum(V2.flag(r,'algorithm_valid') and any(not math.isfinite(V2.num(r,f'agv{i}_wheel_{s}_pre_limit')) for i in (1,2,3) for s in ('left','right')) for r in all_rows),
            'controller_all_car_raw_peak_mps':max(abs(v) for r in samples for v in r['raw']),
            'limiter_first_ros_stamp':V2.num(limited[0],'stamp') if limited else None,
            'limiter_all_before_disturbance_trigger':all(V2.num(r,'stamp')<trigger for r in limited),
            'limiter_samples':sum(any(V2.flag(r,f'agv{i}_wheel_{s}_speed_limit_active') for i in (1,2,3) for s in ('left','right')) for r in samples),
            'limiter_duration_seconds':sum(t for r,t in zip(samples,dt) if any(V2.flag(r,f'agv{i}_wheel_{s}_speed_limit_active') for i in (1,2,3) for s in ('left','right'))),
            'v5_native_diagnostic_start_delay_from_recording_seconds':V2.num(diagnostic_rows[0],'yaw_effectiveness_hold_source_stamp')-V2.num(all_rows[0],'stamp'),
            'native_full_task_scope':'valid control samples with available v5 native diagnostics; startup coverage may be delayed; raw/limiter full-recording metrics above are independent'}
    for i in (1,2,3):
        peaks=[max(abs(V2.num(r,f'agv{i}_wheel_{s}_pre_limit')) for s in ('left','right')) for r in samples]
        result[f'agv{i}_raw_peak_mps']=max(peaks)
        for tag,threshold in (('150',.150),('155',.155),('160',.160)):
            result[f'agv{i}_raw_above_0p{tag}_duration_seconds']=sum(t for p,t in zip(peaks,dt) if p>threshold)
    return result


def analyze(run, candidate, log_path, historical=False):
    manifest = yaml.safe_load((run/'manifest.yaml').read_text())
    params = yaml.safe_load((run/'rosparams.yaml').read_text())
    runtime = params['formal_fake_algorithm']['formal_fake_runtime']
    if historical: raise ValueError('v5 requires fresh M1b and M1; no reused baseline')
    expected = IDENTITY
    if manifest['experiment_id'] != expected or runtime['experiment_id'] != expected:
        raise ValueError('wrong experiment provenance')
    if not historical and runtime['yaw_effectiveness_hold_v5b'].get('enabled') is not True:
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
    pulse_config = runtime['yaw_effectiveness_hold_v5b']
    for key,expected in (('ramp_down',.6),('hold',2.5),('ramp_up',.6)):
        if abs(pulse_config[key]-expected)>1e-12:
            raise ValueError('incorrect fixed v5b profile segment')
    expected_gain=.12 if candidate.startswith('S1R12') and params['formal_fake_algorithm']['formal_upper']['mode']=='M1' else .10
    if any(abs(gain-expected_gain)>1e-12 for gain in params['formal_fake_algorithm']['formal_upper']['agents']['risk_gain_upper']):
        raise ValueError('unexpected v5b upper risk gain')
    if abs(pulse_config['gamma_hold']-amplitude)>1e-12 or abs(pulse_config['duration']-total)>1e-12 or pulse_config['trigger_progress'] != 2.:
        raise ValueError('incorrect exploratory pulse')
    with (run/'converted/aligned_samples.csv').open(newline='') as stream:
        all_rows = list(csv.DictReader(stream))
    rows=[]; seen=set()
    for row in all_rows:
        if not V2.flag(row,'algorithm_valid') or not V2.flag(row,'yaw_effectiveness_hold_state_available'): continue
        row = dict(row)
        for key,value in list(row.items()):
            if key.startswith('yaw_effectiveness_hold_'): row[key.replace('yaw_effectiveness_hold_','transient_yaw_',1)] = value
        row['transient_yaw_left_disturbed']=row['yaw_effectiveness_hold_left_degraded']
        row['transient_yaw_right_disturbed']=row['yaw_effectiveness_hold_right_degraded']
        wall=V2.num(row,'yaw_effectiveness_hold_wall_time')
        if not math.isfinite(wall) or wall in seen: continue
        seen.add(wall); rows.append(row)
    triggers = {V2.num(row,'transient_yaw_trigger_wall_time') for row in rows if V2.flag(row,'transient_yaw_triggered')}
    if len(triggers)!=1 or not all(math.isfinite(v) for v in triggers):
        raise ValueError('missing/multiple pulse triggers')
    trigger=next(iter(triggers))
    for row in rows:
        row['relative_wall_time']=V2.num(row,'transient_yaw_wall_time')-trigger
        row['errors']=V4.error_signals(row)
        for key,field in (('heading','heading_error'),('lateral','lateral_error'),('longitudinal','longitudinal_error')):
            row['errors'][key]=V2.num(row,'agv2_hold_tracker_'+field)
        if not all(math.isfinite(v) for v in row['errors'].values()): raise ValueError('nonfinite geometry')
    rows.sort(key=lambda row: row['relative_wall_time'])
    selected={'baseline':[r for r in rows if -3.<=r['relative_wall_time']<0.],
              'pulse':[r for r in rows if 0.<=r['relative_wall_time']<total],
              'recovery':[r for r in rows if total<=r['relative_wall_time']<total+15.]}
    selected['post_trigger']=selected['pulse']+selected['recovery']
    selected['full_task']=rows
    if len(selected['baseline'])<200 or not selected['pulse'] or not selected['recovery']:
        log=log_path.read_text(errors='replace').lower()
        hard=[term for term in ('safety abort latched','held fail-zero:','hard gate rejected') if term in log]
        validation=json.loads((run/'validation.json').read_text())
        if not validation.get('task_completion',{}).get('complete'):
            hard.append('STATE_CHAIN_INVALID: task incomplete after fatal stop' if hard else 'TASK_TIMEOUT: task incomplete at fixed deadline')
        if not hard: raise ValueError('insufficient exploratory observation')
        phases={name:dict(V4.phase_metrics(rs),**extras(rs)) for name,rs in selected.items() if rs}
        return {'experiment_id':IDENTITY,'markers':MARKERS,'candidate':candidate,
                'run_dir':str(run),'method_id':manifest['method_id'],
                'gamma_min':amplitude,'duration_seconds':total,'cal_used':False,
                'hardware_authorization':False,'historical_baseline_reused':False,
                'exploration_run_usable':False,'exploration_hard_stop_reasons':hard,
                'task_complete':bool(validation.get('task_completion',{}).get('complete')),'task_completion_time_seconds':None,
                'partial_observation':True,'recovery_horizon_complete':False,
                'observed_post_trigger_seconds':selected['post_trigger'][-1]['relative_wall_time'] if selected['post_trigger'] else None,
                'phases':phases,'recovery_times_seconds':{key:None for key in FLOORS},
                'heading_definition':'native PlanarTrackingResult.heading_error',
                'startup_and_full_run_limiter':V4.limiter_locations(rows),
                'reported_limits_normal':all(abs(V2.num(r,f'agv{i}_wheel_{side}_reported_limit')-.16)<1e-12
                    for r in rows for i in (1,2,3) for side in ('left','right'))}
    phases={name:dict(V4.phase_metrics(rs),**extras(rs)) for name,rs in selected.items()}
    bounds={key:V4.envelope([r['errors'][key] for r in selected['baseline']],floor) for key,floor in FLOORS.items()}
    recovery={key:V4.recovery_time(selected['recovery'],key,bounds[key],total) for key in bounds}
    log=log_path.read_text(errors='replace')
    # Ignore arming/terminal validity transitions, never motion-time fail-zero.
    motion_log=log.split('Formal execution initialized',1)[-1].split('Formal execution reached',1)[0]
    validation=json.loads((run/'validation.json').read_text())
    summary=json.loads((run/'summary_metrics.json').read_text()) if (run/'summary_metrics.json').exists() else {}
    hard=[]
    for term,category in (('safety abort latched','NUMERICAL_INVALID'),
                          ('held fail-zero:','STATE_CHAIN_INVALID'),
                          ('hard gate rejected','STRUCTURAL_GEOMETRY_INVALID'),
                          ('numerical_invalid','NUMERICAL_INVALID'),
                          ('state_chain_invalid','STATE_CHAIN_INVALID'),
                          ('structural_geometry_invalid','STRUCTURAL_GEOMETRY_INVALID')):
        if term in motion_log.lower(): hard.append(category+': '+term)
    if not validation.get('task_completion',{}).get('complete'):
        hard.append('STATE_CHAIN_INVALID: task incomplete after fatal stop' if hard else 'TASK_TIMEOUT: task incomplete at fixed deadline')
    if selected['recovery'][-1]['relative_wall_time'] < total+15.-.03: hard.append('fake recovery horizon incomplete')
    waveform=all(abs(V2.num(r,'yaw_effectiveness_hold_gamma')-(1.-(1.-amplitude)*V2.num(r,'yaw_effectiveness_hold_envelope')))<1e-12
                 and amplitude-1e-12<=V2.num(r,'yaw_effectiveness_hold_gamma')<=1.+1e-12
                 and abs(V2.num(r,'yaw_effectiveness_hold_mean_longitudinal_delta'))<1e-12
                 and (r['relative_wall_time']<total or V2.num(r,'yaw_effectiveness_hold_gamma')==1.) for r in rows)
    waveform = waveform and all(abs(V2.num(r,'yaw_effectiveness_hold_gamma')-hold_gamma(r['relative_wall_time']))<1e-12 for r in rows)
    if not waveform: hard.append('invalid fake pulse waveform')
    return {'experiment_id':IDENTITY,'markers':MARKERS,'candidate':candidate,'run_dir':str(run),
            'historical_baseline_reused':historical,'method_id':manifest['method_id'],
            'hardware_authorization':False,'exploration_hard_stop_reasons':hard,
            'exploration_run_usable':not hard,'generic_validation_valid':validation.get('valid'),
            'task_complete':bool(validation.get('task_completion',{}).get('complete')),
            'quality_policy':runtime.get(QUALITY_POLICY,False),
            'whole_recorded_execution':whole_recorded_execution(all_rows,rows),
            'quality_exposure':{name:quality_metrics(rs) for name,rs in selected.items()},
            'measured_rigid_quality_exposure':measured_rigid_quality(run,trigger,rows,total) if runtime.get(QUALITY_POLICY,False) else None,
            'diagnostic_classification':{'QUALITY_THRESHOLD_EXCEEDED':'observation only',
                'EXECUTION_LIMIT_ACTIVE':'observation only; limiter remains applied',
                'NUMERICAL_INVALID':'fatal','STATE_CHAIN_INVALID':'fatal','STRUCTURAL_GEOMETRY_INVALID':'fatal'},
            'task_completion_time_seconds':summary.get('task',{}).get('completion_time'),
            'phases':phases,'recovery_times_seconds':recovery,'recovery_envelopes':bounds,
            'startup_and_full_run_limiter':V4.limiter_locations(rows),
            'heading_definition':'native PlanarTrackingResult.heading_error', 'gamma_min': amplitude, 'duration_seconds': total,
            'cal_used': False, 'reported_limits_normal': all(abs(V2.num(r,f'agv{i}_wheel_{side}_reported_limit')-.16)<1e-12 for r in rows for i in (1,2,3) for side in ('left','right'))}


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
    geometry = {key: (b[key]-a[key])/b[key] if b[key]>1e-12 else 0. for key in ('lateral_error_IAE','heading_error_IAE','support_error_IAE','rigid_fit_error_IAE')}
    # Operational exploration convention for "not long-term dependent":
    # assess the fixed disturbance window, not startup or the combined recovery.
    # Preserve every limiter event in the output; this is not a control gate.
    limiter_independent=all(data['phases']['pulse']['physical_speed_limiter_max_continuous_seconds']<.5
                           for data in (m1,m1b))
    found=usable and all(mechanism.values()) and any(clear.values()) and any(value>=.10 for value in geometry.values()) and all(value>=0. for value in geometry.values()) and limiter_independent
    return {'experiment_id':IDENTITY,'markers':MARKERS,'candidate':m1['candidate'],
            'status':('S1R12_GEOMETRY_WORKING_POINT_FOUND' if m1['candidate'].startswith('S1R12') else 'S1_GEOMETRY_WORKING_POINT_FOUND') if found else 'EFFECT_INSUFFICIENT' if usable else 'CANDIDATE_HARD_STOP',
            'methods':{'M1':m1,'M1b':m1b},'mechanism_checks':mechanism,
            'resource_improvement_M1b_minus_M1':benefit,'clear_resource_improvement':clear, 'geometry_IAE_relative_improvement':geometry,
            'numeric_resolution_floors':floors,'not_formal_pair':True,
            'limiter_independence_convention':'disturbance-window maximum continuous limiter <0.5 s; observational selection only, not a safety gate',
            'limiter_independence_check':limiter_independent,
            'baseline_reused':m1b['historical_baseline_reused'],
            'task_time_increase_seconds':m1['task_completion_time_seconds']-m1b['task_completion_time_seconds']}


def write_hard_stopped_candidate(run,value):
    out=run.parent
    other='M1' if value['method_id']=='M1b_R1' else 'M1b'
    other_exists=(out/(other+'_R1')).exists()
    report={'experiment_id':IDENTITY,'markers':MARKERS,'candidate':value['candidate'],
            'status':'CANDIDATE_HARD_STOP','methods':{value['method_id']:value},
            'not_run':{} if other_exists else {other:'candidate stopped under requested hard-stop rule'},
            'existing_other_run_not_used_for_success':other if other_exists else None,
            'paired_geometry_or_risk_advantage':None,'formal_evidence':False}
    save(out/'effect_comparison.json',report)
    lines=['# '+' / '.join(MARKERS),'','CANDIDATE_HARD_STOP','',
           'Only partial observations are available; no completed comparison or recovery is inferred.',
           '','Hard-stop reasons: '+', '.join(value['exploration_hard_stop_reasons']),
           '',other+(' exists but no successful paired conclusion is inferred.' if other_exists else ' was not run.')+' No stronger gamma candidate was run.','',
           '| partial phase / metric | '+value['method_id']+' |','| --- | ---: |']
    with (out/'effect_metrics.csv').open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['evidence_scope','observation_status','phase','metric',value['method_id']])
        for phase,metrics in value['phases'].items():
            for key,result in metrics.items():
                writer.writerow([' / '.join(MARKERS),'PARTIAL_HARD_STOPPED',phase,key,result])
                lines.append(f'| {phase}/{key} | {result} |')
    (out/'effect_comparison.md').write_text('\n'.join(lines)+'\n')


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--prepare',type=Path); p.add_argument('--root',type=Path)
    p.add_argument('--candidate',choices=tuple(CANDIDATES))
    p.add_argument('--check-run',type=Path); p.add_argument('--check-log',type=Path)
    args=p.parse_args()
    if args.prepare: prepare(args.prepare.resolve()); return 0
    if args.check_run:
        if not args.candidate or not args.check_log: p.error('--candidate and --check-log required')
        value=analyze_or_failure(args.check_run.resolve(),args.candidate,args.check_log.resolve())
        save(args.check_run/'EFFECT_EXPLORATION_ONLY_NOT_FORMAL_PAPER_EVIDENCE.json',value)
        if not value['exploration_run_usable']: write_hard_stopped_candidate(args.check_run,value)
        print('EXPLORATION_RUN_USABLE' if value['exploration_run_usable'] else 'CANDIDATE_HARD_STOP')
        return 0 if value['exploration_run_usable'] else 20
    if not args.root or not args.candidate: p.error('--root and --candidate required')
    root=args.root.resolve(); c=args.candidate
    m1=analyze_or_failure(root/c/'M1_R1',c,root/'logs'/c/'M1_R1/algorithm.log')
    baseline=root/c/'M1b_R1'
    log=root/'logs'/c/'M1b_R1/algorithm.log'
    m1b=analyze_or_failure(baseline,c,log)
    if not m1['exploration_run_usable'] or not m1b['exploration_run_usable']:
        value=m1 if not m1['exploration_run_usable'] else m1b
        write_hard_stopped_candidate(Path(value['run_dir']),value)
        return 20
    result=comparison(m1,m1b)
    out=root/c
    save(out/'effect_comparison.json',result)
    with (out/'quality_exposure.csv').open('w',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['evidence_scope','method','phase','metric','statistic','value'])
        for method,data in (('M1b',m1b),('M1',m1)):
            for phase,metrics in data['quality_exposure'].items():
                for metric,stats in metrics.items():
                    for key,value in stats.items():
                        writer.writerow([' / '.join(MARKERS),method,phase,metric,key,value])
            for phase,stats in data['measured_rigid_quality_exposure'].items():
                for key,value in stats.items():
                    writer.writerow([' / '.join(MARKERS),method,phase,'estimator_rigid_fit',key,value])
    lines=['# '+' / '.join(MARKERS),'',f"{c}: {result['status']}",'',
           'Fresh M1b/M1 yaw-effectiveness exploration; native tracker errors, no CAL, no formal screening changes.','',
           '| phase / metric | M1b | M1 |','| --- | ---: | ---: |']
    with (out/'effect_metrics.csv').open('w',newline='') as stream:
        w=csv.writer(stream); w.writerow(['evidence_scope','phase','metric','M1b_R1','M1_R1'])
        for phase in m1['phases']:
            for key in m1['phases'][phase]:
                w.writerow([' / '.join(MARKERS),phase,key,m1b['phases'][phase][key],m1['phases'][phase][key]])
                lines.append(f"| {phase}/{key} | {m1b['phases'][phase][key]} | {m1['phases'][phase][key]} |")
        for key in m1['recovery_times_seconds']:
            w.writerow([' / '.join(MARKERS),'recovery_time',key,m1b['recovery_times_seconds'][key],m1['recovery_times_seconds'][key]])
            lines.append(f"| {key} recovery time / s | {m1b['recovery_times_seconds'][key]} | {m1['recovery_times_seconds'][key]} |")
    (out/'effect_comparison.md').write_text('\n'.join(lines)+'\n')
    print(result['status'])
    return 0 if result['status'].endswith('GEOMETRY_WORKING_POINT_FOUND') else 10 if result['status']=='EFFECT_INSUFFICIENT' else 20


if __name__=='__main__':
    try: sys.exit(main())
    except Exception as exc:
        print('EXPLORATION_HARD_STOP: '+str(exc),file=sys.stderr); sys.exit(20)
