#!/usr/bin/env python3
"""Frozen S1 three-method repeat validation; no parameter search or formal claim."""
import argparse
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import statistics
import subprocess
import yaml
from scipy import stats

SPEC=importlib.util.spec_from_file_location('hold',Path(__file__).with_name('analyze_exp2c_v5b_yaw_hold.py'))
H=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(H)
V4,V2=H.V4,H.V2
ID='exp2c_v5b_s1_three_method_repeat_validation'
MARKERS=['REPEAT_VALIDATION','NOT_FORMAL_PAPER_EVIDENCE']
S1=next(iter(H.CANDIDATES))
METHODS={'M1':'M1_R1','M1b':'M1b_R1','M2b':'M2b_COMPLETE'}
ORDERS=[['M1b','M1','M2b'],['M2b','M1b','M1'],['M1','M2b','M1b'],['M1b','M2b','M1'],['M2b','M1','M1b']]
GEOMETRY=['lateral_error_IAE','heading_error_IAE','support_error_IAE','rigid_fit_error_IAE']
CORE=GEOMETRY+['pairwise_side_error_IAE','progress_error_IAE','J_risk_0p5','J_risk_0p7',
 'wheel_margin_below_0p5_duration_seconds','controller_raw_above_0p160_duration_seconds',
 'physical_speed_limiter_duration_seconds','task_time_seconds']
CONFIG=Path(__file__).resolve().parents[2]/'multi_agv_bringup/config'

def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')

def prepare(root,method):
    # Obtain the frozen S1 config without changing any historical config.
    original_candidates=H.CANDIDATES
    try:
        H.CANDIDATES={S1:original_candidates[S1]}
        H.prepare(root)
    finally:H.CANDIDATES=original_candidates
    for m in ('M1','M1b'):
        path=root/'configs'/f'{S1}_{m}.yaml'
        c=yaml.safe_load(path.read_text());c['formal_fake_runtime']['experiment_id']=ID
        c['formal_fake_runtime']['evidence_scope']=MARKERS
        c['formal_fake_runtime']['repeat_validation_metadata']={'triad':root.parent.name,'method':m,
            'order':json.loads((root.parent/'order.json').read_text()) if (root.parent/'order.json').exists() else ['integration_smoke']}
        c['formal_upper']['configuration_status']='_'.join(MARKERS)
        path.write_text(yaml.safe_dump(c,sort_keys=False))
        auth_path=root/'configs'/f'{S1}_{m}_authorization.yaml'
        auth=yaml.safe_load(auth_path.read_text());auth['evidence_scope']=MARKERS
        auth_path.write_text(yaml.safe_dump(auth,sort_keys=False))
    runtime=yaml.safe_load((CONFIG/'formal_serial_m2b_circle_0p10_observation_runtime.yaml').read_text())
    r=runtime['formal_fake_runtime']
    r.update({'experiment_id':ID,'command_publication_authorized':True,'serial_execution_authorized':False,
        H.QUALITY_POLICY:True,'evidence_scope':MARKERS,
        'yaw_effectiveness_hold_v5b':yaml.safe_load((root/'configs'/f'{S1}_M1.yaml').read_text())['formal_fake_runtime']['yaw_effectiveness_hold_v5b'],
        'repeat_validation_metadata':{'triad':root.parent.name,'method':'M2b_COMPLETE',
         'order':json.loads((root.parent/'order.json').read_text()) if (root.parent/'order.json').exists() else ['integration_smoke']}})
    # Existing M2b control/lower configuration remains byte-for-byte semantic equivalent.
    m2=yaml.safe_load((CONFIG/'exp2b_M2b.yaml').read_text())
    (root/'configs'/f'{S1}_M2b.yaml').write_text(yaml.safe_dump(m2,sort_keys=False))
    (root/'configs/M2b_runtime.yaml').write_text(yaml.safe_dump(runtime,sort_keys=False))
    auth=yaml.safe_load((root/'configs'/f'{S1}_M1_authorization.yaml').read_text())
    auth['authorization_scope']['upper_config']=str(root/'configs'/f'{S1}_M2b.yaml')
    auth['authorization_scope']['runtime_config']=str(root/'configs/M2b_runtime.yaml')
    auth['evidence_scope']=MARKERS
    (root/'configs'/f'{S1}_M2b_authorization.yaml').write_text(yaml.safe_dump(auth,sort_keys=False))
    loc=root/'configs/localization_v5_quality_observation.yaml'
    c=yaml.safe_load(loc.read_text());c['experiment_id']=ID;loc.write_text(yaml.safe_dump(c,sort_keys=False))
    topics=root/'configs/record_topics_v5.yaml';c=yaml.safe_load(topics.read_text())
    if method=='M2b':
        for key in ('topics','required_topics'):
            if '/multi_agv/m2b_algorithm_state' not in c['experiment_recording'][key]:
                c['experiment_recording'][key].append('/multi_agv/m2b_algorithm_state')
    topics.write_text(yaml.safe_dump(c,sort_keys=False))
    save(root/'REPEAT_SCOPE.json',{'experiment_id':ID,'markers':MARKERS,'method':METHODS[method],
        'hardware_authorization':False,'profile':r['yaw_effectiveness_hold_v5b'],'risk_gain_upper':.10,
        'task_timeout_seconds':95,'windows':[-3,0,3.7,18.7],
        'm2b_config_sha256':hashlib.sha256((CONFIG/'exp2b_M2b.yaml').read_bytes()).hexdigest()})
    save(root/'EXPLORATION_SCOPE.json',{'experiment_id':ID,'markers':MARKERS,'formal_evidence':False,
        'parameter_search':False,'profile':S1})

def phase_metrics(rows,method):
    dt=V4.durations(rows);out={'samples':len(rows),'duration_seconds':sum(dt),'maximum_spacing_seconds':max(dt)}
    quality=H.quality_metrics(rows)
    for key in ('lateral','heading','longitudinal','support','rigid_fit','pairwise_side','progress'):
        values=[r['errors'][key] for r in rows];e=H.exposure(rows,values,.010 if key=='rigid_fit' else None)
        for tag,k in [('peak','peak'),('rms','RMS'),('IAE','IAE')]:out[key+'_error_'+tag]=e[k]
    raw=[(V2.num(r,'yaw_effectiveness_hold_left_raw'),V2.num(r,'yaw_effectiveness_hold_right_raw')) for r in rows]
    peaks=[max(abs(a),abs(b)) for a,b in raw]
    if not all(math.isfinite(v) for pair in raw for v in pair):raise ValueError('nonfinite raw demand')
    margin=[min(1.,max(0.,min(V2.num(r,'agv2_wheel_left_reported_limit')-abs(a),
                             V2.num(r,'agv2_wheel_right_reported_limit')-abs(b))/.010)) for r,(a,b) in zip(rows,raw)]
    out.update({'controller_raw_wheel_peak_mps':max(peaks),
      'controller_raw_wheel_rms_mps':math.sqrt(V4.weighted_mean([(a*a+b*b)/2 for a,b in raw],dt)),
      'wheel_margin_minimum':min(margin),'wheel_margin_p05':V2.percentile(margin,.05),
      'wheel_margin_mean':V4.weighted_mean(margin,dt)})
    for tag,threshold in [('150',.150),('155',.155),('160',.160)]:
        out['controller_raw_above_0p'+tag+'_duration_seconds']=sum(t for p,t in zip(peaks,dt) if p>threshold)
    for tag,threshold in [('1p0',1.),('0p7',.7),('0p5',.5)]:
        out['wheel_margin_below_'+tag+'_duration_seconds']=sum(t for v,t in zip(margin,dt) if v<threshold)
    for tag,threshold in [('5',.5),('7',.7)]:out['J_risk_0p'+tag]=sum(max(0.,threshold-v)*t for v,t in zip(margin,dt))
    for name,prefix in [('degraded','yaw_effectiveness_hold'),('applied','agv2_wheel')]:
        fields=[prefix+'_'+side+('_degraded' if name=='degraded' else '_applied') for side in ('left','right')]
        out[name+'_wheel_peak_mps']=max(abs(V2.num(r,f)) for r in rows for f in fields)
    active=[any(V2.flag(r,f'agv{i}_wheel_{side}_speed_limit_active') for i in (1,2,3) for side in ('left','right')) for r in rows]
    continuous=maximum=0.
    for a,t in zip(active,dt):continuous=continuous+t if a else 0.;maximum=max(maximum,continuous)
    out.update({'physical_speed_limiter_samples':sum(active),'physical_speed_limiter_duration_seconds':sum(t for a,t in zip(active,dt) if a),
                'physical_speed_limiter_max_continuous_seconds':maximum})
    return out,quality

def analyze(run,method,log):
    params=yaml.safe_load((run/'rosparams.yaml').read_text());manifest=yaml.safe_load((run/'manifest.yaml').read_text())
    p=params['formal_fake_algorithm'];runtime=p['formal_fake_runtime']
    assert manifest['experiment_id']==runtime['experiment_id']==ID
    assert runtime['leader']['velocity']==manifest['metrics']['nominal_common_velocity']==.10
    assert runtime['yaw_effectiveness_hold_v5b']=={'enabled':True,'trigger_progress':2.,'gamma_hold':.2,'duration':3.7,'ramp_down':.6,'hold':2.5,'ramp_up':.6}
    for i in (1,2,3):assert params[f'agv{i}']['chassis_controller']['transport_type']=='fake'
    if method=='M2b':
        frozen=yaml.safe_load((CONFIG/'exp2b_M2b.yaml').read_text())
        for key in ('formal_m2b','formal_lower','formal_upper'):assert p[key]==frozen[key],key
    else:assert p['formal_upper']['agents']['risk_gain_upper']==[.1]*3
    def grants(x):
        if isinstance(x,dict):
            for k,v in x.items():
                if k.endswith('hardware_execution_authorized'):assert v is False
                grants(v)
        elif isinstance(x,list):
            for v in x:grants(v)
    grants(params)
    all_rows=list(csv.DictReader((run/'converted/aligned_samples.csv').open()))
    rows=[];seen=set()
    for source in all_rows:
        if not V2.flag(source,'algorithm_valid') or not V2.flag(source,'yaw_effectiveness_hold_state_available'):continue
        wall=V2.num(source,'yaw_effectiveness_hold_wall_time')
        if wall in seen:continue
        seen.add(wall);r=dict(source)
        r['relative_wall_time']=wall-V2.num(r,'yaw_effectiveness_hold_trigger_wall_time') if V2.flag(r,'yaw_effectiveness_hold_triggered') else None
        rows.append(r)
    triggered=next(r for r in rows if V2.flag(r,'yaw_effectiveness_hold_triggered'))
    trigger=V2.num(triggered,'yaw_effectiveness_hold_trigger_wall_time')
    for r in rows:
        r['relative_wall_time']=V2.num(r,'yaw_effectiveness_hold_wall_time')-trigger;r['errors']=V4.error_signals(r)
        for key in ('lateral','heading','longitudinal'):r['errors'][key]=V2.num(r,'agv2_hold_tracker_'+key+'_error')
        assert all(math.isfinite(x) for x in r['errors'].values())
        assert abs(V2.num(r,'yaw_effectiveness_hold_gamma')-H.hold_gamma(r['relative_wall_time']))<1e-12
        assert abs(V2.num(r,'yaw_effectiveness_hold_mean_longitudinal_delta'))<1e-12
        for i in (1,2,3):
            for side in ('left','right'):assert abs(V2.num(r,f'agv{i}_wheel_{side}_reported_limit')-.16)<1e-12
    rows.sort(key=lambda r:r['relative_wall_time'])
    selected={'baseline':[r for r in rows if -3<=r['relative_wall_time']<0],
        'disturbance':[r for r in rows if 0<=r['relative_wall_time']<3.7],
        'recovery':[r for r in rows if 3.7<=r['relative_wall_time']<18.7],
        'primary':[r for r in rows if 0<=r['relative_wall_time']<18.7], 'full_task':rows,
        'startup':[r for r in rows if r['relative_wall_time']<0]}
    phases={};quality={}
    for name,rs in selected.items():
        if rs:phases[name],quality[name]=phase_metrics(rs,method)
    motion=log.read_text(errors='replace').split('Formal execution initialized',1)[-1].split('Formal execution reached',1)[0].lower()
    invalid=any(t in motion for t in ('state_chain_invalid','numerical_invalid','structural_geometry_invalid','held fail-zero:'))
    validation=json.loads((run/'validation.json').read_text());summary=json.loads((run/'summary_metrics.json').read_text()) if (run/'summary_metrics.json').exists() else {}
    complete=bool(validation.get('task_completion',{}).get('complete'))
    full_horizon=selected['recovery'] and selected['recovery'][-1]['relative_wall_time']>=18.67
    category='INVALID_RUN' if invalid or not full_horizon else 'VALID_COMPLETED' if complete else 'METHOD_PERFORMANCE_RESULT_TASK_TIMEOUT'
    measured=H.measured_rigid_quality(run,trigger,rows,3.7)
    if method=='M2b':
        for phase in measured.values():
            context=phase.get('first_exceedance_context')
            if context:
                context['agv2_robust_margin']=None
                context['M2b_load_velocity_reference']=context.pop('common_velocity_reference',None)
    for name in phases:
        key='post_trigger' if name=='primary' else 'pulse' if name=='disturbance' else name
        if key in measured:phases[name]['rigid_fit_above_10mm_duration_seconds']=measured[key]['duration_above_threshold_seconds']
    mechanism=None
    if method!='M2b':
        rs=selected['primary'];dt=V4.durations(rs)
        reduction=[V2.num(r,'candidate_common_velocity')-V2.num(r,'common_velocity_reference') for r in rs]
        contraction=[V2.num(r,'risk_contraction') for r in rs]
        active=[V4.V3.boundary_active(r) for r in rs]
        mechanism={'risk_contraction_peak':max(contraction),'risk_contraction_mean':V4.weighted_mean(contraction,dt),
          'risk_contraction_integral':sum(v*t for v,t in zip(contraction,dt)),
          'risk_boundary_active_duration':sum(t for a,t in zip(active,dt) if a),
          'risk_boundary_active_fraction':sum(t for a,t in zip(active,dt) if a)/sum(dt),
          'actual_reference_reduction_peak':max(reduction),'actual_reference_reduction_mean':V4.weighted_mean(reduction,dt),
          'actual_reference_reduction_positive_fraction':sum(t for v,t in zip(reduction,dt) if v>1e-5)/sum(dt),
          'published_common_reference_min':min(V2.num(r,'common_velocity_reference') for r in rs),
          'published_common_reference_mean':V4.weighted_mean([V2.num(r,'common_velocity_reference') for r in rs],dt),
          'robust_margin_min':min(V2.num(r,'agv2_robust_margin') for r in rs),
          'robust_margin_p05':V2.percentile([V2.num(r,'agv2_robust_margin') for r in rs],.05),
          'robust_margin_mean':V4.weighted_mean([V2.num(r,'agv2_robust_margin') for r in rs],dt)}
    m2b=None
    if method=='M2b':
        m2b={f: {'min':min(V2.num(r,f) for r in rows),'max':max(V2.num(r,f) for r in rows)}
              for f in rows[0] if f.startswith('m2b_') and all(math.isfinite(V2.num(r,f)) for r in rows)}
        assert m2b,'missing M2b method diagnostics'
    bounds={key:V4.envelope([r['errors'][key] for r in selected['baseline']],floor) for key,floor in H.FLOORS.items()}
    recovery={key:V4.recovery_time(selected['recovery'],key,bounds[key],3.7) for key in bounds} if selected['recovery'] else {}
    return {'experiment_id':ID,'markers':MARKERS,'formal_evidence':False,'method':METHODS[method],
      'category':category,'complete':complete,'task_time_seconds':summary.get('task',{}).get('completion_time'),
      'phases':phases,'geometry_all_robots_and_sides':quality,'measured_rigid_fit':measured,
      'M1_method_specific_mechanism':mechanism,'M2b_method_specific_diagnostics':m2b,'recovery_times_seconds':recovery,
      'whole_recorded_execution':H.whole_recorded_execution(all_rows,rows),
      'wheel_margin_definition':'clip(min(reported physical wheel limit - abs(current pre-disturbance raw wheel demand))/0.010,0,1); same for all methods',
      'run_path':str(run)}

def descriptive(values):
    values=[v for v in values if v is not None and math.isfinite(v)];n=len(values)
    if not n:return {'n':0,'mean':None,'std':None,'median':None,'min':None,'max':None,'ci95_low':None,'ci95_high':None}
    mean=statistics.mean(values);sd=statistics.stdev(values) if n>1 else None
    half=stats.t.ppf(.975,n-1)*sd/math.sqrt(n) if sd is not None else None
    return {'n':n,'mean':mean,'std':sd,'median':statistics.median(values),'min':min(values),'max':max(values),
            'ci95_low':mean-half if half is not None else None,'ci95_high':mean+half if half is not None else None}

def metric(d,key):return d['task_time_seconds'] if key=='task_time_seconds' else d['phases']['primary'].get(key)

def csv_write(path,rows):
    if not rows:return
    with path.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)

def aggregate(root):
    triads=[];absolute=[];pairs={'M1b':[],'M2b':[]}
    for triad in sorted(root.glob('Triad[0-9][0-9]')):
        data={m:json.loads((triad/folder/'run/repeat_metrics.json').read_text()) for m,folder in METHODS.items()}
        triads.append({'triad':triad.name,'order':json.loads((triad/'order.json').read_text()),'methods':data})
        local={'M1b':[],'M2b':[]}
        for key in CORE:
            for m,d in data.items():absolute.append({'scope':' / '.join(MARKERS),'triad':triad.name,'method':METHODS[m],'metric':key,'value':metric(d,key)})
            for comparator in pairs:
                a=metric(data['M1'],key);b=metric(data[comparator],key)
                valid=all(data[m]['category']=='VALID_COMPLETED' for m in ('M1',comparator))
                row={'scope':' / '.join(MARKERS),'triad':triad.name,'metric':key,'baseline_value':b,'M1_value':a,
                    'baseline_minus_M1':b-a if valid and a is not None and b is not None else None,
                    'relative_improvement_percent':100*(b-a)/b if valid and a is not None and b is not None and abs(b)>1e-12 else None,
                    'comparison_valid':valid}
                pairs[comparator].append(row);local[comparator].append(row)
        csv_write(triad/'M1_vs_M1b.csv',local['M1b']);csv_write(triad/'M1_vs_M2b.csv',local['M2b'])
    csv_write(root/'method_absolute_metrics.csv',absolute);csv_write(root/'triad_metrics.csv',absolute)
    absolute_stats=[]
    for method in METHODS.values():
        for key in CORE:
            absolute_stats.append({'method':method,'metric':key,
                **descriptive([r['value'] for r in absolute if r['method']==method and r['metric']==key])})
    csv_write(root/'method_absolute_statistics.csv',absolute_stats)
    summaries={}
    for comparator,rows in pairs.items():
        csv_write(root/('M1_vs_M1b_pairwise_improvements.csv' if comparator=='M1b' else 'M1_vs_M2b_pairwise_comparison.csv'),rows)
        summary=[]
        for key in CORE:
            selected=[r for r in rows if r['metric']==key and r['comparison_valid']]
            for field in ('baseline_minus_M1','relative_improvement_percent'):
                summary.append({'metric':key,'statistic':field,**descriptive([r[field] for r in selected])})
        summaries[comparator]=summary;csv_write(root/f'summary_M1_vs_{comparator}.csv',summary)
    mechanisms=[x['methods']['M1']['M1_method_specific_mechanism'] for x in triads]
    active=all(m and m['risk_contraction_peak']>0 and m['risk_boundary_active_duration']>0 and m['actual_reference_reduction_peak']>1e-5 for m in mechanisms)
    geom={k:descriptive([r['relative_improvement_percent'] for r in pairs['M1b'] if r['metric']==k]) for k in GEOMETRY}
    valid=all(d['category']=='VALID_COMPLETED' for t in triads for d in t['methods'].values())
    ablation_valid=all(t['methods'][m]['category']=='VALID_COMPLETED' for t in triads for m in ('M1','M1b'))
    risk=descriptive([r['relative_improvement_percent'] for r in pairs['M1b'] if r['metric']=='J_risk_0p5'])
    ablation=len(triads)>=5 and ablation_valid and active and risk['mean'] is not None and risk['mean']>10 and all(s['mean'] is not None and s['mean']>=5 for s in geom.values()) and all(
        sum(r['baseline_minus_M1']>0 for r in pairs['M1b'] if r['metric']==k and r['baseline_minus_M1'] is not None)>len(triads)/2 for k in GEOMETRY)
    bgeom={k:descriptive([r['relative_improvement_percent'] for r in pairs['M2b'] if r['metric']==k]) for k in GEOMETRY}
    directions=[s['mean'] for s in bgeom.values() if s['mean'] is not None]
    # Task cost or resource/geometry tradeoffs must not be hidden by a geometry-only label.
    bcore=[descriptive([r['baseline_minus_M1'] for r in pairs['M2b'] if r['metric']==k])['mean'] for k in ['J_risk_0p5','physical_speed_limiter_duration_seconds','task_time_seconds']]
    label='M1_VS_M2B_MIXED_RESULT'
    if valid and directions and all(v>0 for v in directions) and all(v is not None and v>=0 for v in bcore):label='M1_VS_M2B_ADVANTAGE_OBSERVED'
    elif valid and directions and all(v<0 for v in directions) and all(v is not None and v<=0 for v in bcore):label='M2B_OUTPERFORMS_M1_UNDER_S1'
    result={'experiment_id':ID,'markers':MARKERS,'formal_evidence':False,'triads_completed':len(triads),
      'all_runs_valid_completed':valid,'mechanism_active_every_M1':active,
      'ablation_status':'M1_ABLATION_REPEATABILITY_CONFIRMED' if ablation else 'M1_ABLATION_REPEATABILITY_NOT_CONFIRMED',
      'literature_comparison_status':label,'paired_summaries':summaries,'method_absolute_statistics':absolute_stats,'triads':triads,
      'CI_definition':'Student-t 95% CI for mean paired difference / mean per-triad percentage, n=5; no historical pair included'}
    save(root/'repeat_validation_summary.json',result)
    plots(root,triads,pairs)
    lines=['# REPEAT_VALIDATION / NOT_FORMAL_PAPER_EVIDENCE','',result['ablation_status'],'',label,'',
      'Frozen S1, M1 upper risk gain 0.10. No parameter search. Five triads only; all unfavorable runs retained.',
      '', '| Comparison | Metric | Mean improvement % | SD | 95% CI |','| --- | --- | ---: | ---: | --- |']
    for name,summ in summaries.items():
        for s in summ:
            if s['statistic']=='relative_improvement_percent':lines.append(f"| M1 vs {name} | {s['metric']} | {s['mean']} | {s['std']} | [{s['ci95_low']}, {s['ci95_high']}] |")
    lines+=['','Zero-baseline exposure has N/A percentage, not zero; absolute paired differences remain available.',
      'Student-t CIs describe mean paired effects across fresh fake runs; scheduler variation is not independent physical-world evidence.',
      'M2b internal risk boundary/robust margin/reference reduction are N/A, not fabricated zeros.',
      'All source/native diagnostics, signed geometry, measured rigid fit, censoring and startup limiter exposure are retained per run.']
    (root/'repeat_validation_summary.md').write_text('\n'.join(lines)+'\n')
    return result

def plots(root,triads,pairs):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    output=root/'analysis_plots';output.mkdir(exist_ok=True)
    for key in GEOMETRY+['J_risk_0p5','controller_raw_above_0p160_duration_seconds','physical_speed_limiter_duration_seconds','task_time_seconds']:
        fig,ax=plt.subplots()
        for m in METHODS:
            ax.plot(range(1,len(triads)+1),[metric(t['methods'][m],key) for t in triads],marker='o',label=METHODS[m])
        ax.set(xlabel='Fresh triad',ylabel=key,title='REPEAT_VALIDATION / NOT_FORMAL_PAPER_EVIDENCE');ax.legend();fig.tight_layout()
        fig.savefig(output/(key+'.png'));fig.savefig(output/(key+'.pdf'));plt.close(fig)
    for c in pairs:
        fig,ax=plt.subplots()
        for key in GEOMETRY:ax.plot(range(1,len(triads)+1),[r['relative_improvement_percent'] for r in pairs[c] if r['metric']==key],marker='o',label=key)
        ax.axhline(0,color='black');ax.set(xlabel='Fresh triad',ylabel='Relative improvement (%)',title=f'NOT_FORMAL_PAPER_EVIDENCE: M1 vs {c}');ax.legend();fig.tight_layout()
        fig.savefig(output/f'M1_vs_{c}_geometry.png');fig.savefig(output/f'M1_vs_{c}_geometry.pdf');plt.close(fig)

def main():
    p=argparse.ArgumentParser();p.add_argument('--prepare',type=Path);p.add_argument('--method',choices=METHODS)
    p.add_argument('--check-run',type=Path);p.add_argument('--check-log',type=Path);p.add_argument('--aggregate',type=Path)
    a=p.parse_args()
    if a.prepare:prepare(a.prepare.resolve(),a.method);return 0
    if a.aggregate:aggregate(a.aggregate.resolve());return 0
    try:result=analyze(a.check_run.resolve(),a.method,a.check_log.resolve())
    except Exception as e:result={'markers':MARKERS,'category':'INVALID_RUN','reason':str(e)}
    save(a.check_run/'repeat_metrics.json',result);print(result['category'])
    return 20 if result['category']=='INVALID_RUN' else 0

if __name__=='__main__':raise SystemExit(main())
