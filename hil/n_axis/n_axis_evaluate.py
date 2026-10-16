#!/usr/bin/env python3
"""Evaluate controller originals; no motor-application or physical-safety claim."""
import argparse
import csv
import itertools
import json
import math
from pathlib import Path

STATES = {0:'boot',1:'idle',2:'armed',3:'running',4:'holding',5:'released',6:'stopped',7:'aborted'}

def load(path, allow_empty=False):
    lines=Path(path).read_text().splitlines()
    meta={}
    for line in lines:
        if line.startswith('#'):
            for field in line[1:].split(';'):
                if '=' in field:
                    k,v=field.split('=',1); meta[k]=v
    reader=csv.DictReader([x for x in lines if not x.startswith('#')])
    rows=list(reader)
    if (not rows and not allow_empty) or not reader.fieldnames or len(set(reader.fieldnames)) != len(reader.fieldnames) or any(None in r or any(v is None for v in r.values()) for r in rows):
        raise ValueError('empty original or inconsistent columns')
    return meta,reader.fieldnames,rows

def reference(t):
    t=min(t,35.); w=2*math.pi*.2; A=math.pi/6; e=1.; de=0.
    if t<5:
        e=(1-math.cos(math.pi*t/5))/2; de=math.pi/10*math.sin(math.pi*t/5)
    elif t>30:
        e=(1+math.cos(math.pi*(t-30)/5))/2; de=-math.pi/10*math.sin(math.pi*(t-30)/5)
    return A*e*math.sin(w*t), A*(de*math.sin(w*t)+w*e*math.cos(w*t))

def evaluate(path, pair_commits=None, pair_safety=None, pair_feedback_stale=None,
             reference_tolerance=1e-9, axis_difference=.10, phase_degrees=10.):
    out={}
    def item(name,status,**details): out[name]={'result':status,**details}
    try:
        if any(not math.isfinite(v) or v<0 for v in (reference_tolerance,axis_difference,phase_degrees)):
            raise ValueError('thresholds must be finite nonnegative provisional values')
        meta,columns,rows=load(path)
        n=int(rows[0]['N']); capacity=int(meta['capacity']); dropped=int(meta['dropped'])
        if n not in (2,3): raise ValueError('N must be 2 or 3')
        required={'N','activation','cycle','nonce','request_id','g','previous_g','accepted','accepted_result','advanced','held','holds','max_holds','failures','elapsed','before','after','reason','error_return_ns','heartbeat_sent_ns','lease_deadline_ns','start_ns'}
        required.update('accepted_ns end_ns detected_ns request_sent_ns request_received_ns acceleration state_ok material_missing release_attempted'.split())
        for kind in ('normal','release'): required.update({kind+'_commit_set_ns',kind+'_commit_read_ns'})
        for a in range(n): required.update({f'joint{a}',f'position{a}',f'velocity{a}',f'effort{a}'})
        for kind in ('normal','release'):
            required.update(f'{kind}_{x}' for x in ('write_attempted','write_ok','read_attempted','read_ok','unavailable','mismatch'))
            required.update(f'{kind}_{x}{i}' for i in range(5*n+1) for x in ('value','readback'))
        if n not in (2,3) or meta.get('schema')!='n_axis_controller_v1' or not required.issubset(columns) or len(columns)!=51+24*n:
            raise ValueError('schema/N/columns mismatch')
        numeric=required-{f'joint{a}' for a in range(n)}
        integers={k for k in numeric if not any(k.startswith(prefix) for prefix in ('position','velocity','effort','normal_value','normal_readback','release_value','release_readback'))}-{ 'accepted','elapsed','acceleration'}
        for r in rows:
            for key in integers: int(r[key])
            if any(int(r[k]) not in (0,1) for k in ('advanced','held','release_attempted')):
                raise ValueError('invalid boolean column')
            if int(r['before']) not in STATES or int(r['after']) not in STATES or not 0<=int(r['accepted_result'])<=4 or not 0<=int(r['reason'])<=14:
                raise ValueError('unknown state/result/reason')
            if int(r['N'])!=n or any(r[f'joint{a}']!=f'joint{a+1}' for a in range(n)):
                raise ValueError('N or joint order changes')
            # accepted may be nan when its result explicitly records unavailable/nonfinite.
            for key in numeric-{'accepted'}:
                if not math.isfinite(float(r[key])): raise ValueError('nonfinite '+key)
            if int(r['accepted_result']) not in (3,4) and not math.isfinite(float(r['accepted'])):
                raise ValueError('unrecorded invalid accepted')
        good=dropped==0 and 0<len(rows)<=capacity and bool(meta.get('provisional'))
        item('integrity','PASS' if good else 'FAIL',N=n,rows=len(rows),capacity=capacity,dropped=dropped,provisional=meta.get('provisional'))
    except (OSError,ValueError,KeyError,TypeError) as e:
        item('integrity','FAIL',message=str(e))
        for k in ('generation','reference','measurement','lease_end','hwif'):
            item(k,'NOT_EVALUATED',message='invalid/missing controller original')
        return out
    mask=(1<<(5*n+1))-1; errors=[]; holds=0; max_holds=0; counts={}; previous=0
    for cycle,r in enumerate(rows,1):
        g=int(r['g']); before=int(r['previous_g']); held=int(r['held']); advanced=int(r['advanced'])
        counts[r['accepted_result']]=counts.get(r['accepted_result'],0)+1
        if not 1<=g<=9007199254740991 or int(r['cycle'])!=cycle or before!=previous or (cycle==1 and g!=1) or (cycle>1 and (g!=before+advanced or advanced+held!=1)):
            errors.append(cycle)
        if advanced and float(r['accepted'])!=before: errors.append(cycle)
        holds=holds+1 if held else 0; max_holds=max(max_holds,holds)
        if int(r['holds'])!=holds or int(r['max_holds'])!=max_holds: errors.append(cycle)
        if int(r['after'])!=7:
            if any(int(r[f'normal_{key}'])!=mask for key in ('write_attempted','write_ok','read_attempted','read_ok')) or int(r['normal_unavailable']) or int(r['normal_mismatch']): errors.append(cycle)
            if any(float(r[f'normal_value{i}'])!=float(r[f'normal_readback{i}']) for i in range(5*n+1)): errors.append(cycle)
            if float(r[f'normal_value{5*n}'])!=g: errors.append(cycle)
        previous=g
    item('generation','FAIL' if errors else 'PASS',bad_cycles=sorted(set(errors)),max_consecutive_holds=max_holds,accepted_results=counts,failures=int(rows[-1]['failures']))
    running=[r for r in rows if int(r['after']) in (3,4)]
    differences=[]
    for r in running:
        q,v=reference(float(r['elapsed'])); v=0. if int(r['after'])==4 else v
        for a in range(n):
            differences.extend([abs(float(r[f'normal_value{5*a}'])-q),abs(float(r[f'normal_value{5*a+1}'])-v)])
            for j in range(5): differences.append(abs(float(r[f'normal_value{5*a+j}'])-float(r[f'normal_value{j}'])))
    item('reference', 'NOT_EVALUATED' if not running else 'PASS' if max(differences)<=reference_tolerance else 'FAIL',
         max_error=max(differences,default=None),tolerance=reference_tolerance,transition_cycle='common controller row; no independent motor transition observation')
    flat=[r for r in running if int(r['after'])==3 and 5<=float(r['elapsed'])<30]
    axes=[]; pairs=[]
    try:
        if len(flat)>=20 and max(float(r['elapsed']) for r in flat)-min(float(r['elapsed']) for r in flat)>=5:
            phase=[]
            for a in range(n):
                measured=[float(r[f'position{a}']) for r in flat]; desired=[reference(float(r['elapsed']))[0] for r in flat]
                err=[m-q for m,q in zip(measured,desired)]
                # Independent least-squares fit p=center+B sin(wt)+C cos(wt), no numpy dependency.
                basis=[(1.,math.sin(2*math.pi*.2*float(r['elapsed'])),math.cos(2*math.pi*.2*float(r['elapsed']))) for r in flat]
                matrix=[[sum(x[i]*x[j] for x in basis) for j in range(3)]+[sum(x[i]*m for x,m in zip(basis,measured))] for i in range(3)]
                for i in range(3):
                    pivot=max(range(i,3),key=lambda j:abs(matrix[j][i])); matrix[i],matrix[pivot]=matrix[pivot],matrix[i]
                    if abs(matrix[i][i])<1e-12: raise ValueError('singular measurement fit')
                    divisor=matrix[i][i]; matrix[i]=[v/divisor for v in matrix[i]]
                    for j in range(3):
                        if j!=i:
                            multiple=matrix[j][i]; matrix[j]=[v-multiple*w for v,w in zip(matrix[j],matrix[i])]
                center,b,c=[x[3] for x in matrix]; amplitude=math.hypot(b,c); phase.append(math.atan2(c,b))
                axes.append({'joint':a+1,'rms_error':math.sqrt(sum(e*e for e in err)/len(err)),'max_error':max(map(abs,err)),
                             'amplitude_ratio':amplitude/(math.pi/6),'center':center,'phase_defined':amplitude>1e-9})
            for a,b in itertools.combinations(range(n),2):
                delta=abs(math.degrees(math.atan2(math.sin(phase[a]-phase[b]),math.cos(phase[a]-phase[b]))))
                difference=max(abs(float(r[f'position{a}'])-float(r[f'position{b}'])) for r in flat)
                pairs.append({'axes':[a+1,b+1],'phase_degrees':delta if axes[a]['phase_defined'] and axes[b]['phase_defined'] else None,'max_difference':difference})
            defined=all(a['phase_defined'] for a in axes)
            failed=any(p['max_difference']>axis_difference or (p['phase_degrees'] is not None and p['phase_degrees']>phase_degrees) for p in pairs)
            item('measurement','FAIL' if failed else 'PASS' if defined else 'NOT_EVALUATED',axes=axes,pairs=pairs,
                 provisional_thresholds={'axis_difference_rad':axis_difference,'phase_degrees':phase_degrees},note='RMS/amplitude/center are descriptive; no approved individual tracking thresholds')
        else: item('measurement','NOT_EVALUATED',message='insufficient stationary interval')
    except (ValueError, ZeroDivisionError) as e:
        item('measurement','NOT_EVALUATED',message=str(e))
    invalid_lease=[]
    for r in running:
        if int(r['lease_deadline_ns'])!=int(r['heartbeat_sent_ns'])+500000000 or int(r['start_ns'])>=int(r['lease_deadline_ns']): invalid_lease.append(int(r['cycle']))
    last=rows[-1]; state=int(last['after']); abnormal=state==7 or any(int(r['error_return_ns']) for r in rows)
    normal_reason=(state==5 and int(last['reason']) in (1,2)) or (state==6 and int(last['reason'])==3)
    bad_failures=int(last['failures'])!=sum(int(r['after'])==7 for r in rows)
    item('lease_end','FAIL' if invalid_lease or abnormal or bad_failures or (state in (5,6) and not normal_reason) else 'PASS' if normal_reason else 'NOT_EVALUATED',
         terminal_state=STATES.get(state,'unknown'),reason=int(last['reason']),abnormal=abnormal,bad_cycles=invalid_lease)
    summaries={}
    for name,file in [('commits',pair_commits),('safety',pair_safety),('feedback_stale',pair_feedback_stale)]:
        if not file: summaries[name]={'result':'NOT_EVALUATED','message':'not supplied'}; continue
        try:
            metadata,fields,hw=load(file,allow_empty=True)
            if 'dropped' not in metadata: raise ValueError('HWIF drop metadata missing')
            if int(metadata['dropped']): raise ValueError('HWIF dropped records')
            expected={'commits': {'accepted_g'}, 'safety': {'epoch','N','sources','result','send_attempted','send_success'}, 'feedback_stale': {'epoch','N','sources','result'}}[name]
            if not expected.issubset(fields): raise ValueError('HWIF columns mismatch')
            if name=='commits':
                # Actual pair_commits schema is checked below, never infer an epoch from g alone.
                key='accepted_g' if 'accepted_g' in fields else None
                if key is None: raise ValueError('accepted generation column missing')
                accepted={int(r[key]) for r in hw}; missing=sorted({int(r['g']) for r in rows if int(r['after'])!=7}-accepted)
                summaries[name]={'result':'FAIL' if missing else 'PASS','missing_generations':missing,'rows':len(hw),
                                 'message':'generation coverage only; epoch/run correspondence must be confirmed externally'}
            else:
                summaries[name]={'result':'NOT_EVALUATED','rows':len(hw),'columns':fields,'last_row':hw[-1] if hw else None,
                                 'message':'content summary only; epoch and send result semantics require external review'}
        except (OSError,ValueError,KeyError) as e: summaries[name]={'result':'FAIL','message':str(e)}
    item('hwif','FAIL' if any(v['result']=='FAIL' for v in summaries.values()) else 'NOT_EVALUATED',files=summaries,
         message='readback is not HWIF ack; epoch/run correspondence not established by this evaluator')
    return out

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('csv',type=Path)
    for x in ('pair-commits','pair-safety','pair-feedback-stale'): p.add_argument('--'+x,type=Path)
    p.add_argument('--reference-tolerance',type=float,default=1e-9);p.add_argument('--axis-difference',type=float,default=.10);p.add_argument('--phase-degrees',type=float,default=10.)
    p.add_argument('--json',type=Path); a=p.parse_args(argv)
    result=evaluate(a.csv,a.pair_commits,a.pair_safety,a.pair_feedback_stale,a.reference_tolerance,a.axis_difference,a.phase_degrees)
    payload=json.dumps(result,indent=2,allow_nan=False)
    if a.json:a.json.write_text(payload+'\n')
    print(payload)
    print('; '.join(k+': '+v['result'] for k,v in result.items()))
    return int(any(v['result']=='FAIL' for v in result.values()))
if __name__=='__main__': raise SystemExit(main())
