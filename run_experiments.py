"""Reproduce the experiment matrix. Failures of baseline methods are expected.

No external services, training jobs or GPU are used. Logs and an outcome table
are kept under outputs/reproduction; the original archived results stay intact.
"""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parent

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--quick',action='store_true',help='one integrated positive case and one negative baseline')
    a=parser.parse_args()
    out=ROOT/('outputs/quick' if a.quick else 'outputs/reproduction');out.mkdir(parents=True,exist_ok=True)
    tests=[]
    if not a.quick:
        for script in ['check_assets.py','check_clearance_geometry.py']:
            subprocess.run([sys.executable,str(ROOT/script)],check=True,cwd=ROOT)
        tests += [('navigation','verify_navigation.py',['--seeds','0','1','2'],'passed',True)]
        tests += [('straight','verify_person_follow.py',['--seeds','0','1','2'],'passed',True)]
        tests += [('clearance','verify_clearance.py',['--seeds','0','1','2'],'passed',True)]
        tests += [('robot_only','verify_clearance.py',['--mode','robot_only'],'passed',False)]
        tests += [('narrow_margin','verify_clearance.py',['--margin','.32'],'passed',False)]
    starts=[8] if a.quick else [8,20,32]
    seeds=['0'] if a.quick else ['0','1','2']
    for start in starts:
        tests.append((f'integrated_p{start}','verify_integrated.py',['--pause-start',str(start),'--seeds',*seeds],'guidance_passed',True))
    tests.append(('no_wait','verify_integrated.py',['--waiting','disabled'],'guidance_passed',False))
    if not a.quick:tests.append(('no_pause','verify_integrated.py',['--pause-duration','0'],'guidance_passed',True))
    rows=[]
    for name,script,flags,key,expected in tests:
        command=[sys.executable,str(ROOT/script),'--output',f'{out.relative_to(ROOT / "outputs")}/{name}',*flags]
        print('Running',name,flush=True)
        result=subprocess.run(command,cwd=ROOT,capture_output=True,text=True)
        (out/f'{name}.log').write_text(result.stdout+'\n'+result.stderr)
        if result.returncode:raise RuntimeError(f'{name} failed to execute; see its log')
        files=sorted(out.glob(f'{name}_*.json'))
        if not files:raise RuntimeError(f'{name} produced no JSON')
        for file in files:
            data=json.loads(file.read_text())
            # A negative baseline must fail for the intended physical reason.
            if name=='robot_only':assert data['person_collision']['obstacle']=='low_lintel'
            if name=='narrow_margin':assert data['person_collision']['obstacle']=='left_post'
            if name=='no_wait':assert data['mission_passed'] and data['max_agent_center_gap_m']>1.45 and data['wait_count']==0
            if name=='no_pause':assert data['wait_count']==0
            rows.append({'case':name,'seed':data['seed'],'metric':key,'actual':data[key],
                         'expected':expected,'matches_expectation':data[key]==expected,
                         'min_clearance_m':data.get('min_person_clearance_m'),
                         'max_gap_m':data.get('max_agent_center_gap_m',data.get('max_gap_m')),
                         'wait_count':data.get('wait_count'),'resume_count':data.get('resume_count'),
                         'wait_motion_m':data.get('wait_motion_during_pause_m'),
                         'simulation_seconds':data['simulation_seconds'],'result':file.name})
    (out/'summary.json').write_text(json.dumps(rows,indent=2))
    with (out/'summary.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    print(json.dumps(rows,indent=2))
    if not all(r['matches_expectation'] for r in rows):sys.exit(1)

if __name__=='__main__':main()
