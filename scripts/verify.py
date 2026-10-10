"""M16 local verification evidence collector (no paid CI, network, or uploads).

Run from cloned repo: python scripts/verify.py [--gui]
It writes actual measured logs to evidence/local/<run-id>/ without changing
existing evidence or reporting success before the commands finish.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
from datetime import datetime,timezone

ROOT=Path(__file__).resolve().parents[1]


def run_command(cmd,location,label,timeout=180):
    try:
        result=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,
                              timeout=timeout,check=False,encoding='utf-8',errors='replace')
        output=result.stdout+'\n[STDERR]\n'+result.stderr
        status={'command':cmd,'return_code':result.returncode,'passed':result.returncode==0}
    except subprocess.TimeoutExpired as exc:
        output=(str(exc.stdout or b'')+'\n'+str(exc.stderr or b''))
        status={'command':cmd,'return_code':None,'passed':False,'error':'timeout'}
    (location/(label+'.log')).write_text(output,encoding='utf-8')
    return status


def git_info():
    def git(*args):
        try:
            r=subprocess.run(['git',*args],cwd=ROOT,stdout=subprocess.PIPE,
                             stderr=subprocess.DEVNULL,text=True,timeout=5)
            return r.stdout.strip() if r.returncode==0 else None
        except (OSError,subprocess.TimeoutExpired):
            return None
    return {'commit':git('rev-parse','HEAD'),'branch':git('branch','--show-current'),
            'dirty_status':git('status','--short')}


def verify(gui=False,steps=1000):
    label=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    directory=ROOT/'evidence'/'local'/label
    if directory.exists():
        raise FileExistsError('evidence folder already exists; run again later')
    directory.mkdir(parents=True)
    checks={}
    py=sys.executable
    checks['unit']=run_command([py,'-m','unittest','discover','-s','tests','-v'],
                               directory,'unit_tests')
    checks['scenario']=run_command([py,'-m','robot_sim','--evaluate','--trials','5'],
                                   directory,'scripted_scenario')
    checks['benchmark']=run_command([py,'scripts/benchmark.py','--steps',str(steps)],
                                    directory,'benchmark')
    if gui:
        for name,file in [('basic_gui','tests/gui_smoke.py'),
                          ('advanced_gui','tests/advanced_gui_smoke.py'),
                          ('full_gui','tests/full_gui_smoke.py')]:
            checks[name]=run_command([py,file],directory,name)
    report={
        'timestamp_utc':label,
        'host':platform.node(),
        'platform':platform.platform(),
        'python':sys.version,
        'logical_cpus':os.cpu_count(),
        'git':git_info(),
        'checks':checks,
        'all_executed_checks_passed':all(v['passed'] for v in checks.values()),
        'gui_requested':gui,
        'gui_skipped':not gui,
        'native_gui_user_review':'PENDING',
        'joint_user_assistant_acceptance':'PENDING',
        'limitations':'Headless passes do not prove 3D rigid-body accuracy or physical safety.'
    }
    (directory/'report.json').write_text(json.dumps(report,indent=2,sort_keys=True)+'\n',
                                          encoding='utf-8')
    return directory,report


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--gui',action='store_true',help='run real Tk GUI smoke scripts')
    parser.add_argument('--steps',type=int,default=1000)
    args=parser.parse_args()
    out,report=verify(gui=args.gui,steps=args.steps)
    print('Evidence path:',out)
    for name,item in report['checks'].items():
        print(name, 'PASS' if item['passed'] else 'FAIL',item['return_code'])
    print('Local checks passed:',report['all_executed_checks_passed'])
    print('User/assistant acceptance: PENDING')
    if not report['all_executed_checks_passed']:
        raise SystemExit(1)
