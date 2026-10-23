#!/usr/bin/env python3
"""Build adapters against pinned official upstream source snapshots."""
from pathlib import Path
import os,subprocess,shutil,yaml
root=Path(__file__).resolve().parents[1]
versions={'unitree_sdk2':'d1abf001220d3e5f59e00a0c4aaa595e00089a4a','unitree_rl_mjlab':'1425b15f73bd4095f0df53709d7c389c3eb9e790'}
for name,sha in versions.items():
 path=root/'.deps/src'/name
 if not (path/'.git').exists():
  raise SystemExit(f'{name}: submodule not initialized; run scripts/setup_g1_local_deps.sh')
 actual=subprocess.check_output(['git','-C',str(path),'rev-parse','HEAD'],text=True).strip()
 if actual!=sha:raise SystemExit(f'{name}: expected {sha}, found {actual}; run scripts/setup_g1_local_deps.sh')
up=root/'.deps/src/unitree_rl_mjlab/deploy/robots/g1/config';out=root/'.deps/g1-controller';out.mkdir(exist_ok=True)
cfg=yaml.safe_load((up/'config.yaml').read_text());fsm=cfg['FSM'];fsm['_']={k:v for k,v in fsm['_'].items() if k in ['Passive','FixStand','Velocity']}
for k in list(fsm):
 if k not in ['_','Passive','FixStand','Velocity']:del fsm[k]
for k in ['Passive','FixStand','Velocity']:fsm[k]['transitions']={}
policy=out/'policy';(policy/'params').mkdir(exist_ok=True,parents=True);(policy/'exported').mkdir(exist_ok=True)
s=(up/'policy/velocity/v0/params/deploy.yaml').read_text().replace('  velocity_commands:','  control_velocity_commands:').replace('  gait_phase:','  control_gait_phase:')
(policy/'params/deploy.yaml').write_text(s);shutil.copy2(up/'policy/velocity/v0/exported/policy.onnx',policy/'exported/policy.onnx');fsm['Velocity']['policy_dir']=str(policy)
(out/'config.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
env=os.environ.copy()
for key in ('AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH','PYTHONPATH','LD_LIBRARY_PATH',
            'CONDA_PREFIX','CONDA_DEFAULT_ENV','CONDA_PYTHON_EXE'):
 env.pop(key,None)
env['PATH']='/usr/bin:/bin'
subprocess.run(['cmake','-S',str(root/'controller'),'-B',str(root/'.deps/controller-build'),
                '-Dfmt_DIR=/usr/lib/x86_64-linux-gnu/cmake/fmt'],check=True,env=env)
subprocess.run(['cmake','--build',str(root/'.deps/controller-build'),'-j3'],check=True,env=env)
