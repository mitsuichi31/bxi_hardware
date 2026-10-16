import copy
import csv
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
import pytest
import yaml

ROOT=Path(__file__).resolve().parents[1]
HERE=ROOT/'hil/n_axis'
def module(name):
    spec=importlib.util.spec_from_file_location(name,HERE/(name+'.py'))
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
make=module('make_n_axis_config'); check=module('n_axis_check_config'); evaluate=module('n_axis_evaluate'); command=module('n_axis_command')

@pytest.mark.parametrize('n',[2,3])
def test_generate_validate_xacro_and_launch_static(tmp_path,n):
    output=tmp_path/'run';output.mkdir();d=make.generate(n,list(range(1,n+1)),tmp_path/'config',output)
    subprocess.run([sys.executable,str(ROOT/'scripts/validate_config.py'),str(d/'hardware.yaml'),'--schema',str(ROOT/'config/bxi_hardware.schema.json'),'--spec-dir',str(ROOT/'config/specs')],check=True,capture_output=True)
    xml=tmp_path/'robot.xml'
    with xml.open('w') as f: subprocess.run(['xacro',str(HERE/'n_axis.urdf.xacro'),'axes:='+str(n),'config_file:='+str(d/'hardware.yaml'),'spec_dir:='+str(ROOT/'config/specs')],stdout=f,check=True)
    assert check.check(d,xml)==[]
    spec=importlib.util.spec_from_file_location('g_launch',HERE/'n_axis_hil.launch.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    description=m.generate_launch_description() # No launch execution, CM or CAN.
    assert len(description.entities)==3

@pytest.mark.parametrize('ids', [[],[1],[1,1],[0,2],[1,16],[2,3]])
def test_ids_required_unique_in_range_and_broadcast_order(tmp_path,ids):
    with pytest.raises(ValueError):make.generate(2,ids,tmp_path/'config',tmp_path)

def test_no_id_cli_default():
    result=subprocess.run([sys.executable,str(HERE/'make_n_axis_config.py'),'--axes','3'],capture_output=True,text=True)
    assert result.returncode!=0 and '--ids' in result.stderr

@pytest.mark.parametrize('mutation',['gpio','limiter','extra_controller','disabled','id','provenance','joint'])
def test_check_config_rejects_mutation(tmp_path,mutation):
    d=make.generate(3,[1,2,3],tmp_path/'config',tmp_path)
    import xacro
    xml=tmp_path/'robot.xml';xml.write_text(xacro.process_file(str(HERE/'n_axis.urdf.xacro'),mappings={'axes':'3','config_file':str(d/'hardware.yaml'),'spec_dir':str(ROOT/'config/specs')}).toxml())
    if mutation in ('gpio','limiter','joint'):
        tree=ET.parse(xml);system=tree.getroot().find('ros2_control')
        if mutation=='gpio':system.find('gpio/command_interface').set('data_type','uint64')
        elif mutation=='limiter':ET.SubElement(system.find('gpio/command_interface'),'param',name='min').text='0'
        else:system.find('joint').set('name','alias')
        tree.write(xml)
    elif mutation=='extra_controller':
        p=d/'controllers.yaml';c=yaml.safe_load(p.read_text());c['controller_manager']['ros__parameters']['other']={'type':'forward_command_controller/ForwardCommandController'};p.write_text(yaml.safe_dump(c))
    elif mutation in ('disabled','id'):
        p=d/'hardware.yaml';c=yaml.safe_load(p.read_text());c['joints']['joint3']['enabled' if mutation=='disabled' else 'can_id']=False if mutation=='disabled' else 2;p.write_text(yaml.safe_dump(c,sort_keys=False))
    else:
        p=d/'manifest.json';c=json.loads(p.read_text());c.pop('source');p.write_text(json.dumps(c))
    assert check.check(d,xml)

def synthetic(n=3,phase=0):
    fields='N activation cycle nonce request_id g previous_g accepted accepted_result accepted_ns advanced held holds max_holds failures start_ns end_ns detected_ns error_return_ns heartbeat_sent_ns lease_deadline_ns request_sent_ns request_received_ns elapsed acceleration before after reason state_ok material_missing release_attempted'.split()
    for a in range(n):fields += [f'joint{a}',f'position{a}',f'velocity{a}',f'effort{a}']
    for kind in ('normal','release'):
        fields += [kind+'_'+key for key in 'write_attempted write_ok read_attempted read_ok unavailable mismatch commit_set_ns commit_read_ns'.split()]
        fields += [f'{kind}_{key}{i}' for i in range(5*n+1) for key in ('value','readback')]
    rows=[];mask=(1<<(5*n+1))-1
    times=[0.]+[5+i*.025 for i in range(1000)]+[36.]
    for i,t in enumerate(times):
        row={key:0 for key in fields};row.update(N=n,activation=1,cycle=i+1,nonce=123,request_id=7,g=i+1,previous_g=i,accepted=i,accepted_result=1 if i else 0,advanced=int(i>0),elapsed=t,after=1 if i==0 else 5 if i==len(times)-1 else 3)
        now=2000000000+int(t*1e9);row.update(start_ns=now,end_ns=now,heartbeat_sent_ns=now,lease_deadline_ns=now+500000000)
        q,v=evaluate.reference(t)
        for a in range(n):
            row[f'joint{a}']=f'joint{a+1}'
            row[f'position{a}']=(math.pi/6*math.sin(2*math.pi*.2*t+math.radians(phase)) if a==n-1 and i not in (0,len(times)-1) else q)
            if i not in (0,len(times)-1):
                for j,x in enumerate((q,v,20,1,0)):
                    for key in ('value','readback'):row[f'normal_{key}{a*5+j}']=x
        for key in ('write_attempted','write_ok','read_attempted','read_ok'):row['normal_'+key]=mask
        row[f'normal_value{5*n}']=row[f'normal_readback{5*n}']=i+1
        if i==len(times)-1:row['reason']=1
        rows.append(row)
    return fields,rows

def write_csv(path,fields,rows,dropped=0):
    with path.open('w') as f:
        f.write(f'#schema=n_axis_controller_v1\n#provisional=accepted_hold,api_tries\n#dropped={dropped};capacity=2000\n')
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(rows)

@pytest.mark.parametrize('n',[2,3])
def test_evaluate_synthetic_normal_and_optional_missing(tmp_path,n):
    f,r=synthetic(n);path=tmp_path/'original.csv';write_csv(path,f,r)
    result=evaluate.evaluate(path)
    assert all(result[k]['result']=='PASS' for k in ['integrity','generation','reference','measurement','lease_end'])
    assert result['hwif']['result']=='NOT_EVALUATED'
    assert len(result['measurement']['pairs'])==n*(n-1)//2
    assert max(a['max_error'] for a in result['measurement']['axes'])<1e-12

@pytest.mark.parametrize('mutation,section',[('dropped','integrity'),('column','integrity'),('missing','integrity'),('gap','generation'),('trajectory','reference'),('phase','measurement'),('lease','lease_end')])
def test_evaluate_failures(tmp_path,mutation,section):
    f,r=synthetic(3,phase=25 if mutation=='phase' else 0);path=tmp_path/'original.csv'
    if mutation=='column':f=f+['extra'];r=[dict(x,extra=0) for x in r]
    if mutation=='gap':r[3]['g']+=2
    if mutation=='trajectory':r[3]['normal_value0']+=.01
    if mutation=='lease':r[3]['lease_deadline_ns']-=1
    if mutation=='missing':
        result=evaluate.evaluate(path)
    else:
        write_csv(path,f,r,dropped=int(mutation=='dropped'));result=evaluate.evaluate(path)
    assert result[section]['result']=='FAIL'
    if mutation=='phase':
        # Raise the position-difference threshold to isolate the phase criterion.
        isolated=evaluate.evaluate(path,axis_difference=2.)['measurement']
        assert isolated['result']=='FAIL'
        assert max(p['phase_degrees'] for p in isolated['pairs'])>24.9

def test_hwif_coverage_and_malformed_original(tmp_path):
    f,r=synthetic();path=tmp_path/'original.csv';write_csv(path,f,r)
    hw=tmp_path/'pair.csv';hw.write_text('#dropped=0\naccepted_g\n'+'\n'.join(str(i+1) for i in range(len(r)))+'\n')
    result=evaluate.evaluate(path,pair_commits=hw)
    assert result['hwif']['files']['commits']['result']=='PASS'
    assert result['hwif']['result']=='NOT_EVALUATED' # No epoch/run proof from generation alone.
    hw.write_text('#dropped=0\naccepted_g\n1\n');assert evaluate.evaluate(path,pair_commits=hw)['hwif']['result']=='FAIL'
    hw.write_text('wrong\n1\n');assert evaluate.evaluate(path,pair_commits=hw)['hwif']['result']=='FAIL'

@pytest.mark.parametrize('n',[2,3])
def test_tool_one_request_monotonic_heart_and_no_restart(n):
    s=command.Session(n);r=s.request(1,1234);assert r['axes']==n and r['nonce']!=0
    assert s.heartbeat(1235)['sequence']==1 and s.heartbeat(1236)['sequence']==2
    for state in range(8):
        with pytest.raises(ValueError):s.request(state,1300)
    assert command.outcome(7,True,0)==1
    assert command.outcome(5,True,1)==0
    assert command.outcome(5,True,2)==1
    assert command.outcome(3,False,0) is None

@pytest.mark.parametrize('args',[(1,20,1),(3,float('nan'),1),(3,21,1),(3,20,2)])
def test_tool_rejects_bad_profile(args):
    with pytest.raises(ValueError):command.Session(*args)

@pytest.mark.parametrize('ending,expected',[(5,0),(7,1),('interrupt',1),('reject',1)])
def test_tool_ros_adapter_never_retries_start_and_stops_once(monkeypatch,capsys,ending,expected):
    # Exercise the CLI adapter with generated message classes and a fake ROS transport.
    import rclpy
    import rclpy.qos
    from types import SimpleNamespace
    calls=[];hearts=[];callback=[]
    def status(state):return SimpleNamespace(state=state,terminal=state in (5,7),record_state=1 if state==5 else 0,csv_path='original.csv' if state==5 else '')
    class Future:
        def done(self):return True
        def result(self):return SimpleNamespace(accepted=ending!='reject',success=True,message='accepted' if ending!='reject' else 'rejected')
    class Client:
        def __init__(self,name):self.name=name
        def wait_for_service(self,timeout_sec):return True
        def service_is_ready(self):return True
        def call_async(self,request):calls.append((self.name,request));return Future()
    class Node:
        def create_subscription(self,typ,name,cb,qos):callback.append(cb);cb(status(1));return object()
        def create_publisher(self,*args):return SimpleNamespace(publish=lambda m:hearts.append(m))
        def create_client(self,typ,name):return Client(name.rsplit('/',1)[1])
        def destroy_node(self):pass
    def spin_once(node,timeout_sec):
        if ending=='interrupt':raise KeyboardInterrupt()
        callback[0](status(ending))
    monkeypatch.setattr(rclpy,'init',lambda:None);monkeypatch.setattr(rclpy,'shutdown',lambda:None)
    monkeypatch.setattr(rclpy,'ok',lambda:True);monkeypatch.setattr(rclpy,'create_node',lambda name:Node())
    monkeypatch.setattr(rclpy,'spin_once',spin_once);monkeypatch.setattr(rclpy,'spin_until_future_complete',lambda *a,**k:None)
    assert command.main(['start','--axes','3'])==expected
    assert sum(name=='start' for name,_ in calls)==1
    assert sum(name=='stop' for name,_ in calls)==int(expected==1)
    if ending in (5,7,'interrupt'):assert len(hearts)==1 and hearts[0].sequence==1
    output=capsys.readouterr().out
    if ending in (5,7):assert ('NORMAL' if ending==5 else 'ABNORMAL') in output
    if ending==7:assert 'ABORTED' in output

@pytest.mark.parametrize('n',[2,3])
def test_installed_generator_and_tools(tmp_path,n):
    from ament_index_python.packages import get_package_prefix
    prefix=Path(get_package_prefix('bxi_hardware'));program=prefix/'lib/bxi_hardware'
    result=subprocess.run([sys.executable,str(program/'make_n_axis_config.py'),'--axes',str(n),'--ids',*[str(i+1) for i in range(n)],'--destination',str(tmp_path/'installed'),'--output-dir',str(tmp_path)],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    for tool in ['n_axis_command.py','n_axis_evaluate.py','n_axis_check_config.py']:
        assert subprocess.run([sys.executable,str(program/tool),'--help'],capture_output=True).returncode==0

@pytest.mark.parametrize('mutation',['nonfinite','fractional_integer','missing_column','zero_amplitude','partial_run','bad_reason','failure_count','negative_threshold'])
def test_evaluator_does_not_hide_incomplete_or_invalid_data(tmp_path,mutation):
    f,r=synthetic();path=tmp_path/'original.csv'
    if mutation=='nonfinite':r[2]['position0']=float('nan')
    if mutation=='fractional_integer':r[2]['g']=2.5
    if mutation=='missing_column':
        f.remove('normal_commit_read_ns')
        for row in r:row.pop('normal_commit_read_ns')
    if mutation=='zero_amplitude':
        for row in r:
            for a in range(3):row[f'position{a}']=0
    if mutation=='partial_run':r=r[:2]
    if mutation=='bad_reason':r[-1]['reason']=0
    if mutation=='failure_count':r[-1]['failures']=1
    write_csv(path,f,r)
    result=evaluate.evaluate(path,phase_degrees=-1 if mutation=='negative_threshold' else 10)
    if mutation in ('zero_amplitude','partial_run'):assert result['measurement']['result']=='NOT_EVALUATED'
    elif mutation in ('bad_reason','failure_count'):assert result['lease_end']['result']=='FAIL'
    else:assert result['integrity']['result']=='FAIL'

@pytest.mark.parametrize('mutation',['n','flags','state','result'])
def test_invalid_schema_codes_never_pass(tmp_path,mutation):
    f,r=synthetic();p=tmp_path/'original.csv'
    if mutation=='n':r[0]['N']=4
    if mutation=='flags':r[3].update(advanced=2,held=-1,g=5)
    if mutation=='state':r[3]['after']=99
    if mutation=='result':r[3]['accepted_result']=99
    write_csv(p,f,r);assert evaluate.evaluate(p)['integrity']['result']=='FAIL'

def test_empty_event_file_and_missing_hwif_metadata(tmp_path):
    f,r=synthetic();p=tmp_path/'original.csv';write_csv(p,f,r)
    events=tmp_path/'safety.csv';events.write_text('#dropped=0\nepoch,N,sources,result,send_attempted,send_success\n')
    summary=evaluate.evaluate(p,pair_safety=events)['hwif']['files']['safety']
    assert summary['result']=='NOT_EVALUATED' and summary['rows']==0
    events.write_text('epoch,N,sources,result,send_attempted,send_success\n')
    assert evaluate.evaluate(p,pair_safety=events)['hwif']['result']=='FAIL'

@pytest.mark.parametrize('action',['stop','finish'])
def test_trigger_cli_displays_record_and_does_not_retry_stop(monkeypatch,capsys,action):
    import rclpy
    import rclpy.qos
    from types import SimpleNamespace
    calls=[]
    class Future:
        def done(self):return True
        def result(self):return SimpleNamespace(success=True,message=action+' requested')
    class Client:
        def __init__(self,name):self.name=name
        def wait_for_service(self,timeout_sec):return True
        def service_is_ready(self):return True
        def call_async(self,request):calls.append(self.name);return Future()
    class Node:
        def create_subscription(self,typ,name,cb,qos):cb(SimpleNamespace(state=4,record_state=0,csv_path=''));return object()
        def create_publisher(self,*args):return SimpleNamespace(publish=lambda m:None)
        def create_client(self,typ,name):return Client(name.rsplit('/',1)[1])
        def destroy_node(self):pass
    monkeypatch.setattr(rclpy,'init',lambda:None);monkeypatch.setattr(rclpy,'shutdown',lambda:None)
    monkeypatch.setattr(rclpy,'ok',lambda:True);monkeypatch.setattr(rclpy,'create_node',lambda name:Node())
    monkeypatch.setattr(rclpy,'spin_until_future_complete',lambda *a,**k:None)
    assert command.main([action])==0
    output=capsys.readouterr().out
    assert 'record_state=0' in output and 'csv=not saved' in output and action+' requested' in output
    assert calls==[action]
    if action=='stop':
        def failed(*args,**kwargs):raise RuntimeError('uncertain stop response')
        monkeypatch.setattr(rclpy,'spin_until_future_complete',failed)
        calls.clear();assert command.main(['stop'])==1;assert calls==['stop']


@pytest.mark.parametrize('tool', [
    'make_n_axis_config.py', 'n_axis_check_config.py',
    'n_axis_command.py', 'n_axis_evaluate.py',
])
def test_entry_scripts_are_executable_and_run_directly(tool):
    import os
    import stat
    from ament_index_python.packages import get_package_prefix

    source = HERE / tool
    installed = Path(get_package_prefix('bxi_hardware')) / 'lib/bxi_hardware' / tool
    for entry in (source, installed):
        real = entry.resolve(strict=True)
        assert real.is_file(), str(real)
        assert stat.S_IMODE(real.stat().st_mode) & 0o111 == 0o111, str(real)
        assert os.access(real, os.X_OK), str(real)
        result = subprocess.run([str(entry), '--help'], capture_output=True, text=True, timeout=10)
        assert result.returncode == 0, result.stdout + result.stderr
        assert 'usage:' in result.stdout.lower(), result.stdout


@pytest.fixture
def generated_bounds_files(tmp_path):
    def generate(n):
        d = make.generate(n, list(range(1, n + 1)), tmp_path / 'bounds_config', tmp_path)
        import xacro
        urdf = tmp_path / 'bounds.urdf'
        urdf.write_text(xacro.process_file(str(HERE / 'n_axis.urdf.xacro'), mappings={
            'axes': str(n), 'config_file': str(d / 'hardware.yaml'),
            'spec_dir': str(ROOT / 'config/specs')}).toxml())
        assert check.check(d, urdf) == []
        return d, urdf
    return generate


@pytest.mark.parametrize('n', [2, 3])
def test_generated_profile_has_all_5n_bounds(generated_bounds_files, n):
    d, urdf = generated_bounds_files(n)
    expected = {'position': [-math.pi / 6, math.pi / 6], 'velocity': [-.70, .70],
                'kp': [0., 20.], 'kd': [0., 1.], 'effort': [0., 0.]}
    assert make.profile_bounds() == expected
    hw = yaml.safe_load((d / 'hardware.yaml').read_text())
    joints = ET.parse(urdf).getroot().findall('ros2_control/joint')
    assert len(joints) == n
    for joint in joints:
        commands = joint.findall('command_interface')
        assert len(commands) == 5
        for command_interface in commands:
            values = [float(command_interface.findtext(f"param[@name='{key}']"))
                      for key in ('min', 'max')]
            assert values == expected[command_interface.get('name')]
        runtime = hw['joints'][joint.get('name')]
        assert [runtime['lower'], runtime['upper']] == expected['position']


@pytest.mark.parametrize('n', [2, 3])
@pytest.mark.parametrize('field', ['position', 'velocity', 'kp', 'kd', 'effort'])
@pytest.mark.parametrize('key', ['min', 'max'])
@pytest.mark.parametrize('mutation', ['missing', 'nonnumeric', 'nonfinite', 'mismatch', 'duplicate'])
def test_checker_rejects_bad_command_bounds(generated_bounds_files, n, field, key, mutation):
    d, urdf = generated_bounds_files(n)
    tree = ET.parse(urdf)
    # Include the final axis, so N=3 cannot accidentally validate only two joints.
    command_interface = tree.getroot().findall('ros2_control/joint')[-1].find(
        f"command_interface[@name='{field}']")
    param = command_interface.find(f"param[@name='{key}']")
    if mutation == 'missing':
        command_interface.remove(param)
    elif mutation == 'duplicate':
        ET.SubElement(command_interface, 'param', name=key).text = param.text
    else:
        param.text = {'nonnumeric': 'invalid', 'nonfinite': 'nan',
                      'mismatch': str(float(param.text) + .01)}[mutation]
    tree.write(urdf)
    assert check.check(d, urdf)


@pytest.mark.parametrize('n', [2, 3])
@pytest.mark.parametrize('key', ['lower', 'upper'])
def test_checker_rejects_hwif_position_mismatch(generated_bounds_files, n, key):
    d, urdf = generated_bounds_files(n)
    path = d / 'hardware.yaml'
    hw = yaml.safe_load(path.read_text())
    hw['joints'][f'joint{n}'][key] += .01
    path.write_text(yaml.safe_dump(hw))
    assert check.check(d, urdf)


@pytest.mark.parametrize('n', [2, 3])
def test_checker_rejects_nonzero_effort_upper_separately(generated_bounds_files, n):
    d, urdf = generated_bounds_files(n)
    tree = ET.parse(urdf)
    for joint in tree.getroot().findall('ros2_control/joint'):
        joint.find("command_interface[@name='effort']/param[@name='max']").text = '1'
    tree.write(urdf)
    errors = check.check(d, urdf)
    assert sum('/effort: G bounds mismatch' in error for error in errors) == n
    # This is the strict G declaration check, not an on_init rejection requirement.
