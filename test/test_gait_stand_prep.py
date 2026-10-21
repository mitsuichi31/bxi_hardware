"""Collar option B: source-based geometry, generation and independent checking."""
import copy
import importlib.util
from pathlib import Path
import pytest
import yaml

W = Path(__file__).resolve().parents[3]
B = W / 'src/minibexkai_bringup'

def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

ref = load('gait_reference_test', B/'scripts/champ_gait_reference.py').gait_reference
generator = load('gait_stand_generator', B/'scripts/make_bxi_champ_jtc_config.py')
checker = load('gait_stand_checker', W/'src/bxi_hardware/hil/n_axis/n_axis_check_config.py')
SOURCE = B/'config/bxi_champ_jtc_vcan_gait_stand_legs.yaml'
FIXTURE = W/'log/bxi_champ_jtc_stage8v_20261021/supplement4/motion/normal/config'

def inputs():
    champ=yaml.safe_load((FIXTURE/'champ.yaml').read_text())['/**']['ros__parameters']
    profile=yaml.safe_load((FIXTURE/'bxi_champ_jtc_legs.yaml').read_text())
    return champ,profile

def test_recorded_float_reference():
    champ,profile=inputs()
    result=ref(FIXTURE/'robot.urdf',champ['gait'],champ['joints_map'],profile['joint_settings'])
    for n,v in result.items():
        expected=(-.07736195623874664 if n[:2] in ('FL','BL') else .07736198604106903) if 'collar' in n else (.9682347178459167 if 'hip' in n else -1.5839834213256836)
        assert abs(v-expected)<=1e-6

@pytest.mark.parametrize('kind',['orientation','unreachable','bounds'])
def test_reference_rejects(kind):
    champ,profile=inputs()
    if kind=='orientation':champ['gait']['knee_orientation']='<<'
    if kind=='unreachable':champ['gait']['com_x_translation']=100.
    if kind=='bounds':profile['joint_settings']['FR_collar_joint']['position_max']=0.
    with pytest.raises(ValueError):ref(FIXTURE/'robot.urdf',champ['gait'],champ['joints_map'],profile['joint_settings'])

def test_generated_reference(tmp_path):
    generator.generate(SOURCE,tmp_path)
    p=yaml.safe_load((tmp_path/'bxi_champ_jtc_legs.yaml').read_text());c=yaml.safe_load((tmp_path/'champ.yaml').read_text())['/**']['ros__parameters']
    expected=ref(tmp_path/'robot.urdf',c['gait'],c['joints_map'],p['joint_settings'])
    assert p['standup_from_gait'] is True and p['standup_positions']==expected
    names=[n for leg in ('left_front','right_front','left_hind','right_hind') for n in c['joints_map'][leg]]
    assert c['default_joint_angles']==[expected[n] for n in names]
    assert checker.check_adapter(tmp_path,tmp_path/'robot.urdf')==[]

@pytest.mark.parametrize('kind',['other_profile','explicit_pose','false_flag'])
def test_generator_rejects(tmp_path,kind):
    profile=yaml.safe_load(SOURCE.read_text())
    if kind=='other_profile':profile['profile']='stage8_vcan_walk'
    if kind=='explicit_pose':profile['standup_positions']={}
    if kind=='false_flag':profile['standup_from_gait']=False
    source=tmp_path/'bad.yaml';source.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError,match='standup_from_gait'):generator.generate(source,tmp_path/'out')

@pytest.mark.parametrize('field',['default_joint_angles','standup_positions'])
@pytest.mark.parametrize('shift,accepted',[(.0009,True),(.0011,False)])
def test_checker_tolerance(tmp_path,field,shift,accepted):
    generator.generate(SOURCE,tmp_path)
    f=tmp_path/('champ.yaml' if field=='default_joint_angles' else 'bxi_champ_jtc_legs.yaml');d=yaml.safe_load(f.read_text())
    if field=='default_joint_angles':d['/**']['ros__parameters'][field][0]+=shift
    else:d[field]['FL_collar_joint']+=shift
    f.write_text(yaml.safe_dump(d))
    assert (checker.check_adapter(tmp_path,tmp_path/'robot.urdf')==[]) is accepted
