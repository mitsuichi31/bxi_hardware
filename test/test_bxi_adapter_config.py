"""Static configuration tests. No controller manager, CAN or device access."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parent.parent


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


checker = load('adapter_checker', ROOT / 'hil/n_axis/n_axis_check_config.py')
generator = load('adapter_generator', WORKSPACE / 'src/minibexkai_bringup/scripts/make_bxi_champ_jtc_config.py')
PROFILE = WORKSPACE / 'src/minibexkai_bringup/config/bxi_champ_jtc_legs.yaml'


@pytest.mark.parametrize('legs', [[], ['FR'], ['BL', 'BR', 'FL', 'FR']])
def test_valid_variants(tmp_path, legs):
    generator.generate(PROFILE, tmp_path, legs)
    assert checker.check_adapter(tmp_path, tmp_path / 'robot.urdf') == []


@pytest.mark.parametrize(
    'mutation',
    [
        'real',
        'extra_controller',
        'joint_order',
        'command_index',
        'timeout_zero',
        'timeout_equal',
        'timeout_less',
        'reuse_command',
        'async',
        'gpio_type',
        'gpio_transform',
        'command_missing',
        'command_extra',
        'state_duplicate',
        'hardware_plugin',
        'position_bounds',
        'initial_outside',
        'gain',
        'bus',
        'id',
        'serial',
        'publisher',
        'extra_adapter_joint',
        'nonfinite'
    ]
)
def test_rejects_mutations(tmp_path, mutation):
    generator.generate(PROFILE, tmp_path)
    config_path = tmp_path / 'controllers.yaml'
    profile_path = tmp_path / 'bxi_champ_jtc_legs.yaml'
    urdf = tmp_path / 'robot.urdf'
    controllers = yaml.safe_load(config_path.read_text())
    profile = yaml.safe_load(profile_path.read_text())
    tree = ET.parse(urdf)
    system = tree.getroot().find("ros2_control[@name='bxi_FR']")
    trajectory = controllers['joint_trajectory_controller']['ros__parameters']
    adapter = controllers['bxi_pair_adapter']['ros__parameters']
    joint = 'FR_collar_joint'
    if mutation == 'real':
        profile['real_legs'] = ['FR']
    elif mutation == 'extra_controller':
        controllers['controller_manager']['ros__parameters']['forward'] = {'type': 'forward_command_controller/ForwardCommandController'}
    elif mutation == 'joint_order':
        trajectory['joints'] = list(reversed(trajectory['joints']))
    elif mutation == 'command_index':
        trajectory['command_joints'][0], trajectory['command_joints'][1] = trajectory['command_joints'][1], trajectory['command_joints'][0]
    elif mutation.startswith('timeout_'):
        trajectory['cmd_timeout'] = {'timeout_zero': 0., 'timeout_equal': .1, 'timeout_less': .05}[mutation]
        trajectory['constraints']['goal_time'] = .1
    elif mutation == 'reuse_command':
        trajectory['set_last_command_interface_value_as_state_on_activation'] = True
    elif mutation == 'async':
        adapter['is_async'] = True
    elif mutation == 'gpio_type':
        system.find('gpio/command_interface').set('data_type', 'uint64')
    elif mutation == 'gpio_transform':
        ET.SubElement(system.find('gpio/command_interface'), 'param', name='max').text = '20'
    elif mutation == 'command_missing':
        system.find('joint').remove(system.find('joint/command_interface'))
    elif mutation == 'command_extra':
        ET.SubElement(system.find('joint'), 'command_interface', name='other')
    elif mutation == 'state_duplicate':
        system.find('joint/state_interface').set('name', 'temperature')
    elif mutation == 'hardware_plugin':
        system.find('hardware/plugin').text = 'bxi_hardware/BxiSystemInterface'
    elif mutation == 'position_bounds':
        system.find("joint/command_interface/param[@name='min']").text = '-1'
    elif mutation == 'initial_outside':
        profile['initial_positions'][joint] = .31
    elif mutation == 'gain':
        adapter['joint_settings'][joint]['kp'] = 11.
    elif mutation == 'bus':
        profile['legs']['FR']['bus'] = 'can1'
    elif mutation == 'id':
        profile['legs']['FR']['ids'] = [1, 1, 3]
    elif mutation == 'serial':
        profile['legs']['FR']['expected_serials'].reverse()
    elif mutation == 'publisher':
        profile['publisher_count'] = 2
    elif mutation == 'extra_adapter_joint':
        adapter['joint_settings']['unexpected'] = adapter['joint_settings'][joint]
    elif mutation == 'nonfinite':
        profile['joint_settings'][joint]['velocity_limit'] = float('nan')
    config_path.write_text(yaml.safe_dump(controllers))
    profile_path.write_text(yaml.safe_dump(profile))
    tree.write(urdf)
    assert checker.check_adapter(tmp_path, urdf)


def test_real_configuration_rejects_test_only_plugin(tmp_path):
    generator.generate(PROFILE, tmp_path)
    assert checker.check_adapter(tmp_path, tmp_path / 'robot.urdf', real_configuration=True)


@pytest.mark.parametrize('legs', [['FL'], ['FR', 'FR'], ['unknown']])
def test_generator_rejects_unsupported_legs(tmp_path, legs):
    with pytest.raises(ValueError):
        generator.generate(PROFILE, tmp_path, legs)


def test_locomotion_profiles_follow_names_and_provisional_values(tmp_path):
    generator.generate(PROFILE, tmp_path)
    profile = yaml.safe_load((tmp_path / 'bxi_champ_jtc_legs.yaml').read_text())
    champ = yaml.safe_load((tmp_path / 'champ.yaml').read_text())['/**']['ros__parameters']
    rl = yaml.safe_load((tmp_path / 'rl.yaml').read_text())['/**']['ros__parameters']
    champ_names = [
        name for leg in ('left_front', 'right_front', 'left_hind', 'right_hind') for name in champ['joints_map'][leg]
    ]
    assert champ['measured_start_enabled'] and rl['measured_start_enabled']
    assert champ['initial_state'] == 'STANDBY' and champ['enable_state_machine']
    assert rl['auto_walk'] is False
    assert rl['joint_names'] == profile['joints']
    for names, down, up in [
        (champ_names, champ['standby_joint_angles'], champ['default_joint_angles']),
        (rl['joint_names'], rl['standby_angle'], rl['default_angle'])
    ]:
        assert dict(zip(names, down)) == profile['standby_positions']
        assert dict(zip(names, up)) == profile['standup_positions']
    assert champ['measured_start']['velocity_limit'] == [.7] * 12
    assert rl['measured_start_velocity_limit'] == [.7] * 12
    assert champ['measured_start']['max_age'] == rl['measured_start_max_age'] == .2
    assert champ['measured_start']['resync_tolerance'] == rl['measured_start_resync_tolerance'] == .05
    assert rl['policy_path'] == '/ws/log/bxi_jtc_adapter_stage2_20261019/policy_v3_seed2/seed2_policy.pt'

    zero = {name: 0. for name in profile['joints']}
    generated = tmp_path / 'zero'
    generator.generate(PROFILE, generated, initial_positions=zero)
    assert yaml.safe_load((generated / 'bxi_champ_jtc_legs.yaml').read_text())['initial_positions'] == zero
    assert checker.check_adapter(generated, generated / 'robot.urdf') == []


@pytest.mark.parametrize('upstream', ['champ', 'rl'])
@pytest.mark.parametrize(
    'field,value',
    [
        ('controller_state_topic', '/wrong'),
        ('required_samples', 4),
        ('max_time_from_start', .011),
        ('timeout', 11.)
    ]
)
def test_rejects_jtc_ingestion_setting_changes(tmp_path, upstream, field, value):
    generator.generate(PROFILE, tmp_path)
    path = tmp_path / (upstream + '.yaml')
    config = yaml.safe_load(path.read_text())
    config['/**']['ros__parameters']['jtc_ingestion'][field] = value
    path.write_text(yaml.safe_dump(config))
    assert any(
        'ingestion readiness settings mismatch' in error
        for error in checker.check_adapter(tmp_path, tmp_path / 'robot.urdf')
    )
