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
        profile['initial_positions'][joint] = profile['joint_settings'][joint]['position_max'] + .01
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


VCAN_PROFILE = PROFILE.with_name('bxi_champ_jtc_vcan_legs.yaml')


@pytest.mark.parametrize('legs', [['FR'], ['FR', 'FL']])
def test_stage3_valid_vcan_variants(tmp_path, legs):
    generator.generate(VCAN_PROFILE, tmp_path, legs)
    assert checker.check_adapter(tmp_path, tmp_path / 'robot.urdf') == []
    assert checker.check_adapter(tmp_path, tmp_path / 'robot.urdf', real_configuration=True)


@pytest.mark.parametrize('mutation', ['profile', 'calibration', 'age_missing', 'age_zero', 'feedback', 'trigger', 'rx', 'broadcast', 'grace', 'first_timeout', 'bus', 'direction', 'offset'])
def test_stage3_rejects_unsafe_configuration(tmp_path, mutation):
    generator.generate(VCAN_PROFILE, tmp_path)
    profile_path = tmp_path / 'bxi_champ_jtc_legs.yaml'
    hw_path = tmp_path / 'hardware.yaml'
    profile = yaml.safe_load(profile_path.read_text())
    hw = yaml.safe_load(hw_path.read_text())
    transport = hw['can_transport']['can0']
    if mutation == 'profile':
        profile['profile'] = 'stage2_mock_provisional'
        profile['real_legs'] = []
    elif mutation == 'calibration':
        profile['vcan_provisional_calibration']['motor_dir'] = -1
    elif mutation == 'age_missing':
        del transport['stream_command_max_age_ms']
    elif mutation == 'age_zero':
        transport['stream_command_max_age_ms'] = 0.0
    elif mutation == 'feedback':
        transport['stream_feedback_max_age_ms'] = 4.0
    elif mutation == 'trigger':
        transport['tx_trigger'] = 'timer'
    elif mutation == 'rx':
        transport['rx_mode'] = 'sequential'
    elif mutation == 'broadcast':
        transport['tx_frame'] = 'unicast'
    elif mutation == 'grace':
        transport['stream_write_grace_ms'] = 1.0
    elif mutation == 'first_timeout':
        transport['stream_first_write_timeout_ms'] = 2000.0
    elif mutation == 'bus':
        hw['joints']['FR_collar_joint']['can_bus'] = 'can1'
    elif mutation == 'direction':
        hw['joints']['FR_collar_joint']['motor_dir'] = -1
    elif mutation == 'offset':
        hw['joints']['FR_collar_joint']['offset_angle'] = 0.1
    profile_path.write_text(yaml.safe_dump(profile))
    hw_path.write_text(yaml.safe_dump(hw))
    assert checker.check_adapter(tmp_path, tmp_path / 'robot.urdf')


def test_generator_rejects_vcan_calibration_outside_stage3(tmp_path):
    profile = yaml.safe_load(PROFILE.read_text())
    profile['vcan_provisional_calibration'] = {'motor_dir': 1, 'offset_angle': 0.0}
    # Keep the configuration under the package so relative model paths stay valid.
    source = tmp_path / 'invalid-profile.yaml'
    source.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match='calibration requires Stage 3'):
        generator.generate(source, tmp_path / 'output')


def test_rejects_measured_interpolation_start(tmp_path):
    generator.generate(PROFILE, tmp_path)
    path = tmp_path / 'controllers.yaml'
    controllers = yaml.safe_load(path.read_text())
    trajectory = controllers['joint_trajectory_controller']['ros__parameters']
    assert trajectory['interpolate_from_desired_state'] is True
    trajectory['interpolate_from_desired_state'] = False
    path.write_text(yaml.safe_dump(controllers))
    errors = checker.check_adapter(tmp_path, tmp_path / 'robot.urdf')
    assert any('interpolate_from_desired_state must be true' in error for error in errors)


@pytest.mark.parametrize('upstream', ['champ', 'rl'])
@pytest.mark.parametrize('mutation', ['copy_motor_limit', 'mismatch'])
def test_rejects_measured_start_speed_mismatch(tmp_path, upstream, mutation):
    profile = generator.generate(PROFILE, tmp_path)
    # Read the generated profile; the upstream interpolation limit stays independent.
    profile = yaml.safe_load((tmp_path / 'bxi_champ_jtc_legs.yaml').read_text())
    path = tmp_path / (upstream + '.yaml')
    configuration = yaml.safe_load(path.read_text())
    parameters = configuration['/**']['ros__parameters']
    expected = [profile['measured_start_velocity_limit']] * 12
    actual = parameters['measured_start']['velocity_limit'] if upstream == 'champ' else parameters['measured_start_velocity_limit']
    assert actual == expected == [.7] * 12
    assert all(profile['joint_settings'][name]['velocity_limit'] > .7 for name in profile['joints'])
    replacement = [profile['joint_settings'][name]['velocity_limit'] for name in profile['joints']] if mutation == 'copy_motor_limit' else [1.] * 12
    if upstream == 'champ':
        parameters['measured_start']['velocity_limit'] = replacement
    else:
        parameters['measured_start_velocity_limit'] = replacement
    path.write_text(yaml.safe_dump(configuration))
    assert any('measured-start velocity must match profile' in error for error in checker.check_adapter(tmp_path, tmp_path / 'robot.urdf'))


def test_rejects_measured_start_speed_above_half_motor_limit(tmp_path):
    generator.generate(PROFILE, tmp_path)
    path = tmp_path / 'bxi_champ_jtc_legs.yaml'
    profile = yaml.safe_load(path.read_text())
    profile['measured_start_velocity_limit'] = 6.
    path.write_text(yaml.safe_dump(profile))
    for upstream in ['champ', 'rl']:
        path = tmp_path / (upstream + '.yaml')
        configuration = yaml.safe_load(path.read_text())
        parameters = configuration['/**']['ros__parameters']
        if upstream == 'champ':
            parameters['measured_start']['velocity_limit'] = [6.] * 12
        else:
            parameters['measured_start_velocity_limit'] = [6.] * 12
        path.write_text(yaml.safe_dump(configuration))
    assert any('at most half' in error for error in checker.check_adapter(tmp_path, tmp_path / 'robot.urdf'))


STAGE56 = [
    ('real_zero_gain', True, (0., 0.)),
    ('vcan_zero_gain', False, (0., 0.)),
    ('real_gain20', True, (20., 1.)),
    ('vcan_gain20', False, (20., 1.)),
]


@pytest.mark.parametrize('suffix,real,gains', STAGE56)
def test_stage56_generation_and_configuration_mode(tmp_path, suffix, real, gains):
    source = PROFILE.with_name('bxi_champ_jtc_' + suffix + '_legs.yaml')
    original = yaml.safe_load(source.read_text())
    baseline = yaml.safe_load(VCAN_PROFILE.read_text())
    # Only profile name and all joint gains may differ from Stage 3.
    baseline['profile'] = original['profile']
    for settings in baseline['joint_settings'].values():
        settings.update(kp=gains[0], kd=gains[1], kp_max=gains[0], kd_max=gains[1])
    assert original == baseline
    generator.generate(source, tmp_path)
    assert checker.check_adapter(tmp_path, tmp_path / 'robot.urdf', real_configuration=real) == []
    assert checker.check_adapter(tmp_path, tmp_path / 'robot.urdf', real_configuration=not real)
    note = (tmp_path / 'hardware.yaml').read_text().splitlines()[0]
    assert note.startswith('# REAL:' if real else '# VCAN ONLY:')


@pytest.mark.parametrize('gain', ['zero_gain', 'gain20'])
def test_stage56_real_vcan_structural_equivalence(tmp_path, gain):
    directories = [tmp_path / mode for mode in ('real', 'vcan')]
    for mode, directory in zip(('real', 'vcan'), directories):
        generator.generate(PROFILE.with_name(f'bxi_champ_jtc_{mode}_{gain}_legs.yaml'), directory)
    assert {f.name for f in directories[0].iterdir()} == {f.name for f in directories[1].iterdir()}
    def normalize(value, directory):
        if isinstance(value, str):
            return value.replace(str(directory), '@RUN@')
        if isinstance(value, dict):
            return {key: normalize(item, directory) for key, item in value.items()}
        if isinstance(value, list):
            return [normalize(item, directory) for item in value]
        return value

    for path in directories[0].iterdir():
        left = path.read_text()
        right = (directories[1] / path.name).read_text()
        if path.suffix == '.yaml':
            left = normalize(yaml.safe_load(left), directories[0])
            right = normalize(yaml.safe_load(right), directories[1])
            if path.name == 'bxi_champ_jtc_legs.yaml':
                left['profile'] = right['profile'] = '@PROFILE@'
        else:
            left = left.replace(str(directories[0]), '@RUN@')
            right = right.replace(str(directories[1]), '@RUN@')
            left, right = ET.tostring(ET.fromstring(left)), ET.tostring(ET.fromstring(right))
        assert left == right, path.name


@pytest.mark.parametrize('suffix,real,gains', STAGE56)
def test_stage56_rejects_changed_declared_gains(tmp_path, suffix, real, gains):
    generator.generate(PROFILE.with_name('bxi_champ_jtc_' + suffix + '_legs.yaml'), tmp_path)
    path = tmp_path / 'bxi_champ_jtc_legs.yaml'
    profile = yaml.safe_load(path.read_text())
    profile['joint_settings']['FR_collar_joint']['kp'] += 1.
    path.write_text(yaml.safe_dump(profile))
    assert any('stage5/6 gains mismatch' in e for e in checker.check_adapter(
        tmp_path, tmp_path / 'robot.urdf', real_configuration=real))


STARTUP = load('stage5_startup', WORKSPACE / 'src/minibexkai_bringup/scripts/start_bxi_champ_jtc.py')
VCAN_LAUNCH = load('stage5_vcan_launch', WORKSPACE / 'src/minibexkai_bringup/launch/bxi_champ_jtc_vcan.launch.py')
REAL_LAUNCH = load('stage5_real_launch', WORKSPACE / 'src/minibexkai_bringup/launch/bxi_champ_jtc_real.launch.py')


def real_can_link():
    return [{'flags': ['UP'], 'linkinfo': {'info_kind': 'can', 'info_data': {
        'state': 'ERROR-ACTIVE', 'bittiming': {'bitrate': 1000000},
        'data_bittiming': {'bitrate': 5000000}, 'ctrlmode': ['FD']}}}]


@pytest.mark.parametrize('entry', ['startup', 'launch'])
@pytest.mark.parametrize('mutation', ['missing', 'vcan', 'ether', 'down', 'passive', 'busoff', 'bitrate', 'data_bitrate', 'no_fd'])
def test_stage5_real_guards_reject_before_any_start(tmp_path, monkeypatch, entry, mutation):
    import json
    import subprocess
    link = real_can_link()
    data = link[0]['linkinfo']['info_data']
    if mutation == 'missing':
        link = []
    elif mutation in ('vcan', 'ether'):
        link[0]['linkinfo']['info_kind'] = mutation
    elif mutation == 'down':
        link[0]['flags'] = []
    elif mutation in ('passive', 'busoff'):
        data['state'] = 'ERROR-PASSIVE' if mutation == 'passive' else 'BUS-OFF'
    elif mutation == 'bitrate':
        data['bittiming']['bitrate'] = 500000
    elif mutation == 'data_bitrate':
        data['data_bittiming']['bitrate'] = 2000000
    elif mutation == 'no_fd':
        data['ctrlmode'] = []
    monkeypatch.setattr(subprocess, 'check_output', lambda *args, **kwargs: json.dumps(link))
    monkeypatch.setattr(STARTUP.rclpy, 'init', lambda: pytest.fail('ROS started before guard'))
    if entry == 'startup':
        directory = tmp_path / 'config'
        generator.generate(PROFILE.with_name('bxi_champ_jtc_real_zero_gain_legs.yaml'), directory)
        with pytest.raises(ValueError, match='requires'):
            STARTUP.run(directory)
        assert not (directory / 'startup.jsonl').exists()
    else:
        # setup must reject before reading launch arguments or making any actions.
        with pytest.raises(ValueError, match='requires'):
            REAL_LAUNCH.setup(None)


@pytest.mark.parametrize('real', [True, False])
def test_stage5_valid_link_guard(monkeypatch, real):
    import json
    import subprocess
    link = real_can_link() if real else [{'linkinfo': {'info_kind': 'vcan'}}]
    monkeypatch.setattr(subprocess, 'check_output', lambda *args, **kwargs: json.dumps(link))
    STARTUP.validate_can_link(real)


def test_stage5_vcan_guard_rejects_real_can(monkeypatch):
    import json
    import subprocess
    monkeypatch.setattr(subprocess, 'check_output', lambda *args, **kwargs: json.dumps(real_can_link()))
    with pytest.raises(ValueError, match='kind vcan'):
        VCAN_LAUNCH.setup(None)
