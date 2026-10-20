#!/usr/bin/env python3
"""Small consistency check; cannot detect arbitrary plugins, ownership or physical wiring."""
import argparse
import importlib.util
import math
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

LIMITATION = '任意の他plugin・所有者・実配線は検知できず、完全な構成検査の代替ではありません。'
COMMANDS = {'position', 'velocity', 'kp', 'kd', 'effort'}

def profile_bounds():
    # Resolve both source and installed copies without relying on sys.path.
    path = Path(__file__).resolve().with_name('make_n_axis_config.py')
    spec = importlib.util.spec_from_file_location('bxi_n_axis_generator_bounds', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.profile_bounds()


def check(directory, urdf):
    errors = []
    try:
        d = Path(directory)
        m = json.loads((d / 'manifest.json').read_text())
        hw = yaml.safe_load((d / 'hardware.yaml').read_text())
        ctrl = yaml.safe_load((d / 'controllers.yaml').read_text())
        tree = ET.parse(urdf).getroot()
        bounds = profile_bounds()
        n = m['axes'];
        names = [f'joint{i+1}' for i in range(n)]
        if n not in (2, 3) or m.get('source') != 'explicit_cli_arguments':
            errors.append('N or explicit-ID provenance missing')
        if list(hw['joints']) != names or [j.get('can_id') for j in hw['joints'].values()] != m['explicit_ids']:
            errors.append('joint order or explicit IDs mismatch')
        if m['explicit_ids'] != list(range(1, n + 1)) or len(set(m['explicit_ids'])) != n:
            errors.append('IDs must be distinct explicit 1..N for broadcast ABI')
        if not all(j.get('enabled') is True and j.get('can_bus') == m['can_bus'] for j in hw['joints'].values()):
            errors.append('disabled joint or wrong bus')
        systems = tree.findall('ros2_control')
        if len(systems) != 1:
            errors.append('exactly one ros2_control required')
            return errors
        system = systems[0]
        if system.findtext('hardware/plugin') != 'bxi_hardware/BxiSystemInterface':
            errors.append('unexpected HWIF plugin')
        params = {x.get('name'): x.text for x in system.findall('hardware/param')}
        if params.get('command_commit_mode') != 'pair_generation_v1':
            errors.append('wrong commit mode')
        if Path(params.get('config_file', '')).resolve() != (d / 'hardware.yaml').resolve():
            errors.append('URDF config_file mismatch')
        joints = system.findall('joint')
        if [j.get('name') for j in joints] != names:
            errors.append('URDF joint order mismatch')
        for j in joints:
            commands = j.findall('command_interface');
            states = j.findall('state_interface')
            if len(commands) != 5 or {x.get('name') for x in commands} != COMMANDS:
                errors.append('command set mismatch')
            for command in commands:
                field = command.get('name')
                if field not in bounds:
                    continue
                values = []
                for key in ('min', 'max'):
                    entries = command.findall(f"param[@name='{key}']")
                    if len(entries) != 1:
                        errors.append(f"{j.get('name')}/{field}: one {key} required")
                        break
                    try:
                        value = float(entries[0].text)
                    except (ValueError, TypeError):
                        errors.append(f"{j.get('name')}/{field}: nonnumeric {key}")
                        break
                    values.append(value)
                if len(values) == 2 and (not all(math.isfinite(v) for v in values) or values != bounds[field]):
                    errors.append(f"{j.get('name')}/{field}: G bounds mismatch")
            runtime = hw['joints'].get(j.get('name'), {})
            if [runtime.get('lower'), runtime.get('upper')] != bounds['position']:
                errors.append(f"{j.get('name')}: HWIF position bounds mismatch")
            if len(states) != 4 or {x.get('name') for x in states} != {'position', 'velocity', 'effort', 'temperature'}:
                errors.append('state set mismatch (3 controller states plus temperature)')
        gpios = system.findall('gpio')
        if len(gpios) != 1 or gpios[0].get('name') != 'bxi_pair':
            errors.append('one bxi_pair gpio required')
        else:
            gpio = gpios[0]
            expected = [('command_interface', 'commit_generation'), ('state_interface', 'accepted_generation')]
            if len(list(gpio)) != 2:
                errors.append('extra gpio content')
            for tag, name in expected:
                item = gpio.findall(tag)
                if len(item) != 1 or item[0].attrib != {'name': name, 'data_type': 'double'} or list(item[0]):
                    errors.append('gpio type/limiter/min/max/transform or interface mismatch')
        cm = ctrl['controller_manager']['ros__parameters']
        types = {k: v.get('type') for k, v in cm.items() if isinstance(v, dict)}
        if types != {
            'joint_state_broadcaster': 'joint_state_broadcaster/JointStateBroadcaster',
            'n_axis': 'bxi_hardware/NAxisTrajectoryController'
        }:
            errors.append('unexpected configured controller')
        if set(ctrl) != {'controller_manager', 'n_axis'} or ctrl['n_axis']['ros__parameters']['joints'] != names:
            errors.append('extra controller config or wrong controller joints')
        if ctrl['n_axis']['ros__parameters']['output_dir'] != m['output_dir'] or cm.get('update_rate') != 1000:
            errors.append('output_dir or update rate mismatch')
    except (ValueError, OSError, KeyError, TypeError, ET.ParseError) as e:
        errors.append(str(e))
    return errors


def check_vcan_hardware(directory, profile, root, require):
    """Check the isolated-only FR transport without opening any interface."""
    require(profile.get('vcan_provisional_calibration') == {'motor_dir': 1, 'offset_angle': 0.0}, 'explicit vcan-only calibration required')
    template = Path(__file__).resolve().parent / 'hardware.yaml.in'
    expected = yaml.safe_load(template.read_text().replace('@CAN_BUS@', 'can0'))
    hw = yaml.safe_load((directory / 'hardware.yaml').read_text())
    require(hw.get('active_can_buses') == ['can0'] and hw.get('can_frame_format') == expected['can_frame_format'], 'Stage 3 bus/frame mismatch')
    require(hw.get('can_transport') == expected['can_transport'], 'Stage 3 transport must explicitly equal G4 including watchdog ages')
    require(hw.get('defaults') == expected['defaults'], 'Stage 3 hardware defaults differ from G4')
    names = profile['legs']['FR']['joints']
    require(set(hw['joints']) == set(names), 'Stage 3 hardware joint set mismatch')
    for index, name in enumerate(names, 1):
        limits = profile['joint_settings'][name]
        require(hw['joints'][name] == dict(enabled=True, can_bus='can0', can_id=index, motor_type='MOTOR_70', motor_dir=1, offset_angle=0.0, lower=limits['position_min'], upper=limits['position_max'], default_kp=0.0, default_kd=0.0), name + ': vcan hardware mapping mismatch')
    system = root.find("ros2_control[@name='" + profile['legs']['FR']['hardware_name'] + "']")
    params = {x.get('name'): x.text for x in system.findall('hardware/param')}
    require(Path(params.get('config_file', '')).resolve() == (directory / 'hardware.yaml').resolve() and params.get('command_commit_mode') == 'pair_generation_v1', 'Stage 3 HWIF configuration/commit mismatch')


def check_adapter(directory, urdf, *, real_configuration=False):
    """Check the provisional mock profile; runtime ownership/wiring remain unproven."""
    errors = []
    def require(condition, message):
        if not condition:
            errors.append(message)
    try:
        directory = Path(directory)
        profile = yaml.safe_load((directory / 'bxi_champ_jtc_legs.yaml').read_text())
        controllers = yaml.safe_load((directory / 'controllers.yaml').read_text())
        root = ET.parse(urdf).getroot()
        real_profiles = ('stage5_real_zero_gain', 'stage6_real_gain20', 'stage8_real_walk')
        vcan_profiles = ('stage3_vcan_provisional', 'stage5_vcan_zero_gain', 'stage6_vcan_gain20', 'stage8_vcan_walk')
        stage3 = profile.get('profile') in real_profiles + vcan_profiles
        require(profile.get('profile') in (real_profiles if real_configuration else ('stage2_mock_provisional',) + vcan_profiles), 'profile forbidden for requested real_configuration')
        require(profile.get('real_legs') == (['FR'] if stage3 else []), 'provisional real_legs mismatch')
        require(stage3 or 'vcan_provisional_calibration' not in profile, 'vcan provisional calibration requires Stage 3')
        if stage3:
            check_vcan_hardware(directory, profile, root, require)
        if profile.get('profile') in real_profiles + vcan_profiles[1:]:
            gains = (0., 0.) if profile['profile'].startswith('stage5_') else (20., 1.)
            require(all((v['kp'], v['kd'], v['kp_max'], v['kd_max']) == (*gains, *gains)
                        for v in profile['joint_settings'].values()), 'stage5/6 gains mismatch')
        walk = profile.get('profile') in ('stage8_real_walk', 'stage8_vcan_walk')
        gait = profile.get('champ_gait', {})
        height = gait.get('swing_height') if isinstance(gait, dict) else None
        if walk:
            require(isinstance(gait, dict) and set(gait) == {'swing_height'} and
                    isinstance(height, (int, float)) and not isinstance(height, bool) and
                    math.isfinite(height) and 0 < height <= .05, 'invalid stage8 swing_height')
            require(all(v['torque_limit'] == 7.5 and v['torque_limit'] <= 15
                        for v in profile['joint_settings'].values()), 'stage8 torque_limit must be 7.5 <= rated 15')
        else:
            require('champ_gait' not in profile, 'champ_gait is stage8 only')
        champ = yaml.safe_load((directory / 'champ.yaml').read_text())['/**']['ros__parameters']
        require(champ.get('measured_start', {}).get('walk_rate_limit') == 2.0, 'Champ walk_rate_limit must be 2.0')
        if walk:
            require(champ['gait']['swing_height'] == height, 'Champ swing_height differs from profile')
        names = [
            f'{leg}_{kind}_joint' for leg in ('BL', 'BR', 'FL', 'FR')
            for kind in ('collar', 'hip', 'knee')
        ]
        require(profile['joints'] == names, '12-joint order mismatch')
        adapted = profile['adapter_legs']
        require(adapted in ((['FR'], ['FR', 'FL']) if stage3 else (['FR'], ['BL', 'BR', 'FL', 'FR'], [])), 'unsupported adapter legs')
        require(
            (profile.get('publisher') in ('champ', 'rl') or
             (profile.get('publisher') == 'manual' and profile.get('profile') in
              ('stage6_real_gain20', 'stage6_vcan_gain20'))) and profile.get('publisher_count') == 1,
            'exactly one upstream publisher required'
        )
        cm = controllers['controller_manager']['ros__parameters']
        types = {name: value.get('type') for name, value in cm.items() if isinstance(value, dict)}
        expected_types = {
            'joint_state_broadcaster': 'joint_state_broadcaster/JointStateBroadcaster',
            'joint_trajectory_controller': 'joint_trajectory_controller/JointTrajectoryController'
        }
        if adapted:
            expected_types['bxi_pair_adapter'] = 'bxi_hardware/BxiPairAdapter'
        require(
            types == expected_types and cm.get('update_rate') == 1000,
            'unexpected controller, owner or update rate'
        )
        require(set(controllers) == set(expected_types) | {'controller_manager'}, 'extra controller configuration')
        expected_ingestion = dict(
            controller_state_topic='/joint_trajectory_controller/controller_state',
            required_samples=5,
            max_time_from_start=.010,
            timeout=10.
        )
        start_speed = profile.get('measured_start_velocity_limit')
        valid_start_speed = (
            isinstance(start_speed, (int, float)) and not isinstance(start_speed, bool) and
            math.isfinite(start_speed) and start_speed > 0
        )
        require(valid_start_speed, 'invalid measured_start_velocity_limit')
        if valid_start_speed:
            require(
                all(start_speed <= profile['joint_settings'][name]['velocity_limit'] / 2 for name in names),
                'measured_start_velocity_limit must be at most half each joint velocity_limit'
            )
        for upstream in ('champ', 'rl'):
            parameters = yaml.safe_load((directory / (upstream + '.yaml')).read_text())['/**']['ros__parameters']
            generated_start_speed = (
                parameters.get('measured_start', {}).get('velocity_limit') if upstream == 'champ'
                else parameters.get('measured_start_velocity_limit')
            )
            require(
                generated_start_speed == [start_speed] * len(names),
                f'{upstream}: measured-start velocity must match profile'
            )
            require(
                parameters.get('measured_start_enabled') is True and
                parameters.get('jtc_ingestion') == expected_ingestion,
                f'{upstream}: JTC ingestion readiness settings mismatch'
            )
        trajectory = controllers['joint_trajectory_controller']['ros__parameters']
        command_names = ['bxi_pair_adapter/' + name if name.split('_')[0] in adapted else name for name in names]
        require(
            trajectory['joints'] == names and trajectory['command_joints'] == command_names and
            trajectory['command_interfaces'] == ['position'] and
            trajectory['state_interfaces'] == ['position', 'velocity'],
            'JTC joint/reference mismatch'
        )
        for key in (
            'set_last_command_interface_value_as_state_on_activation',
            'open_loop_control',
            'allow_partial_joints_goal',
            'is_async'
        ):
            require(trajectory.get(key) is False, f'JTC {key} must be false')
        require(trajectory.get('interpolate_from_desired_state') is True,
                'JTC interpolate_from_desired_state must be true')
        timeout, goal = trajectory['cmd_timeout'], trajectory['constraints']['goal_time']
        require(
            all(math.isfinite(value) for value in (timeout, goal)) and timeout > 0 and timeout > goal,
            'cmd_timeout must be positive and greater than goal_time'
        )
        adapter = controllers.get('bxi_pair_adapter', {}).get('ros__parameters', {})
        groups = [profile['legs'][leg]['hardware_name'] for leg in adapted]
        if adapted:
            require(
                adapter['hardware_groups'] == groups and adapter.get('is_async') is False and
                adapter.get('api_max_tries') == 1 and adapter.get('max_period_sec') == .005,
                'adapter groups, synchronization or API policy mismatch'
            )
        systems = root.findall('ros2_control')
        require(len(systems) == 4, 'four leg systems required')
        hardware_names, gpio_names, covered = set(), set(), []
        buses = {'FR': 'can0', 'FL': 'can1', 'BR': 'can2', 'BL': 'can3'}
        for leg in ('BL', 'BR', 'FL', 'FR'):
            spec = profile['legs'][leg]
            joint_names = [f'{leg}_{kind}_joint' for kind in ('collar', 'hip', 'knee')]
            require(
                spec['joints'] == joint_names and spec['ids'] == [1, 2, 3] and spec['bus'] == buses[leg],
                f'{leg}: joint/bus/ID mapping mismatch'
            )
            require(
                leg != 'FR' or spec['expected_serials'] == ['SN-0016', 'SN-0014', 'SN-0010'],
                'FR serial mapping mismatch'
            )
            require(
                spec['hardware_name'] not in hardware_names and spec['gpio'] not in gpio_names,
                'duplicate hardware or GPIO name'
            )
            hardware_names.add(spec['hardware_name']);
            gpio_names.add(spec['gpio'])
            matches = [system for system in systems if system.get('name') == spec['hardware_name']]
            require(len(matches) == 1, f'{leg}: missing or duplicate system')
            if len(matches) != 1:
                continue
            system = matches[0]
            plugin = ('bxi_hardware/BxiSystemInterface' if stage3 and leg == 'FR' else
                      'bxi_hardware/BxiPairMockSystem' if leg in adapted else 'mock_components/GenericSystem')
            require(
                system.findtext('hardware/plugin') == plugin and system.get('type') == 'system',
                f'{leg}: unexpected hardware plugin'
            )
            joints = system.findall('joint')
            require([joint.get('name') for joint in joints] == joint_names, f'{leg}: URDF joint order mismatch')
            covered.extend(joint.get('name') for joint in joints)
            for joint in joints:
                name = joint.get('name')
                settings = profile['joint_settings'][name]
                require(
                    all(isinstance(value, (int, float)) and math.isfinite(value) for value in settings.values()),
                    f'{name}: nonfinite setting'
                )
                require(
                    settings['position_min'] < settings['position_max'] and settings['velocity_limit'] > 0 and
                    settings['max_target_delta_per_cycle'] > 0 and settings['torque_limit'] > 0 and
                    settings['deviation_limit_rad'] > 0 and settings['deviation_cycles'] > 0 and
                    0 <= settings['kp'] <= settings['kp_max'] and 0 <= settings['kd'] <= settings['kd_max'],
                    f'{name}: invalid bounds or gains'
                )
                bounds = {
                    'position': [settings['position_min'], settings['position_max']],
                    'velocity': [-settings['velocity_limit'], settings['velocity_limit']],
                    'kp': [0., settings['kp_max']],
                    'kd': [0., settings['kd_max']],
                    'effort': [0., 0.]
                }
                mechanical = root.find(f"joint[@name='{name}']/limit")
                require(
                    mechanical is not None and float(mechanical.get('lower')) <= bounds['position'][0] and
                    float(mechanical.get('upper')) >= bounds['position'][1],
                    f'{name}: mechanical bounds mismatch'
                )
                commands = joint.findall('command_interface')
                expected = COMMANDS if leg in adapted else {'position'}
                require(
                    len(commands) == len(expected) and {item.get('name') for item in commands} == expected,
                    f'{name}: command set mismatch'
                )
                for command in commands:
                    field = command.get('name')
                    require(
                        command.get('data_type', 'double') == 'double' and field in bounds,
                        f'{name}: command type or field mismatch'
                    )
                    params = command.findall('param')
                    require(
                        len(params) == 2 and {item.get('name') for item in params} == {'min', 'max'},
                        f'{name}/{field}: transformed or missing bounds'
                    )
                    if field in bounds:
                        require(
                            [float(command.findtext(f"param[@name='{key}']")) for key in ('min', 'max')] == bounds[field],
                            f'{name}/{field}: bounds mismatch'
                        )
                states = joint.findall('state_interface')
                expected = {'position', 'velocity', 'effort', 'temperature'} if leg in adapted else {'position', 'velocity'}
                require(
                    len(states) == len(expected) and {item.get('name') for item in states} == expected and
                    all(item.get('data_type', 'double') == 'double' for item in states),
                    f'{name}: state set/type mismatch'
                )
                require(
                    float(joint.findtext("state_interface[@name='position']/param[@name='initial_value']")) ==
                    profile['initial_positions'][name],
                    f'{name}: initial position mismatch'
                )
                for pose in ('initial_positions', 'standby_positions', 'standup_positions'):
                    value = profile[pose][name]
                    require(
                        math.isfinite(value) and bounds['position'][0] <= value <= bounds['position'][1],
                        f'{name}: {pose} outside range'
                    )
                if leg in adapted:
                    require(adapter['joint_settings'][name] == settings, f'{name}: adapter/profile limits differ')
            gpios = system.findall('gpio')
            if leg in adapted:
                require(len(gpios) == 1 and gpios[0].get('name') == spec['gpio'], f'{leg}: GPIO mismatch')
                if len(gpios) == 1:
                    items = list(gpios[0])
                    expected = [('command_interface', 'commit_generation'), ('state_interface', 'accepted_generation')]
                    require(
                        len(items) == 2 and all(
                            item.tag == tag and item.attrib == {'name': name, 'data_type': 'double'} and not list(item)
                            for item,
                            (tag, name) in zip(items, expected)
                        ),
                        f'{leg}: GPIO type/transform mismatch'
                    )
                require(
                    adapter['groups'][spec['hardware_name']] == {
                        'joints': joint_names,
                        'hardware_name': spec['hardware_name'],
                        'gpio': spec['gpio']
                    },
                    f'{leg}: adapter group mismatch'
                )
            else:
                require(not gpios, f'{leg}: unexpected GPIO')
        require(sorted(covered) == sorted(names), 'duplicate or missing robot joint')
        if adapted:
            require(
                set(adapter['groups']) == set(groups) and set(adapter['joint_settings']) ==
                {name for name in names if name.split('_')[0] in adapted},
                'extra adapter group/joint'
            )
        tolerance = profile.get('standup_resync_tolerance')
        if profile.get('profile') in ('stage6_real_gain20', 'stage6_vcan_gain20', 'stage8_real_walk', 'stage8_vcan_walk'):
            valid_tolerance = (isinstance(tolerance, (int, float)) and not isinstance(tolerance, bool)
                               and math.isfinite(tolerance) and tolerance > 0)
            require(valid_tolerance and all(
                profile['joint_settings'][name]['kp_max'] * tolerance <
                profile['joint_settings'][name]['torque_limit']
                for name in profile['legs']['FR']['joints']),
                'gain20 resync tolerance must be positive and below each FR torque bound')
        else:
            require(tolerance == .05, 'Stage 2 synchronization thresholds differ')
        require(profile.get('joint_states_max_age') == .2, 'Stage 2 synchronization thresholds differ')
    except (ValueError, OSError, KeyError, TypeError, AttributeError, ET.ParseError) as exception:
        errors.append(str(exception))
    return errors

def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('config_dir', type=Path);
    p.add_argument('urdf', type=Path)
    p.add_argument('--adapter', action='store_true')
    p.add_argument('--real-configuration', action='store_true')
    a = p.parse_args(argv)
    errors = check_adapter(a.config_dir, a.urdf, real_configuration=a.real_configuration) if a.adapter else check(a.config_dir, a.urdf)
    print(
        json.dumps(
            {'result': 'FAIL' if errors else 'PASS', 'errors': errors, 'limitation': LIMITATION},
            ensure_ascii=False,
            indent=2
        )
    )
    return int(bool(errors))
if __name__ == '__main__':
    raise SystemExit(main())
