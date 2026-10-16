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
        m = json.loads((d/'manifest.json').read_text())
        hw = yaml.safe_load((d/'hardware.yaml').read_text())
        ctrl = yaml.safe_load((d/'controllers.yaml').read_text())
        tree = ET.parse(urdf).getroot()
        bounds = profile_bounds()
        n = m['axes']; names = [f'joint{i+1}' for i in range(n)]
        if n not in (2, 3) or m.get('source') != 'explicit_cli_arguments':
            errors.append('N or explicit-ID provenance missing')
        if list(hw['joints']) != names or [j.get('can_id') for j in hw['joints'].values()] != m['explicit_ids']:
            errors.append('joint order or explicit IDs mismatch')
        if m['explicit_ids'] != list(range(1, n+1)) or len(set(m['explicit_ids'])) != n:
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
        if Path(params.get('config_file', '')).resolve() != (d/'hardware.yaml').resolve():
            errors.append('URDF config_file mismatch')
        joints = system.findall('joint')
        if [j.get('name') for j in joints] != names:
            errors.append('URDF joint order mismatch')
        for j in joints:
            commands = j.findall('command_interface'); states = j.findall('state_interface')
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
        types = {k: v.get('type') for k,v in cm.items() if isinstance(v, dict)}
        if types != {'joint_state_broadcaster': 'joint_state_broadcaster/JointStateBroadcaster',
                     'n_axis': 'bxi_hardware/NAxisTrajectoryController'}:
            errors.append('unexpected configured controller')
        if set(ctrl) != {'controller_manager', 'n_axis'} or ctrl['n_axis']['ros__parameters']['joints'] != names:
            errors.append('extra controller config or wrong controller joints')
        if ctrl['n_axis']['ros__parameters']['output_dir'] != m['output_dir'] or cm.get('update_rate') != 1000:
            errors.append('output_dir or update rate mismatch')
    except (ValueError, OSError, KeyError, TypeError, ET.ParseError) as e:
        errors.append(str(e))
    return errors

def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('config_dir', type=Path); p.add_argument('urdf', type=Path)
    a=p.parse_args(argv); errors=check(a.config_dir,a.urdf)
    print(json.dumps({'result': 'FAIL' if errors else 'PASS', 'errors': errors, 'limitation': LIMITATION}, ensure_ascii=False, indent=2))
    return int(bool(errors))
if __name__=='__main__':
    raise SystemExit(main())
