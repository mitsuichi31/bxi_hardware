#!/usr/bin/env python3
"""Generate G-only files from explicit IDs; never discover or change a CAN device."""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import re
import yaml

HERE = Path(__file__).resolve().parent
if not (HERE / 'hardware.yaml.in').is_file():
    from ament_index_python.packages import get_package_share_directory
    HERE = Path(get_package_share_directory('bxi_hardware')) / 'hil/n_axis'

def package_root():
    for p in HERE.parents:
        if (p / 'config/bxi_hardware.schema.json').is_file():
            return p
    raise ValueError('cannot locate bxi_hardware package')

def generate(axes, ids, destination, output_dir, bus='can0'):
    if axes not in (2, 3) or len(ids) != axes or any(type(i) is not int or not 1 <= i <= 15 for i in ids):
        raise ValueError('N=2/3 and exactly N explicit CAN IDs in 1..15 required')
    if len(set(ids)) != axes:
        raise ValueError('duplicate CAN IDs')
    # Broadcast ABI maps joint1..N to ID1..N; this is validation, not a default.
    if ids != list(range(1, axes + 1)):
        raise ValueError('pair_generation_v1 broadcast requires explicit ordered IDs 1..N')
    if not re.fullmatch(r'[A-Za-z0-9_]+', bus):
        raise ValueError('invalid CAN bus name')
    output_dir = Path(output_dir).resolve()
    if not output_dir.is_dir():
        raise ValueError('output_dir must already exist')
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=False)
    config = yaml.safe_load((HERE / 'hardware.yaml.in').read_text().replace('@CAN_BUS@', bus))
    config['joints'] = {f'joint{a+1}': dict(enabled=True, can_bus=bus, can_id=i,
        motor_dir=1, motor_type='MOTOR_70', lower=-math.pi/6, upper=math.pi/6,
        default_kp=0., default_kd=0.) for a, i in enumerate(ids)}
    hp = destination / 'hardware.yaml'
    hp.write_text('# 暫定: first_write=1000ms, feedback=3ms, command=5ms; physical setup unverified\n' + yaml.safe_dump(config, sort_keys=False))
    root = package_root()
    validator = root / 'scripts/validate_config.py'
    if not validator.is_file():
        validator = root.parent.parent / 'lib/bxi_hardware/validate_config.py'
    spec = importlib.util.spec_from_file_location('bxi_validate', validator)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    errors = module.validate(hp, root / 'config/bxi_hardware.schema.json', root / 'config/specs')
    if errors:
        raise ValueError('; '.join(errors))
    controllers = {'controller_manager': {'ros__parameters': {'update_rate': 1000,
        'thread_priority': 50, 'cpu_affinity': 11, 'lock_memory': True, 'use_sim_time': False,
        'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
        'n_axis': {'type': 'bxi_hardware/NAxisTrajectoryController'}}},
        'n_axis': {'ros__parameters': {'joints': list(config['joints']), 'output_dir': str(output_dir),
        'api_max_tries': 1, 'record_seconds': 120., 'status_period': .05}}}
    (destination / 'controllers.yaml').write_text('# 暫定RT配置・試行上限・容量。実機承認ではない\n' + yaml.safe_dump(controllers, sort_keys=False))
    (destination / 'manifest.json').write_text(json.dumps({'axes': axes, 'explicit_ids': ids,
        'can_bus': bus, 'output_dir': str(output_dir), 'source': 'explicit_cli_arguments'}, indent=2)+'\n')
    return destination

def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--axes', type=int, choices=(2, 3), required=True)
    p.add_argument('--ids', type=int, nargs='+', required=True)
    p.add_argument('--destination', type=Path, required=True)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--can-bus', default='can0')
    a = p.parse_args(argv)
    try:
        print(generate(a.axes, a.ids, a.destination, a.output_dir, a.can_bus))
    except (ValueError, OSError) as e:
        p.error(str(e))
    return 0
if __name__ == '__main__':
    raise SystemExit(main())
