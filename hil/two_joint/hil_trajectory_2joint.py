#!/usr/bin/env python3
"""2軸の同位相/逆位相軌道。既定はCSV生成のみ。ROS実行はvcan専用。

RT controller内の軌道生成ではない。実機用のlease/watchdogは別レビュー。
各軸のpublishは同じ標本から行うがDDSの同時適用は保証しない。
"""
import argparse
import csv
import math
import time
from pathlib import Path


def sample(t, amplitude=math.pi/2, frequency=.5, cycles=5., ramp_cycles=2., phase='same'):
    if phase not in ('same', 'opposite'):
        raise ValueError('unknown phase')
    if not all(math.isfinite(x) for x in (t, amplitude, frequency, cycles, ramp_cycles)):
        raise ValueError('nonfinite trajectory')
    if not 0 < amplitude <= math.pi/2 or frequency <= 0 or cycles <= 0 or ramp_cycles <= 0:
        raise ValueError('invalid trajectory')
    ramp, flat = ramp_cycles/frequency, cycles/frequency
    omega = 2*math.pi*frequency
    if amplitude*math.hypot(omega, math.pi/(2*ramp)) > 5.0:
        raise ValueError('velocity upper bound exceeds 5 rad/s')
    t = max(0., min(t, 2*ramp+flat))
    if t < ramp:
        a = amplitude*.5*(1-math.cos(math.pi*t/ramp))
        da = amplitude*.5*math.pi/ramp*math.sin(math.pi*t/ramp)
    elif t > ramp+flat:
        x = (t-ramp-flat)/ramp
        a = amplitude*.5*(1+math.cos(math.pi*x))
        da = -amplitude*.5*math.pi/ramp*math.sin(math.pi*x)
    else:
        a, da = amplitude, 0.
    p, v = a*math.sin(omega*t), da*math.sin(omega*t)+a*omega*math.cos(omega*t)
    sign = 1 if phase == 'same' else -1
    return ((p, v), (sign*p, sign*v))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase', choices=('same','opposite'), required=True)
    parser.add_argument('--amplitude', type=float, default=math.pi/2)
    parser.add_argument('--frequency', type=float, default=.5)
    parser.add_argument('--cycles', type=float, default=5)
    parser.add_argument('--ramp-cycles', type=float, default=2)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--execute-vcan', help='vcan interface, validated before ROS init')
    args = parser.parse_args()
    sample(0, args.amplitude, args.frequency, args.cycles, args.ramp_cycles, args.phase)
    duration = (2*args.ramp_cycles+args.cycles)/args.frequency
    if args.execute_vcan:
        import json, re, subprocess
        if not re.fullmatch(r'vcan\d+', args.execute_vcan):
            parser.error('vcan only')
        data = json.loads(subprocess.check_output(['ip','-details','-json','link','show',args.execute_vcan]))
        if data[0]['linkinfo']['info_kind'] != 'vcan':
            parser.error('not a vcan device')
        import rclpy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray
        rclpy.init()
        node = rclpy.create_node('two_joint_trajectory')
        pubs = [node.create_publisher(Float64MultiArray, f'/joint{i}_command/commands', 1) for i in (1,2)]
        state = {'at':0, 'position':{}}
        def receive(msg):
            state.update(at=time.monotonic(), position=dict(zip(msg.name,msg.position)))
        sub = node.create_subscription(JointState,'/joint_states',receive,1)
        deadline = time.monotonic()+5
        while len(state['position']) < 2 and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
        if any(abs(state['position'].get(f'joint{i}',float('inf'))) > .05 for i in (1,2)):
            raise RuntimeError('start position unavailable or outside +/-0.05 rad')
    with args.out.open('x',newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['t_s','published_ns','can_id','position','velocity','kp','kd','effort'])
        start = time.monotonic()
        try:
            for cycle in range(round(duration*1000)+1):
                t = cycle/1000
                if args.execute_vcan:
                    wait = start+t-time.monotonic()
                    if wait > 0: time.sleep(wait)
                    rclpy.spin_once(node,timeout_sec=0)
                    if time.monotonic()-state['at'] > .2:
                        raise RuntimeError('joint state older than 0.2s')
                for motor_id,(p,v) in enumerate(sample(t,args.amplitude,args.frequency,args.cycles,args.ramp_cycles,args.phase),1):
                    if args.execute_vcan:
                        actual = state['position'].get(f'joint{motor_id}',float('nan'))
                        if not math.isfinite(actual) or 20*abs(p-actual) > 5:
                            raise RuntimeError('position/torque protection')
                        pubs[motor_id-1].publish(Float64MultiArray(data=[p,v,20.,1.,0.]))
                    writer.writerow([t,time.monotonic_ns() if args.execute_vcan else cycle*1000000,motor_id,p,v,20.,1.,0.])
        finally:
            if args.execute_vcan:
                for _ in range(5):
                    for pub in pubs: pub.publish(Float64MultiArray(data=[0.,0.,0.,0.,0.]))
                    rclpy.spin_once(node,timeout_sec=.02)
                node.destroy_node();rclpy.shutdown()
    print(f'CSV: {args.out}; phase={args.phase}; duration={duration}s')


if __name__ == '__main__':
    main()
