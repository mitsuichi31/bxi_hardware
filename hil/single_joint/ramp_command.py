#!/usr/bin/env python3
"""joint1 の目標位置を少しずつ動かす（1軸の実機試験用）。

/joint_states で今の位置を読み、目標位置を --speed [rad/s] で目標まで動かしながら
/joint1_command/commands（data = [position, kp, kd]）を送る。

次のときは中止し、Kp=0 の指令（その場で力を抜く）を送って終わる:
  - Kp × |目標 − 位置| が --max-torque [N·m] を超えた
  - /joint_states が 0.2 秒以上届かない
  - Ctrl+C
正常に終わったときは、目標位置で保持したまま終わる（--release を付けると力を抜く）。

例:
  ros2 run bxi_hardware ramp_command.py 0.2 --kp 10 --kd 2
  ros2 run bxi_hardware ramp_command.py 0.0 --kp 20 --kd 2 --release
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class Ramp(Node):
    def __init__(self, args):
        super().__init__("bxi_ramp_command")
        self.args = args
        self.position = None
        self.stamp = 0.0
        self.pub = self.create_publisher(Float64MultiArray, "/joint1_command/commands", 10)
        self.create_subscription(JointState, "/joint_states", self.on_state, 10)

    def on_state(self, msg):
        if self.args.joint in msg.name:
            value = msg.position[msg.name.index(self.args.joint)]
            if math.isfinite(value):
                self.position = value
                self.stamp = time.monotonic()

    def send(self, position, kp, kd):
        self.pub.publish(Float64MultiArray(data=[position, kp, kd]))

    def wait_state(self, timeout):
        end = time.monotonic() + timeout
        while self.position is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.position


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("target", type=float, help="目標位置 [rad]")
    ap.add_argument("--joint", default="joint1")
    ap.add_argument("--kp", type=float, default=10.0)
    ap.add_argument("--kd", type=float, default=2.0)
    ap.add_argument("--speed", type=float, default=0.3, help="目標を動かす速さ [rad/s]（0〜2）")
    ap.add_argument("--hold", type=float, default=1.0, help="到達後に待つ時間 [s]")
    ap.add_argument("--rate", type=float, default=100.0, help="指令の送信周期 [Hz]")
    ap.add_argument("--max-kp", type=float, default=20.0)
    ap.add_argument("--max-torque", type=float, default=5.0, help="Kp×誤差 の上限 [N·m]")
    ap.add_argument("--release", action="store_true", help="終了時に Kp=0 にして力を抜く")
    args = ap.parse_args()
    if not 0.0 < args.kp <= args.max_kp:
        raise SystemExit(f"--kp は 0 より大きく {args.max_kp} 以下にする（--max-kp で変更）")
    if not 0.0 <= args.kd <= 5.0:
        raise SystemExit("--kd は 0〜5 にする")
    if not 0.0 < args.speed <= 2.0:
        raise SystemExit("--speed は 0〜2 rad/s にする")

    rclpy.init()
    node = Ramp(args)
    start = node.wait_state(2.0)
    if start is None:
        node.get_logger().error("/joint_states が届かない。launch が起動しているか確認する")
        rclpy.shutdown()
        raise SystemExit(1)

    dist = args.target - start
    print(f"今の位置 {start:+.4f} rad → 目標 {args.target:+.4f} rad（差 {dist:+.4f} rad、{math.degrees(dist):+.1f}°）")
    ramp_time = abs(dist) / args.speed
    direction = math.copysign(1.0, dist)
    t0 = time.monotonic()
    last_print = -1.0
    aborted = None
    try:
        while True:
            elapsed = time.monotonic() - t0
            if elapsed < ramp_time:
                p_des = start + direction * args.speed * elapsed
            else:
                p_des = args.target
                if elapsed >= ramp_time + args.hold:
                    break
            rclpy.spin_once(node, timeout_sec=0.0)
            if time.monotonic() - node.stamp > 0.2:
                aborted = "/joint_states が 0.2 秒以上途切れた"
                break
            est = args.kp * abs(p_des - node.position)
            if est > args.max_torque:
                aborted = f"位置の誤差が大きい（Kp×誤差 = {est:.2f} N·m > {args.max_torque}）"
                break
            node.send(p_des, args.kp, args.kd)
            if elapsed - last_print >= 0.2:
                print(f"  t={elapsed:5.2f}s  目標 {p_des:+.4f}  位置 {node.position:+.4f}")
                last_print = elapsed
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        aborted = "Ctrl+C"

    rclpy.spin_once(node, timeout_sec=0.05)
    if aborted or args.release:
        for _ in range(5):
            node.send(node.position, 0.0, 0.0)
            time.sleep(0.02)
    if aborted:
        print(f"\n[中止] {aborted}。Kp=Kd=0 を送って力を抜いた")
    err = node.position - args.target
    print(f"終了時: 位置 {node.position:+.4f} rad（目標との差 {err:+.4f} rad、{math.degrees(err):+.2f}°）"
          + ("、力を抜いた" if (aborted or args.release) else "、目標位置で保持中"))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
