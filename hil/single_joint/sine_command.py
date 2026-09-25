#!/usr/bin/env python3
"""joint1 を SIN 波で動かす（1軸の実機試験用）。

目標位置  p(t) = center + a(t) * sin(2πft)
目標速度  v(t) = dp/dt（MIT の目標速度として送る）
振幅 a(t) は最初の --ramp-cycles 周期で 0 → --amplitude、最後の --ramp-cycles 周期で 0 に戻す
（なめらかに始めて止めるため、cos 形状で変化させる）。中心は既定で実行時の位置。

/joint1_command/commands（data = [position, velocity, kp, kd, effort]）を --rate [Hz] で送る。

摩擦の打ち消し: 目標速度の向きにトルクを上乗せする（MIT のトルク欄）。Stribeck モデル:
  effort = (Fc + (Fs − Fc) × exp(−|v| / vs)) × tanh(v / v0)
    v : 目標速度            Fc: 動摩擦 --friction-ff        Fs: 静止摩擦 --friction-static（省略時 = Fc）
    vs: --stribeck-velocity  v0: 向きの切り替え幅 --friction-velocity
動き出し（低速）では静止摩擦 Fs に近い大きさ、速く動いている間は動摩擦 Fc になる。
Fs を省略すると一定値 Fc の上乗せになる。実測の速度ではなく目標速度を使い、折り返し点では tanh で
なめらかに向きを変える（振動を避けるため）。

実行前に止める:
  - center ± amplitude が --lower / --upper（bxi_hardware.yaml の範囲）をはみ出す
  - 最大速度 amplitude × 2πf が --max-velocity を超える
実行中に中止し、Kp=0 の指令（その場で力を抜く）を送って終わる:
  - Kp × |目標 − 位置| + |上乗せトルク| が --max-torque [N·m] を超えた
  - /joint_states が 0.2 秒以上届かない
  - Ctrl+C
正常に終わったときは中心位置で保持したまま終わる（--release を付けると力を抜く）。

目標と実際の位置・速度・トルクを CSV に保存し、終了後に追従の結果（誤差、振幅比、位相の遅れ）を表示する。

例:
  ros2 run bxi_hardware sine_command.py --amplitude 0.1 --frequency 0.5 --cycles 5
  ros2 run bxi_hardware sine_command.py --amplitude 0.2 --frequency 1.0 --kp 30 --max-kp 40 --release
  ros2 run bxi_hardware sine_command.py --amplitude 0.1 --frequency 0.5 --friction-ff 0.8
  ros2 run bxi_hardware sine_command.py --amplitude 0.3 --frequency 0.2 --friction-ff 0.6 --friction-static 1.0
"""

import argparse
import csv
import datetime as dt
import math
import pathlib
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class Sine(Node):
    def __init__(self, joint):
        super().__init__("bxi_sine_command")
        self.joint = joint
        self.position = None
        self.velocity = math.nan
        self.effort = math.nan
        self.stamp = 0.0
        self.pub = self.create_publisher(Float64MultiArray, "/joint1_command/commands", 10)
        # 深さ 1: 受信キューに古い状態がたまると、spin_once が古い値から読んで遅れが大きく見える
        self.create_subscription(JointState, "/joint_states", self.on_state, 1)

    def on_state(self, msg):
        if self.joint not in msg.name:
            return
        i = msg.name.index(self.joint)
        value = msg.position[i]
        if math.isfinite(value):
            self.position = value
            self.velocity = msg.velocity[i] if i < len(msg.velocity) else math.nan
            self.effort = msg.effort[i] if i < len(msg.effort) else math.nan
            self.stamp = time.monotonic()

    def send(self, position, velocity, kp, kd, effort=0.0):
        self.pub.publish(Float64MultiArray(data=[position, velocity, kp, kd, effort]))

    def wait_state(self, timeout):
        end = time.monotonic() + timeout
        while self.position is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.position


class Trajectory:
    """振幅を cos 形状で立ち上げ・立ち下げる SIN 波。"""

    def __init__(self, center, amplitude, frequency, cycles, ramp_cycles):
        self.center = center
        self.amplitude = amplitude
        self.omega = 2.0 * math.pi * frequency
        self.ramp = ramp_cycles / frequency
        self.flat = cycles / frequency
        self.duration = 2.0 * self.ramp + self.flat

    def envelope(self, t):
        """(a, da/dt)"""
        if self.ramp <= 0.0:
            return self.amplitude, 0.0
        if t < self.ramp:
            x = t / self.ramp
            return (self.amplitude * 0.5 * (1.0 - math.cos(math.pi * x)),
                    self.amplitude * 0.5 * math.pi / self.ramp * math.sin(math.pi * x))
        if t > self.ramp + self.flat:
            x = min((t - self.ramp - self.flat) / self.ramp, 1.0)
            return (self.amplitude * 0.5 * (1.0 + math.cos(math.pi * x)),
                    -self.amplitude * 0.5 * math.pi / self.ramp * math.sin(math.pi * x))
        return self.amplitude, 0.0

    def at(self, t):
        """(目標位置, 目標速度)"""
        t = min(max(t, 0.0), self.duration)
        a, da = self.envelope(t)
        s, c = math.sin(self.omega * t), math.cos(self.omega * t)
        return self.center + a * s, da * s + a * self.omega * c

    def in_flat(self, t):
        return self.ramp <= t <= self.ramp + self.flat


def friction_torque(target_velocity, kinetic, friction_velocity, static=None, stribeck_velocity=0.2):
    """目標速度の向きに上乗せするトルク [N·m]（Stribeck モデル）。

    大きさは低速で static、高速で kinetic に近づく。向きは tanh で速度 0 付近をなめらかに通る。
    static を省略すると一定値 kinetic になる。
    """
    static = kinetic if static is None else static
    if kinetic == 0.0 and static == 0.0:
        return 0.0
    magnitude = kinetic + (static - kinetic) * math.exp(-abs(target_velocity) / stribeck_velocity)
    return magnitude * math.tanh(target_velocity / friction_velocity)


def summarize(rows, traj):
    """振幅が一定の区間で、誤差と、位置の SIN 成分（振幅比・位相）を求める。"""
    flat = [r for r in rows if traj.in_flat(r["t"])]
    if len(flat) < 10:
        print("振幅一定の区間のデータが少なく、集計できない")
        return
    t = np.array([r["t"] for r in flat])
    target = np.array([r["target"] for r in flat])
    pos = np.array([r["position"] for r in flat])
    err = pos - target
    # pos ≈ c0 + s·sin(ωt) + k·cos(ωt) を最小二乗で当てはめる
    basis = np.column_stack([np.ones_like(t), np.sin(traj.omega * t), np.cos(traj.omega * t)])
    (c0, s, k), *_ = np.linalg.lstsq(basis, pos, rcond=None)
    ratio = math.hypot(s, k) / traj.amplitude
    lag_deg = -math.degrees(math.atan2(k, s))
    lag_ms = lag_deg / 360.0 / (traj.omega / (2.0 * math.pi)) * 1000.0
    print(f"\n振幅一定の区間（{t[-1] - t[0]:.1f} 秒、{len(flat)} 点）:")
    print(f"  追従誤差: RMS {np.sqrt(np.mean(err ** 2)):.4f} rad / 最大 {np.max(np.abs(err)):.4f} rad"
          f"（{math.degrees(np.max(np.abs(err))):.2f}°）")
    print(f"  位置の SIN 成分: 振幅比 {ratio:.3f}（目標 {traj.amplitude:.3f} rad → {math.hypot(s, k):.3f} rad）、"
          f"位相の遅れ {lag_deg:.1f}°（{lag_ms:.0f} ms）、中心のずれ {c0 - traj.center:+.4f} rad")
    # 引っかかり（stick-slip）の目安: 目標が動いているのに軸が止まっている時間の割合
    moving = [r for r in flat if abs(r["target_velocity"]) > 0.1 * traj.amplitude * traj.omega]
    if moving and all(math.isfinite(r["velocity"]) for r in moving):
        stuck = sum(1 for r in moving if abs(r["velocity"]) < 0.02)
        peak = max(abs(r["velocity"]) for r in flat)
        print(f"  引っかかり: 目標が動いているのに止まっている時間 {100.0 * stuck / len(moving):.0f} %、"
              f"軸の最大速度 {peak:.2f} rad/s（目標の最大 {traj.amplitude * traj.omega:.2f} rad/s）")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joint", default="joint1")
    ap.add_argument("--amplitude", type=float, default=0.1, help="振幅 [rad]")
    ap.add_argument("--frequency", type=float, default=0.5, help="周波数 [Hz]")
    ap.add_argument("--cycles", type=float, default=5.0, help="振幅一定で動かす周期の数")
    ap.add_argument("--ramp-cycles", type=float, default=1.0, help="振幅を立ち上げ・立ち下げる周期の数")
    ap.add_argument("--center", type=float, default=None, help="中心位置 [rad]（既定: 実行時の位置）")
    ap.add_argument("--kp", type=float, default=20.0)
    ap.add_argument("--kd", type=float, default=1.0)
    ap.add_argument("--friction-ff", type=float, default=0.0,
                    help="動摩擦 Fc: 速く動いている間に上乗せするトルク [N·m]（0〜--max-friction-ff）")
    ap.add_argument("--friction-static", type=float, default=None,
                    help="静止摩擦 Fs: 動き出し（低速）で上乗せするトルク [N·m]（省略時は --friction-ff と同じ = 一定値）")
    ap.add_argument("--stribeck-velocity", type=float, default=0.2,
                    help="Fs から Fc へ移る速さの目安 vs [rad/s]")
    ap.add_argument("--friction-velocity", type=float, default=0.05,
                    help="上乗せトルクの向きを切り替えるときの速度の幅 v0 [rad/s]（tanh の目盛り）")
    ap.add_argument("--max-friction-ff", type=float, default=2.0)
    ap.add_argument("--rate", type=float, default=200.0, help="指令の送信周期 [Hz]")
    ap.add_argument("--hold", type=float, default=1.0, help="終了後に中心位置で待つ時間 [s]")
    ap.add_argument("--lower", type=float, default=-0.5, help="関節の下限 [rad]（bxi_hardware.yaml と合わせる）")
    ap.add_argument("--upper", type=float, default=0.5, help="関節の上限 [rad]")
    ap.add_argument("--max-kp", type=float, default=20.0)
    ap.add_argument("--max-velocity", type=float, default=2.0, help="最大速度 amplitude×2πf の上限 [rad/s]")
    ap.add_argument("--max-torque", type=float, default=5.0, help="Kp×誤差 の上限 [N·m]")
    ap.add_argument("--release", action="store_true", help="終了時に Kp=0 にして力を抜く")
    ap.add_argument("--log", type=pathlib.Path, default=None,
                    help="CSV の保存先（既定: /ws/log/ があればその中、なければカレント）")
    args = ap.parse_args()

    if not 0.0 < args.amplitude <= 0.5:
        raise SystemExit("--amplitude は 0 より大きく 0.5 rad 以下にする")
    if not 0.0 < args.frequency <= 5.0:
        raise SystemExit("--frequency は 0 より大きく 5 Hz 以下にする")
    if args.cycles < 0.0 or args.ramp_cycles < 0.0:
        raise SystemExit("--cycles と --ramp-cycles は 0 以上にする")
    if not 0.0 < args.kp <= args.max_kp:
        raise SystemExit(f"--kp は 0 より大きく {args.max_kp} 以下にする（--max-kp で変更）")
    if not 0.0 <= args.kd <= 5.0:
        raise SystemExit("--kd は 0〜5 にする")
    if not 0.0 <= args.friction_ff <= args.max_friction_ff:
        raise SystemExit(f"--friction-ff は 0〜{args.max_friction_ff} N·m にする（--max-friction-ff で変更）")
    if args.friction_static is None:
        args.friction_static = args.friction_ff
    if not args.friction_ff <= args.friction_static <= args.max_friction_ff:
        raise SystemExit(f"--friction-static は --friction-ff 以上、{args.max_friction_ff} N·m 以下にする")
    if args.stribeck_velocity <= 0.0:
        raise SystemExit("--stribeck-velocity は 0 より大きくする")
    if args.friction_velocity <= 0.0:
        raise SystemExit("--friction-velocity は 0 より大きくする")
    if not 10.0 <= args.rate <= 500.0:
        raise SystemExit("--rate は 10〜500 Hz にする")
    peak_velocity = args.amplitude * 2.0 * math.pi * args.frequency
    if peak_velocity > args.max_velocity:
        raise SystemExit(f"最大速度 {peak_velocity:.2f} rad/s が --max-velocity {args.max_velocity} を超える")

    rclpy.init()
    node = Sine(args.joint)
    start = node.wait_state(2.0)
    if start is None:
        node.get_logger().error("/joint_states が届かない。launch が起動しているか確認する")
        rclpy.shutdown()
        raise SystemExit(1)
    center = start if args.center is None else args.center
    if center - args.amplitude < args.lower or center + args.amplitude > args.upper:
        rclpy.shutdown()
        raise SystemExit(f"中心 {center:+.3f} ± 振幅 {args.amplitude:.3f} rad が範囲 "
                         f"[{args.lower}, {args.upper}] をはみ出す（--center で中心を指定できる）")
    if abs(center - start) > 1e-3:
        rclpy.shutdown()
        raise SystemExit(f"中心 {center:+.4f} が今の位置 {start:+.4f} と違う。先に ramp_command.py で中心へ移動する")

    traj = Trajectory(center, args.amplitude, args.frequency, args.cycles, args.ramp_cycles)
    log_path = args.log
    if log_path is None:
        base = pathlib.Path("/ws/log") if pathlib.Path("/ws/log").is_dir() else pathlib.Path.cwd()
        log_path = base / f"sine_{dt.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    print(f"中心 {center:+.4f} rad、振幅 {args.amplitude:.3f} rad、{args.frequency} Hz、"
          f"振幅一定 {args.cycles:g} 周期 + 立ち上げ・立ち下げ 各 {args.ramp_cycles:g} 周期（{traj.duration:.1f} 秒）")
    if args.friction_static != args.friction_ff:
        friction = (f"摩擦の打ち消し Stribeck（静止 {args.friction_static:g} → 動 {args.friction_ff:g} N·m、"
                    f"vs {args.stribeck_velocity:g} rad/s、切り替え幅 {args.friction_velocity:g} rad/s）")
    else:
        friction = f"摩擦の打ち消し {args.friction_ff:g} N·m（切り替え幅 {args.friction_velocity:g} rad/s）"
    print(f"Kp={args.kp} Kd={args.kd}、{friction}、最大速度 {peak_velocity:.2f} rad/s、"
          f"送信 {args.rate:g} Hz、記録 {log_path}")

    rows = []
    period = 1.0 / args.rate
    t0 = time.monotonic()
    next_tick = t0
    last_print = -1.0
    aborted = None
    try:
        while True:
            elapsed = time.monotonic() - t0
            if elapsed > traj.duration + args.hold:
                break
            rclpy.spin_once(node, timeout_sec=0.0)
            if time.monotonic() - node.stamp > 0.2:
                aborted = "/joint_states が 0.2 秒以上途切れた"
                break
            p_des, v_des = traj.at(elapsed)
            ff = friction_torque(v_des, args.friction_ff, args.friction_velocity,
                                 args.friction_static, args.stribeck_velocity)
            est = args.kp * abs(p_des - node.position) + abs(ff)
            if est > args.max_torque:
                aborted = f"位置の誤差が大きい（Kp×誤差 + 上乗せ = {est:.2f} N·m > {args.max_torque}）"
                break
            node.send(p_des, v_des, args.kp, args.kd, ff)
            rows.append({"t": elapsed, "target": p_des, "target_velocity": v_des, "friction_ff": ff,
                         "position": node.position, "velocity": node.velocity, "effort": node.effort})
            if elapsed - last_print >= 0.5:
                print(f"  t={elapsed:5.2f}s  目標 {p_des:+.4f}  位置 {node.position:+.4f}  "
                      f"誤差 {node.position - p_des:+.4f}  トルク {node.effort:+.2f}")
                last_print = elapsed
            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        aborted = "Ctrl+C"

    rclpy.spin_once(node, timeout_sec=0.05)
    if aborted or args.release:
        for _ in range(5):
            node.send(node.position, 0.0, 0.0, 0.0)
            time.sleep(0.02)

    try:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else ["t"])
            writer.writeheader()
            writer.writerows(rows)
        print(f"\n記録: {log_path}（{len(rows)} 行）")
    except OSError as e:
        print(f"\n[注意] CSV を保存できない: {e}")

    if aborted:
        print(f"[中止] {aborted}。Kp=Kd=0 を送って力を抜いた")
    else:
        summarize(rows, traj)
    print(f"終了時: 位置 {node.position:+.4f} rad"
          + ("、力を抜いた" if (aborted or args.release) else "、中心位置で保持中"))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
