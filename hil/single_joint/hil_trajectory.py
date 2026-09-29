#!/usr/bin/env python3
"""HilTrajectoryController に1回の hold / ramp / sine を要求する。

軌道は controller_manager の update() 内で作られる。このプロセスは要求番号付き
keepalive を送り、脱力と CSV の保存を確認する。異常時は [中止] と終了コード1を返す。
実機では HIL ランナーが終了コードを見て launch を止め、disable を確認する。
"""

import argparse
import csv
import math
import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import UInt32
from std_srvs.srv import Trigger

from bxi_hardware.msg import HilTrajectoryStatus
from bxi_hardware.srv import StartHilTrajectory


class Abort(Exception):
    pass


def _solve_3x3(matrix, values):
    """Solve the three normal equations for a sine/cosine/constant fit."""
    a = [list(row) + [value] for row, value in zip(matrix, values)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(a[row][col]))
        if abs(a[pivot][col]) < 1e-12:
            raise Abort("SIN 波の追従集計で係数を求められない")
        a[col], a[pivot] = a[pivot], a[col]
        scale = a[col][col]
        for k in range(col, 4):
            a[col][k] /= scale
        for row in range(3):
            if row == col:
                continue
            factor = a[row][col]
            for k in range(col, 4):
                a[row][k] -= factor * a[col][k]
    return tuple(a[row][3] for row in range(3))


def summarize_sine_csv(path, request):
    """Summarize the constant-amplitude interval from the controller's saved CSV."""
    omega = 2.0 * math.pi * request.frequency
    flat_start = request.ramp_cycles / request.frequency
    flat_end = flat_start + request.cycles / request.frequency
    normal = [[0.0] * 3 for _ in range(3)]
    rhs = [0.0] * 3
    count = 0
    square_error = 0.0
    max_error = 0.0
    moving = 0
    stopped = 0
    max_speed = 0.0
    centers = 0.0
    with open(path, newline="", encoding="utf-8") as source:
        if source.readline().startswith("#") is False:
            raise Abort("CSV に実行条件の行がない")
        for row in csv.DictReader(source):
            t = float(row["elapsed_s"])
            if not (flat_start <= t <= flat_end and int(row["state"]) ==
                    HilTrajectoryStatus.RUNNING):
                continue
            target = float(row["target_position"])
            target_velocity = float(row["target_velocity"])
            position = float(row["position"])
            velocity = float(row["velocity"])
            if not all(map(math.isfinite, (target, target_velocity, position, velocity))):
                raise Abort("SIN 波の追従集計に有限でない値がある")
            basis = (1.0, math.sin(omega * t), math.cos(omega * t))
            for i in range(3):
                rhs[i] += basis[i] * position
                for j in range(3):
                    normal[i][j] += basis[i] * basis[j]
            centers += target - request.amplitude * basis[1]
            error = position - target
            square_error += error * error
            max_error = max(max_error, abs(error))
            max_speed = max(max_speed, abs(velocity))
            if abs(target_velocity) > 0.1 * request.amplitude * omega:
                moving += 1
                stopped += abs(velocity) < 0.02
            count += 1
    if count < 10:
        print("振幅一定の区間のデータが少なく、追従を集計できない", flush=True)
        return
    center, sine, cosine = _solve_3x3(normal, rhs)
    ratio = math.hypot(sine, cosine) / request.amplitude
    lag_deg = -math.degrees(math.atan2(cosine, sine))
    lag_ms = lag_deg / 360.0 / request.frequency * 1000.0
    print(f"振幅一定区間 {count} 周期: 誤差 RMS {math.sqrt(square_error / count):.4f} rad、"
          f"最大 {max_error:.4f} rad", flush=True)
    print(f"振幅比 {ratio:.3f}、位相遅れ {lag_deg:.1f}°（{lag_ms:.0f} ms）、"
          f"中心のずれ {center - centers / count:+.4f} rad", flush=True)
    if moving:
        print(f"止まっている時間 {100.0 * stopped / moving:.0f} %、"
              f"軸の最大速度 {max_speed:.2f} rad/s", flush=True)


class StopRequest:
    """Signal handler only records the request; ROS work stays in the main loop."""

    def __init__(self):
        self.reason = None
        self.late = None
        self.closed = False
        signal.signal(signal.SIGINT, self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

    def _on_signal(self, signum, _frame):
        name = "Ctrl+C" if signum == signal.SIGINT else "SIGTERM"
        if self.closed:
            self.late = self.late or name
        else:
            self.reason = self.reason or name

    def close(self):
        self.closed = True
        return self.reason


class TrajectoryClient(Node):
    def __init__(self, controller, heartbeat_file, stop):
        super().__init__("bxi_hil_trajectory_command")
        prefix = "/" + controller.strip("/")
        self.start_client = self.create_client(StartHilTrajectory, prefix + "/start")
        self.finish_client = self.create_client(Trigger, prefix + "/finish")
        self.stop_client = self.create_client(Trigger, prefix + "/stop")
        self.keepalive_pub = self.create_publisher(UInt32, prefix + "/keepalive", 10)
        self.create_subscription(HilTrajectoryStatus, prefix + "/status", self._on_status, 10)
        self.heartbeat_file = heartbeat_file
        self.stop = stop
        self.status = None
        self.status_at = 0.0
        self.id = 0
        self.baseline_write_failures = 0
        self.monitor_failures = False
        self.next_keepalive = 0.0
        self.last_print = 0.0

    def _on_status(self, msg):
        self.status = msg
        self.status_at = time.monotonic()

    def _heartbeat_ok(self):
        if not self.heartbeat_file:
            return True
        try:
            return time.time() - os.stat(self.heartbeat_file).st_mtime <= 3.0
        except OSError:
            return False

    def poll(self, check=True):
        now = time.monotonic()
        if self.id and now >= self.next_keepalive:
            self.keepalive_pub.publish(UInt32(data=self.id))
            self.next_keepalive = now + 0.05
        rclpy.spin_once(self, timeout_sec=0.02)
        if not check:
            return
        if self.stop.reason:
            raise Abort(self.stop.reason)
        if not self._heartbeat_ok():
            raise Abort("ホストの心拍が3秒以上途切れた")
        if self.status is not None:
            if self.monitor_failures and self.status.write_failures > self.baseline_write_failures:
                raise Abort("command interface の書き込み失敗")
            if self.id and self.status.applied_request_id == self.id and self.status.state == HilTrajectoryStatus.ABORTED:
                raise Abort("controller の中止: " + self.status.end_reason)
            if self.id and self.status.applied_request_id == self.id and self.status.end_reason == "stop":
                raise Abort("controller の停止要求")
            if now - self.status_at > 0.5:
                raise Abort("~/status が0.5秒届かない")

    def until(self, condition, timeout, what, check=True):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.poll(check=check)
            if condition():
                return
        raise Abort(what + " が時間内に確認できない")

    def call(self, client, request, timeout, name, check=True):
        self.until(lambda: client.service_is_ready(), timeout, name + " サービス", check)
        future = client.call_async(request)
        self.until(future.done, timeout, name + " の応答", check)
        try:
            return future.result()
        except Exception as exc:
            raise Abort(name + " の呼び出しに失敗: " + str(exc)) from exc

    def run(self, request):
        self.until(lambda: self.status is not None and self.keepalive_pub.get_subscription_count() > 0,
                   2.0, "~/status と keepalive の接続")
        if self.status.state != HilTrajectoryStatus.IDLE:
            raise Abort("controller が待機中ではない")
        if not math.isfinite(self.status.position):
            raise Abort("現在位置が有限でない")
        self.baseline_write_failures = self.status.write_failures
        self.monitor_failures = True
        print(f"今の位置 {self.status.position:+.4f} rad", flush=True)
        if request.kind == StartHilTrajectory.Request.SINE:
            print(f"中心 {self.status.position:+.4f} rad", flush=True)
        response = self.call(self.start_client, request, 2.0, "~/start")
        if not response.accepted or response.request_id == 0:
            raise Abort("~/start が拒否した: " + response.message)
        self.id = response.request_id
        self.next_keepalive = 0.0
        self.until(lambda: self.status.applied_request_id == self.id and
                   self.status.state in (HilTrajectoryStatus.RUNNING, HilTrajectoryStatus.HOLDING,
                                         HilTrajectoryStatus.RELEASED),
                   1.0, "開始の適用")
        print(f"要求番号 {self.id}: 開始を確認", flush=True)
        # The controller validates the duration; the extra time only bounds a stalled run.
        deadline = time.monotonic() + 65.0
        while True:
            self.poll()
            s = self.status
            if s.applied_request_id != self.id:
                raise Abort("要求番号が変わった")
            if time.monotonic() >= deadline:
                raise Abort("実行が65秒以内に終わらない")
            if time.monotonic() - self.last_print >= 1.0:
                print(f"t={s.elapsed:.2f} s 位置={s.position:+.4f} rad", flush=True)
                self.last_print = time.monotonic()
            if s.state == HilTrajectoryStatus.HOLDING:
                response = self.call(self.finish_client, Trigger.Request(), 1.0, "~/finish")
                if not response.success:
                    raise Abort("~/finish が拒否した: " + response.message)
                self.until(lambda: self.status.applied_request_id == self.id and
                           self.status.state == HilTrajectoryStatus.RELEASED and
                           self.status.end_reason == "finish", 1.0, "終了による脱力")
                expected = "finish"
                break
            if s.state == HilTrajectoryStatus.RELEASED:
                if s.end_reason != "release":
                    raise Abort("予期しない終了理由: " + s.end_reason)
                expected = "release"
                break
        response_id = self.id
        self.id = 0  # Stop keepalive only after release has been reported.
        if self.status.release_write_pending:
            raise Abort("command interface への脱力値の書き込みが未完了")
        self.until(lambda: self.status.applied_request_id == response_id and
                   self.status.state == HilTrajectoryStatus.RELEASED and
                   self.status.end_reason == expected and
                   not self.status.release_write_pending and
                   self.status.record_state == HilTrajectoryStatus.RECORD_WRITTEN and
                   bool(self.status.csv_path) and self.status.dropped_cycles == 0,
                   5.0, "CSV の保存")
        print("CSV: " + self.status.csv_path, flush=True)
        if request.kind == StartHilTrajectory.Request.SINE:
            summarize_sine_csv(self.status.csv_path, request)
        return expected

    def stop_run(self):
        # Best effort. Keepalive stops before the first wait, so the lease expires even when the
        # service never answers; the saved request id still confirms which run was stopped.
        current_id = self.id
        self.id = 0
        try:
            previous_stops = self.status.applied_stop_count if self.status is not None else 0
            response = self.call(self.stop_client, Trigger.Request(), 1.0, "~/stop", check=False)
            if not response.success:
                raise Abort("~/stop が拒否した")
            self.until(lambda: self.status is not None and
                       self.status.applied_stop_count > previous_stops and
                       (current_id == 0 or self.status.applied_request_id == current_id) and
                       self.status.state in (HilTrajectoryStatus.IDLE, HilTrajectoryStatus.RELEASED,
                                             HilTrajectoryStatus.ABORTED) and
                       not self.status.release_write_pending,
                       1.0, "停止と脱力", check=False)
            print("~/stop の適用と脱力値の書き込みを確認", file=sys.stderr, flush=True)
        except Exception as exc:
            print("停止の確認ができない: " + str(exc), file=sys.stderr, flush=True)


def make_request(args):
    request = StartHilTrajectory.Request()
    request.kind = {"hold": request.HOLD, "ramp": request.RAMP, "sine": request.SINE}[args.kind]
    request.target = getattr(args, "target", 0.0)
    request.speed = args.speed
    request.amplitude = args.amplitude
    request.frequency = args.frequency
    request.cycles = args.cycles
    request.ramp_cycles = args.ramp_cycles
    request.friction_kinetic = args.friction_kinetic
    request.friction_static = args.friction_static
    request.friction_velocity = args.friction_velocity
    request.stribeck_velocity = args.stribeck_velocity
    request.kp = args.kp
    request.kd = args.kd
    request.hold = args.hold
    request.release = args.release
    request.lease = args.lease
    return request


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--controller", default="hil_trajectory", help="controller 名")
    ap.add_argument("--heartbeat-file", help="ホストの心拍ファイル（3秒で途切れと判定）")
    ap.add_argument("--kp", type=float, default=10.0)
    ap.add_argument("--kd", type=float, default=2.0)
    ap.add_argument("--hold", type=float, default=1.0)
    ap.add_argument("--lease", type=float, default=0.5)
    ap.add_argument("--release", action="store_true")
    ap.add_argument("--speed", type=float, default=0.3)
    ap.add_argument("--amplitude", type=float, default=0.1)
    ap.add_argument("--frequency", type=float, default=0.5)
    ap.add_argument("--cycles", type=float, default=5.0)
    ap.add_argument("--ramp-cycles", type=float, default=1.0)
    ap.add_argument("--friction-kinetic", type=float, default=0.0)
    ap.add_argument("--friction-static", type=float, default=-1.0)
    ap.add_argument("--friction-velocity", type=float, default=0.05)
    ap.add_argument("--stribeck-velocity", type=float, default=0.2)
    modes = ap.add_subparsers(dest="kind", required=True)
    modes.add_parser("hold")
    modes.add_parser("ramp").add_argument("target", type=float)
    modes.add_parser("sine")
    args = ap.parse_args(argv)
    for key, value in vars(args).items():
        if isinstance(value, float) and not math.isfinite(value):
            ap.error("有限でない値: --" + key.replace("_", "-"))
    if not args.controller.strip("/"):
        ap.error("controller 名が空")
    return args


def main(argv=None):
    args = parse_args(argv)
    stop = StopRequest()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    code = 1
    try:
        node = TrajectoryClient(args.controller, args.heartbeat_file, stop)
        reason = node.run(make_request(args))
        if stop.close():
            raise Abort(stop.reason)
        text = "終了で力を抜いた" if reason == "finish" else "release で力を抜いた"
        print(f"終了時: {text}", flush=True)
        code = 0
    except Exception as exc:
        if node is not None:
            node.stop_run()
        print("[中止] " + str(exc), file=sys.stderr, flush=True)
    finally:
        stop.close()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    if stop.late:
        print("[終了後の合図] " + stop.late, file=sys.stderr, flush=True)
        code = 1
    return code


if __name__ == "__main__":
    raise SystemExit(main())
