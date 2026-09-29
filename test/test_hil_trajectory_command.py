"""Exercise the command against a simulated ROS service/status provider (no CAN)."""

import os
import pathlib
import math
import signal
import subprocess
import sys
import threading
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import UInt32
from std_srvs.srv import Trigger

from bxi_hardware.msg import HilTrajectoryStatus as Status
from bxi_hardware.srv import StartHilTrajectory


SCRIPT = pathlib.Path(__file__).parents[1] / "hil/single_joint/hil_trajectory.py"


class SimulatedController(Node):
    def __init__(self, *, finish=True, start=True, run_seconds=0.15, fail_writes=False):
        super().__init__("fake_hil_trajectory")
        self.status = Status()
        self.status.state = Status.IDLE
        self.status.position = 0.03
        self.status.record_state = Status.RECORD_NONE
        self.start_request = None
        self.started_at = 0.0
        self.released_at = 0.0
        self.run_seconds = run_seconds
        self.fail_writes = fail_writes
        self.abort_reason = ""
        self.abort_after_release = False
        self.finish_unapplied = False
        self.start_delay = 0.0
        self.finish_delay = 0.0
        self.in_start = False
        self.in_finish = False
        self.stops = 0
        self.csv_path = "/tmp/simulated_trajectory.csv"
        self.stop_unapplied = False
        self.pub = self.create_publisher(Status, "/hil_trajectory/status", 10)
        self.create_subscription(UInt32, "/hil_trajectory/keepalive", self.keepalive, 10)
        if start:
            self.create_service(StartHilTrajectory, "/hil_trajectory/start", self.start)
        if finish:
            self.create_service(Trigger, "/hil_trajectory/finish", self.finish)
        self.create_service(Trigger, "/hil_trajectory/stop", self.stop)
        self.create_timer(0.02, self.tick)

    def start(self, request, response):
        self.in_start = True
        if self.start_delay:
            time.sleep(self.start_delay)
        self.start_request = request
        self.status.state = Status.ARMED
        self.status.applied_request_id = 1
        self.status.record_state = Status.RECORD_RECORDING
        response.accepted = True
        response.request_id = 1
        self.in_start = False
        return response

    def keepalive(self, msg):
        if msg.data == 1 and self.status.state == Status.ARMED:
            self.status.state = Status.RUNNING
            self.started_at = time.monotonic()

    def finish(self, _request, response):
        self.in_finish = True
        if self.finish_delay:
            time.sleep(self.finish_delay)
        response.success = self.status.state == Status.HOLDING
        if response.success and not self.finish_unapplied:
            self.release("finish")
        self.in_finish = False
        return response

    def stop(self, _request, response):
        self.stops += 1
        if not self.stop_unapplied:
            self.status.applied_stop_count += 1
            self.release("stop")
        response.success = True
        return response

    def release(self, reason):
        self.status.state = Status.RELEASED
        self.status.end_reason = reason
        self.released_at = time.monotonic()

    def tick(self):
        now = time.monotonic()
        if self.status.state == Status.RUNNING:
            self.status.elapsed = now - self.started_at
            if self.fail_writes and self.status.elapsed > 0.08:
                self.status.write_failures = 1
            if self.abort_reason and self.status.elapsed > 0.08:
                self.status.state = Status.ABORTED
                self.status.end_reason = self.abort_reason
            if self.status.elapsed > self.run_seconds:
                if self.start_request.release:
                    self.release("release")
                else:
                    self.status.state = Status.HOLDING
        if self.released_at and now - self.released_at > 0.55:
            self.status.record_state = Status.RECORD_WRITTEN
            self.status.csv_path = self.csv_path
        if self.abort_after_release and self.released_at and now - self.released_at > 0.2:
            self.status.state = Status.ABORTED
            self.status.end_reason = "late fault"
        self.pub.publish(self.status)


@pytest.fixture
def controller():
    # Use a separate domain from any real controller. Every case gets a fresh ROS graph.
    previous = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 70)
    rclpy.init()
    node = SimulatedController()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    yield node
    executor.shutdown()
    worker.join(timeout=2)
    node.destroy_node()
    rclpy.shutdown()
    if previous is None:
        os.environ.pop("ROS_DOMAIN_ID", None)
    else:
        os.environ["ROS_DOMAIN_ID"] = previous


def command(args, timeout=9):
    return subprocess.run([sys.executable, str(SCRIPT), *args], capture_output=True, text=True,
                          timeout=timeout, env=os.environ.copy())


def test_finish_and_csv(controller):
    result = command(["--hold", "0.1", "hold"])
    assert result.returncode == 0, result.stdout + result.stderr
    assert "終了時: 終了で力を抜いた" in result.stdout
    assert "CSV: /tmp/simulated_trajectory.csv" in result.stdout
    assert controller.stops == 0


def test_release(controller):
    result = command(["--release", "ramp", "0.2"])
    assert result.returncode == 0, result.stdout + result.stderr
    assert "終了時: release で力を抜いた" in result.stdout


def test_sine_summary_uses_saved_controller_csv(controller, tmp_path):
    path = tmp_path / "trajectory.csv"
    controller.csv_path = str(path)
    with path.open("w", encoding="utf-8") as output:
        output.write("#run_id=1\n")
        output.write("elapsed_s,state,target_position,target_velocity,position,velocity\n")
        for index in range(1001):
            t = 2.0 + index * 0.01
            angle = math.pi * t
            target = 0.03 + 0.1 * math.sin(angle)
            velocity = 0.08 * math.pi * math.cos(angle - math.pi / 6)
            position = 0.03 + 0.08 * math.sin(angle - math.pi / 6)
            output.write(f"{t},2,{target},{0.1 * math.pi * math.cos(angle)},"
                         f"{position},{velocity}\n")
    result = command(["--release", "sine"])
    assert result.returncode == 0, result.stdout + result.stderr
    assert "振幅比 0.800" in result.stdout
    assert "位相遅れ 30.0°" in result.stdout


def test_late_abort_cannot_be_mistaken_for_saved_release(controller):
    controller.abort_after_release = True
    result = command(["--release", "hold"])
    assert result.returncode != 0
    assert "[中止]" in result.stderr
    assert "CSV の保存" in result.stderr


def test_write_failure_is_not_success(controller):
    controller.run_seconds = 1.0
    controller.fail_writes = True
    result = command(["hold"])
    assert result.returncode != 0
    assert "[中止]" in result.stderr
    assert "書き込み失敗" in result.stderr
    assert controller.stops >= 1


def test_missing_finish_response_is_not_success(controller):
    # Remove the service by destroying and replacing the fixture node before a command starts.
    for service in list(controller.services):
        if service.srv_name == "/hil_trajectory/finish":
            controller.destroy_service(service)
    result = command(["hold"])
    assert result.returncode != 0
    assert "~/finish" in result.stderr
    assert controller.stops >= 1


def test_finish_accepted_but_not_applied_is_not_success(controller):
    controller.finish_unapplied = True
    result = command(["hold"])
    assert result.returncode != 0
    assert "終了による脱力" in result.stderr
    assert controller.stops >= 1


def test_controller_abort_is_not_success(controller):
    controller.abort_reason = "lease expired"
    result = command(["hold"])
    assert result.returncode != 0
    assert "[中止]" in result.stderr
    assert "lease expired" in result.stderr


def test_missing_start_response_does_not_move(controller):
    for service in list(controller.services):
        if service.srv_name == "/hil_trajectory/start":
            controller.destroy_service(service)
    result = command(["hold"])
    assert result.returncode != 0
    assert "~/start" in result.stderr
    assert controller.start_request is None


def test_nonfinite_argument_is_rejected_before_ros_start(controller):
    result = command(["--kp", "nan", "hold"])
    assert result.returncode != 0
    assert "有限でない値: --kp" in result.stderr
    assert controller.start_request is None


def test_stale_host_heartbeat_is_not_success(controller, tmp_path):
    heartbeat = tmp_path / "heartbeat"
    heartbeat.touch()
    old = time.time() - 10
    os.utime(heartbeat, (old, old))
    result = command(["--heartbeat-file", str(heartbeat), "hold"])
    assert result.returncode != 0
    assert "心拍" in result.stderr
    assert controller.start_request is None


def test_sigint_requests_stop(controller):
    controller.run_seconds = 5.0
    process = subprocess.Popen([sys.executable, str(SCRIPT), "hold"], stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, text=True, env=os.environ.copy())
    try:
        time.sleep(0.7)
        process.send_signal(signal.SIGINT)
        stdout, stderr = process.communicate(timeout=5)
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()
    assert process.returncode != 0, stdout + stderr
    assert "[中止]" in stderr
    assert controller.stops >= 1


def test_sigint_while_waiting_for_csv_is_not_success(controller):
    process = subprocess.Popen([sys.executable, str(SCRIPT), "--release", "hold"],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                               env=os.environ.copy())
    try:
        deadline = time.monotonic() + 4
        while controller.status.state != Status.RELEASED and time.monotonic() < deadline:
            time.sleep(0.01)
        assert controller.status.state == Status.RELEASED
        assert controller.status.record_state == Status.RECORD_RECORDING
        process.send_signal(signal.SIGINT)
        stdout, stderr = process.communicate(timeout=5)
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()
    assert process.returncode != 0, stdout + stderr
    assert "[中止]" in stderr


@pytest.mark.parametrize("boundary", ["start", "finish"])
def test_sigterm_during_service_response_is_not_success(controller, boundary):
    if boundary == "start":
        controller.start_delay = 0.4
    else:
        controller.finish_delay = 0.4
    process = subprocess.Popen([sys.executable, str(SCRIPT), "hold"], stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, text=True, env=os.environ.copy())
    try:
        deadline = time.monotonic() + 4
        while not getattr(controller, "in_" + boundary) and time.monotonic() < deadline:
            time.sleep(0.01)
        assert getattr(controller, "in_" + boundary)
        process.send_signal(signal.SIGTERM)
        stdout, stderr = process.communicate(timeout=5)
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()
    assert process.returncode != 0, stdout + stderr
    assert "[中止]" in stderr


def test_disconnect_during_run_requests_stop(controller, tmp_path):
    controller.run_seconds = 5.0
    heartbeat = tmp_path / "heartbeat"
    heartbeat.touch()
    process = subprocess.Popen([sys.executable, str(SCRIPT), "--heartbeat-file", str(heartbeat),
                                "hold"], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               text=True, env=os.environ.copy())
    try:
        time.sleep(0.7)
        old = time.time() - 10
        os.utime(heartbeat, (old, old))
        stdout, stderr = process.communicate(timeout=5)
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()
    assert process.returncode != 0, stdout + stderr
    assert "心拍" in stderr
    assert controller.stops >= 1


def test_missing_stop_response_remains_failure(controller):
    controller.run_seconds = 5.0
    for service in list(controller.services):
        if service.srv_name == "/hil_trajectory/stop":
            controller.destroy_service(service)
    process = subprocess.Popen([sys.executable, str(SCRIPT), "hold"], stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, text=True, env=os.environ.copy())
    try:
        time.sleep(0.7)
        process.send_signal(signal.SIGINT)
        stdout, stderr = process.communicate(timeout=5)
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()
    assert process.returncode != 0, stdout + stderr
    assert "停止の確認ができない" in stderr


def test_stop_response_without_application_is_not_confirmed(controller):
    controller.run_seconds = 5.0
    controller.stop_unapplied = True
    process = subprocess.Popen([sys.executable, str(SCRIPT), "hold"], stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, text=True, env=os.environ.copy())
    try:
        time.sleep(0.7)
        process.send_signal(signal.SIGINT)
        stdout, stderr = process.communicate(timeout=5)
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()
    assert process.returncode != 0, stdout + stderr
    assert "停止の確認ができない" in stderr
    assert controller.stops >= 1
