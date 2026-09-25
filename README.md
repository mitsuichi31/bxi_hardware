# bxi_hardware

BXI MIT制御アクチュエータ向けの共通ROS 2 `SystemInterface`です。四脚・双腕で同じrevisionを利用し、機体固有のjoint割当は各bringup側のYAMLに置きます。

## 実装状態

- MIT command/feedback codec: `implemented`、既知fixtureでunit test済み。温度は実機の応答（2026-09-25）で換算式を確認済み
- enable/disable/save-zeroとcommand/feedback: `implemented`、mockおよびvcan emulatorで検証済み
- 連続timeout時のlatched safe-stop: `implemented`、vcanで検証済み
- CAN FD（`can_frame_format: classic / fd / fd_brs`、バス単位）: `implemented`、vcanで検証済み
- 温度センサー異常（範囲下限への張り付き）の除外と警告: `implemented`、mockで検証済み
- 実モーターとの通信: 1軸で確認済み（2026-09-25）。多軸・長時間・異常注入（通信断など）のHILは未実施
- 実機の前提確認ツール `bxi_probe`: `implemented`、実機で実行済み（2026-09-25）。enable/disable特殊フレームへの応答とMotor Modeの切り替え、応答時間（約1.3 ms）を確認。結果は[`docs/PROTOCOL_AND_SAFETY.md`](docs/PROTOCOL_AND_SAFETY.md)
- 1軸試験の設定（`hil/`）: `implemented`、実機で実行済み（2026-09-25、classic 200 Hz）。enable / 位置・Kp・Kd指令 / 脱力 / 温度センサー異常の除外を確認。手順と結果は [`hil/README.md`](hil/README.md)
- 1軸のSIN波試験（`hil/single_joint/sine_command.py`、目標速度と摩擦を打ち消すトルク（Stribeckモデル）の上乗せも指令）: `implemented`。実機で打ち消しなし・一定値の上乗せを実行（stick-slipと改善を確認）。Stribeckモデルは実機では未実行
- error frame/run-mode切替frame: 取得済み仕様に定義がないため推測実装していない

`implemented`は実機検証済みを意味しません。実機への通電は独立した安全レビュー後に限ります。

## Dev Container test

```bash
ip link add dev vcan0 type vcan
ip link set vcan0 mtu 72        # CAN FD（vcanの既定値も72だが明示する）
ip link set up vcan0
ip link add dev vcan1 type vcan # classic only (FD open must be refused)
ip link set vcan1 mtu 16
ip link set up vcan1
export BXI_VCAN_IFACE=vcan0
export BXI_VCAN_CLASSIC_IFACE=vcan1
python3 scripts/validate_config.py test/config/valid.yaml
colcon build
colcon test
colcon test-result --verbose
```

vcan作成にはDev Containerの`NET_ADMIN` capabilityを使用します。
