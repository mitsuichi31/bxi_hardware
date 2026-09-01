# bxi_hardware

BXI MIT制御アクチュエータ向けの共通ROS 2 `SystemInterface`です。四脚・双腕で同じrevisionを利用し、機体固有のjoint割当は各bringup側のYAMLに置きます。

## 実装状態

- MIT command/feedback codec: `implemented`、既知fixtureでunit test済み
- enable/disable/save-zeroとcommand/feedback: `implemented`、mockおよびvcan emulatorで検証済み
- 連続timeout時のlatched safe-stop: `implemented`、vcanで検証済み
- 実モーターHIL: 未実施
- error frame/run-mode切替frame: 取得済み仕様に定義がないため推測実装していない

`implemented`は実機検証済みを意味しません。実機への通電は独立した安全レビュー後に限ります。

## Dev Container test

```bash
ip link add dev vcan0 type vcan
ip link set up vcan0
export BXI_VCAN_IFACE=vcan0
python3 scripts/validate_config.py test/config/valid.yaml
colcon build
colcon test
colcon test-result --verbose
```

vcan作成にはDev Containerの`NET_ADMIN` capabilityを使用します。
