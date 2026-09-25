# 1軸の実機試験（ベンチ）

モーター1台（BXI7010-19、`can_id` 1）を PCAN の `can0` に直接つないで試験する。bxi_usb2can は使わない。

実機への通電と、モーターを動かす操作は、試験者が安全を確認してから行う。

## 0. 準備（ホスト）

```bash
# can0 を CAN FD 1 Mbps / 5 Mbps（サンプル点 0.8 / 0.8125、berr-reporting on）で up する。
# FD で起動しても、通常の CAN（can_frame_format: classic）はそのまま使える。
cd ~/devel/minibexkai_ros2_ws
./docker/can_setup.sh can0
ip -details -statistics link show can0      # state ERROR-ACTIVE を確認
```

`docker/can_setup.sh` はワークスペース（`minibexkai_ros2_ws`）側にある。引数を省略すると can0〜can3 をすべて up する。

- モーターの電源を入れ、MENU の状態（`STATE : 01 MENU`）にしておく。
- モーターの `can_timeout` は 0（通信が止まっても最後の指令を出し続ける）。**試験中にプロセスを強制終了すると、最後の指令が残る。** 止めるときは Ctrl+C で終了し、それでも止まらなければ電源を切る。
- 試験中は、ほかのプログラム（BXI_Tool、`can_motor.py` など）から同じバスに指令を送らない。

開発コンテナ（`network_mode: host` なので can0 が見える）でビルドする。
VS Code の Dev Container が起動済みなら、そのウィンドウのターミナル（ターミナル → 新しいターミナル）を使う。
ホストの端末から入るときは `cd ~/devel/minibexkai_ros2_ws/docker && docker compose exec minibex_dev bash`
（`docker compose run --rm` は別のコンテナを新しく起動してしまうので使わない）。

```bash
# コンテナ内（/ws）
colcon build --packages-select bxi_hardware
source install/setup.bash
```

`forward_command_controller` は、ワークスペースでソースからビルドしたものを使うこと（`ros2 pkg prefix forward_command_controller` が `/ws/install/...` を指す）。
apt 版は apt の `controller_interface` に合わせてビルドされていて、ワークスペースの fork の ros2_control と ABI が合わず、`joint1_command` の有効化で `ros2_control_node` が落ちる（2026-09-25 に発生）。
ワークスペース側の `infrastructure/apply_colcon_ignore.sh` でビルド対象に戻している。

## 1. 前提の確認（bxi_probe、モーターは力を出さない）

```bash
ros2 run bxi_hardware bxi_probe --iface can0 --can-id 1 --format classic --count 500 \
  | tee /ws/log/bxi_probe_classic_$(date +%Y%m%d_%H%M%S).log
# can0 が fd on なら、CAN FD でも確認する
ros2 run bxi_hardware bxi_probe --iface can0 --can-id 1 --format fd_brs --count 500 \
  | tee /ws/log/bxi_probe_fd_brs_$(date +%Y%m%d_%H%M%S).log
```

確認すること:

| 項目 | 見るところ | ドライバが前提にしていること |
|---|---|---|
| enable（FC）への応答 | `[2]` の「MIT応答」 | 応答がある（ないと on_activate の enable が失敗する） |
| Motor Mode に入るか | `[2]` の「状態」に `02 MOTOR` | enable で Motor Mode に入る |
| disable（FD）への応答 | `[4]` の「MIT応答」と「状態」 | 応答があり、MENU に戻る |
| 応答時間 | `[1]` `[3]` の最大値と timeout | `control_timeout_ms`（既定 3 ms）より十分短い |

## 2. ros2_control で動かす

```bash
# 端末1（コンテナ内）
ros2 launch bxi_hardware single_joint_hil.launch.py
```

起動ログで `Enable joint1 pos=... temp=...` が出て、温度が室温程度（約 30 ℃）であることを確認する。
NTC2 が異常なモーターでは `NTC2 (byte 7) reads -28.6 degC; treating it as a sensor fault` の警告が出る（NTC1 だけで温度を監視する）。

起動直後は指令が未入力なので Kp=Kd=0 で、モーターは力を出さない。

```bash
# 端末2（コンテナ内。VS Code のターミナルを「+」で追加するか、docker compose exec minibex_dev bash）
source install/setup.bash
ros2 topic echo /joint_states --once                     # 位置・温度の確認
ros2 run bxi_hardware ramp_command.py 0.0 --kp 10 --kd 2  # 今の位置付近で保持
ros2 run bxi_hardware ramp_command.py 0.2 --kp 10 --kd 2  # 約 11° 動かす
ros2 run bxi_hardware ramp_command.py 0.0 --kp 20 --kd 2 --release   # 戻して力を抜く
```

- 目標位置は 0.3 rad/s で少しずつ動く。`Kp × 誤差` が 5 N·m を超えるか、`/joint_states` が途切れると中止して力を抜く。
- 範囲は `bxi_hardware.yaml` の `lower` / `upper`（±0.5 rad）でクランプされる。
- 摩擦（約 0.9 N·m）があるため、目標の手前（誤差 ≈ 0.9 ÷ Kp rad）で止まる（2026-09-25 の結果）。
- 終了は端末1で Ctrl+C。ハードウェアの deactivate で disable（FD）が送られる。

CAN FD で動かすときは、`bxi_hardware.yaml` の `can_frame_format` を `fd` または `fd_brs` にする（can0 が fd on であること）。

## 3. 結果（2026-09-25、BXI7010-19、can0 classic、200 Hz）

`bxi_probe`（手順 1）:

| | enable（FC） | disable（FD） | 応答時間 中央 / 最大 | timeout |
|---|---|---|---|---|
| classic | 応答あり、`02 MOTOR` | 応答あり、`01 MENU` | 1.28〜1.29 / 1.44 ms | 0 / 1000 |
| FD+BRS | 応答あり、`02 MOTOR` | 応答あり、`01 MENU` | 1.23〜1.24 / 1.59 ms | 0 / 1000 |

ros2_control（手順 2）:

- on_activate で enable に成功（`Enable joint1 pos=0.044 temp=30.7`）。NTC2（-30 ℃）はセンサー異常として除外され、警告が 1 回出た。温度の状態インターフェースは NTC1 の 32.1 ℃。
- `joint_state_broadcaster` と `joint1_command` が active。指令を送るまではモーターは力を出さなかった。

| 指令 | 開始 → 終了位置 [rad] | 目標との差 | Kp × 差 [N·m] |
|---|---|---|---|
| `ramp_command.py 0.0 --kp 10` | +0.0437 → +0.0437（動かない） | +0.044 | 0.44 |
| `ramp_command.py 0.2 --kp 10` | +0.0433 → +0.1013 | -0.099（-5.7°） | 0.99 |
| `ramp_command.py 0.0 --kp 20 --release` | +0.1017 → +0.0605 | +0.061（+3.5°） | 1.21 |

- 位置指令、Kp / Kd の指令、`--release` での脱力がすべて ros2_control 経由で動作した。自動中止は起きなかった。
- 目標の手前で止まるのは静止摩擦による（Python から直接操作したときと同じ傾向）。止まったときの Kp × 差は 0.99〜1.21 N·m で、Python での結果（0.8〜0.9 N·m）より少し大きかった。動き出しが遅れて一気に動く（stick-slip）様子もある。
- 1 回目は Kp × 差が 0.44 N·m で、摩擦に負けて動かなかった。
