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
既定の `classic` では、can0 が fd on でも MIT パケットは通常の CAN（8 バイト）で送る。

`joint1_command` の指令は `data = [position, velocity, kp, kd, effort]`（`controllers.yaml` の `interface_names`）。
velocity は MIT の目標速度で、0 のままだと Kd の項が動きを妨げるため、`ramp_command.py` / `sine_command.py` は軌道の速度を送る。
effort は MIT のトルク（フィードフォワード）で、`sine_command.py --friction-ff` の摩擦の打ち消しに使う（`ramp_command.py` は常に 0）。

## 2b. SIN 波で動かす（sine_command.py）

手順 2 の launch を起動したまま、端末2で実行する。

```bash
ros2 run bxi_hardware ramp_command.py 0.0 --kp 20 --kd 1              # 中心（例: 0 rad）へ移動
ros2 run bxi_hardware sine_command.py --amplitude 0.1 --frequency 0.5 --cycles 5
ros2 run bxi_hardware sine_command.py --amplitude 0.2 --frequency 1.0 --kp 30 --max-kp 40 --release
```

- 中心は実行時の位置。振幅は最初と最後の 1 周期（`--ramp-cycles`）で 0 からなめらかに増減する。
- 実行前に止める条件: 中心 ± 振幅が `--lower` / `--upper`（既定 ±0.5 rad、`bxi_hardware.yaml` と合わせる）をはみ出す、
  最大速度（振幅 × 2π × 周波数）が `--max-velocity`（既定 2 rad/s）を超える、`--center` が今の位置と違う。
- 実行中に中止して力を抜く条件: `Kp × 誤差` が `--max-torque`（既定 5 N·m）を超える、`/joint_states` が 0.2 秒以上途切れる、Ctrl+C。
- 目標・位置・速度・トルクを `/ws/log/sine_<日時>.csv` に保存し、振幅一定の区間の追従誤差、振幅比、位相の遅れを表示する。
- 表示される位相の遅れには、ROS 内の遅れ（指令 → controller_manager → 状態 → `/joint_states`）が含まれる。
  mock_components で約 13 ms（200 Hz、1 Hz の SIN 波で 4.5°）。
- 静止摩擦（約 1 N·m）があるため、`Kp × 振幅` が小さいと、SIN 波の頂点付近で止まったり、動き出しが遅れたりする（stick-slip）。
  まず Kp 20、振幅 0.1 rad、0.5 Hz 程度から始める。
  2026-09-25 の実機（Kp 20、振幅 0.1 rad、0.5 Hz、打ち消しなし）では、目標が動いているのに止まっている時間が約 25 %、
  軸の最大速度は目標の 2 倍以上（0.7 rad/s）で、止まっては急に動く動きになった。
- 摩擦の打ち消し（Stribeck モデル）: 目標速度 v の向きにトルクを上乗せする。
  `effort = (Fc + (Fs − Fc) × exp(−|v| / vs)) × tanh(v / v0)`
  （Fc: 動摩擦 `--friction-ff`、Fs: 静止摩擦 `--friction-static`（省略時 = Fc、一定値の上乗せ）、
  vs: `--stribeck-velocity` 既定 0.2 rad/s、v0: 向きの切り替え幅 `--friction-velocity` 既定 0.05 rad/s）。
  実測速度ではなく目標速度を使うので、折り返し点で振動しにくい。上限は `--max-friction-ff`（既定 2 N·m）。
  大きすぎると、目標を追い越したり、折り返し点で押し戻されたりする。
  安全チェックは `Kp × 誤差 + |上乗せトルク|` で判定する。
- Stribeck モデルを使うときは、v0 を 0.02 程度に狭める。既定の 0.05 のままだと、低速で tanh が上乗せを削り、
  静止摩擦に近い大きさが出ない（Fc 0.6 / Fs 1.0 のとき、目標速度 0.05 rad/s で v0 0.05 なら 0.69、v0 0.02 なら 0.90 N·m）。
- 集計の「引っかかり」は、目標が動いているのに軸が止まっている時間の割合と軸の最大速度。打ち消しの効果の比較に使う。

```bash
ros2 run bxi_hardware sine_command.py --amplitude 0.3 --frequency 0.2 --cycles 5                    # 打ち消しなし（比較用）
ros2 run bxi_hardware sine_command.py --amplitude 0.3 --frequency 0.2 --cycles 5 --friction-ff 0.9  # 一定値
ros2 run bxi_hardware sine_command.py --amplitude 0.3 --frequency 0.2 --cycles 5 \
  --friction-ff 0.6 --friction-static 1.0 --friction-velocity 0.02                                   # Stribeck
```

### SIN 波の結果（2026-09-25、Kp 20 / Kd 1、classic 200 Hz）

| 条件 | 摩擦の打ち消し | 誤差 RMS / 最大 [rad] | 振幅比 | 位相の遅れ | 止まっている時間 |
|---|---|---|---|---|---|
| 振幅 0.1 rad、0.5 Hz | なし | 0.037 / 0.068 | 0.69 | 27.6° | 46 % |
| 〃 | 一定 0.6 N·m | 0.015 / 0.037 | 0.94 | 9.3° | 30 % |
| 〃 | 一定 0.9 N·m | 0.0075 / 0.020 | 1.00 | -2.8° | 19 % |
| 振幅 0.3 rad、0.2 Hz | なし | 0.035 / 0.064 | 0.94 | 8.4° | 25 % |
| 〃 | 一定 0.6 N·m | 0.012 / 0.036 | 0.99 | 2.0° | 17 % |
| 〃 | 一定 0.9 N·m | 0.0092 / 0.022 | 1.01 | -1.5° | 14 % |
| 〃（Kp 30） | なし | 0.026 / 0.050 | 0.97 | 6.5° | 19 % |
| 〃 | Stribeck（Fc 0.6 / Fs 1.0、v0 0.02） | 0.0090 / 0.033 | 1.00 | 1.0° | 15 % |
| 〃 | Stribeck（Fc 0.8 / Fs 1.0、v0 0.02） | 0.0093 / 0.025 | 1.01 | -0.8° | 15 % |
| 〃（Kp 40） | Stribeck（Fc 0.8 / Fs 1.0、v0 0.02） | 0.0058 / 0.018 | 1.00 | 0.2° | 10 % |

- 一定 0.9 N·m の上乗せで、追従誤差は 1/3〜1/5、振幅比はほぼ 1 になった。Kp を 30 に上げるより効果が大きい。
- 打ち消しなしの記録から: 動いている間のトルク（動摩擦）は ＋方向 +0.55 / −方向 −0.60 N·m、
  止まった状態から動き出す直前のトルク（静止摩擦）は平均 0.96、最大 1.31 N·m。
- 一定値では、0.9 N·m は動いている間に多すぎ（位相がわずかに進む）、0.6 N·m は動き出しに足りない。
  残った「止まっている時間」の多くは、折り返し後の低速（目標速度 0.1 rad/s 前後）で起きていた。これが Stribeck モデルを入れた理由。
- Stribeck（Fc 0.6 / Fs 1.0）では、低速（目標速度 0.04〜0.10 rad/s）で止まっている割合が 55 % → 35 % に減り、
  位相の進みもなくなった。一方、中速（0.1〜0.3 rad/s）では上乗せが一定 0.9 より小さく、止まっている割合が増えた
  （0.10〜0.20: 28 % → 37 %）。最大誤差は目標速度が最大の中心通過で出た。
  打ち消しなしの記録から出した動摩擦 0.6 N·m は、速く滑っている区間を多く含むため小さめの見積もりとみられる。
- Stribeck（Fc 0.8 / Fs 1.0）は、最大誤差を 0.025 rad に抑えつつ位相の遅れをほぼ 0 にした。ただし RMS（約 0.009 rad）と
  止まっている時間（14〜15 %）は一定 0.9 N·m と同程度で、上乗せの形の調整による改善はここで頭打ち。
  軸の最大速度は目標の 2 倍（0.76 rad/s）のままで、止まっては滑る動きが少し残る。次の手は Kp を上げること
  （止まっている間にたまる誤差を小さくし、滑る幅を縮める）と、減速機のガタ（ENC1 と ENC2 の差 約 0.6°）の影響の確認。
- Kp を 40 に上げると（Stribeck Fc 0.8 / Fs 1.0 のまま）、RMS 0.0058 rad、最大 0.018 rad（約 1.0°）、止まっている時間 10 % と、
  すべての指標がこれまでで最もよくなった（打ち消しなし Kp 20 の約 1/6）。軸の最大速度は 0.74 rad/s のままで、
  止まっては滑る動きは少し残る。安全チェックの値（Kp × 誤差 + 上乗せ）は最大約 1.7 N·m。
- 現時点の推奨: `--kp 40 --max-kp 40 --friction-ff 0.8 --friction-static 1.0 --friction-velocity 0.02`（Kd 1）。

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
