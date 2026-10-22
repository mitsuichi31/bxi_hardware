# 1軸の動作確認ガイド

モーター1台（BXI7010-19、`can_id` 1）を PCAN の `can0` につないで、動作を確かめるときの手順をまとめる。やりたいことに合う節だけを読めばよい。

設定の詳しい説明、測定の結果、経緯は [`README.md`](README.md)（このディレクトリ）に残してある。controller の違いは [`../docs/CONTROLLERS.md`](../docs/CONTROLLERS.md)。

## やりたいことから選ぶ

| やりたいこと | モーターは力を出すか | 節 |
|---|---|---|
| 今の位置・温度を見る | 出さない | [A. 状態を見る](#a-状態を見る力を出さない) |
| 手で指令を出して動かす（トピック経由の指令） | 出す | [B. 指令スクリプトで動かす](#b-指令スクリプトで動かすトピック経由) |
| 手で指令を出して動かす（RT 軌道 controller） | 出す | [C. RT 軌道 controller で動かす](#c-rt-軌道-controller-で動かす) |
| 記録と判定を残す正式な試験 | 出す | [D. 正式な試験（ランナー）](#d-正式な試験ランナー) |

**記録を残す試験は D を使う。** A〜C は、その場で動きを確かめるための手順で、自動の判定や停止の確認はしない。

## 始める前に（毎回）

### 安全

- 電源装置（OWON SPE6103）は 24 V・電流制限 5 A。**出力スイッチにすぐ手が届く位置にいる。**
- モーター本体が固定されていて、出力軸の周りに人・物がない。出力軸に重力で回る負荷（アーム、おもりなど）を付けない。
- ほかのプログラム（BXI_Tool など）から同じ `can0` に送らない。前の試験の端末（launch、`candump` など）は閉じる。
- モーターは `can_timeout=0` で、**通信が止まっても最後の指令を保持し続ける**（2026-10-21 に 200 ms にした FR の3台を除く。値は `scripts/bxi_can_timeout.py read` で確かめる）。 プロセスが異常終了したり、止め方が分からなくなったりしたら、**まず電源装置の出力を切る。**

### CAN（ホストの端末）

```bash
cd ~/devel/minibexkai_ros2_ws
./docker/can_setup.sh can0                    # CAN FD 1 Mbps / 5 Mbps、restart-ms 0 で up
ip -details -statistics link show can0         # state ERROR-ACTIVE、berr-counter tx 0 rx 0 を確かめる
```

### 開発コンテナ

```bash
cd ~/devel/minibexkai_ros2_ws/docker && docker compose up -d     # 止まっていれば起動
docker compose exec minibex_dev bash                              # コンテナに入る（run --rm は使わない）
```

コンテナの中では、毎回最初に次を実行する。ソースを変えたときだけ、`colcon build --packages-select bxi_hardware` でビルドする。

```bash
source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
```

以下の `H` は設定ファイルの場所。

```bash
H=/ws/install/bxi_hardware/share/bxi_hardware/hil/single_joint
```

## A. 状態を見る（力を出さない）

1 kHz の設定で起動し、指令を出さずに位置を読む。指令がない間、HWIF は Kp=Kd=0 を送るので、モーターは力を出さない。

端末1（コンテナ）：

```bash
ros2 launch bxi_hardware single_joint_hil.launch.py \
  config_file:=$H/bxi_hardware_stream_1khz.yaml controllers_file:=$H/controllers_1khz.yaml
```

- 起動ログ（端末1に流れる。見方は最後の「[起動ログの見方](#起動ログの見方)」）の `Enable joint1 pos=... temp=...` に、その時点の位置と温度が出る。
- `送受信分離を開始` と、2つの controller の `Configured and activated` が出れば起動できている。

端末2（コンテナ）：

```bash
ros2 topic echo /joint_states --once           # position が現在位置 [rad]
```

- 温度は `/joint_states` には入らない（`/dynamic_joint_states` か、起動ログの Enable 行で見る）。

止めるときは、端末1で Ctrl+C。ログに `joint1 disable (deactivate): acknowledged` が出たことを確かめる。

- launch の既定の設定（引数なし）は、古い 200 Hz・Classic の設定で、起動の応答待ち 3 ms が短く失敗することがある。上のように 1 kHz の設定を指定する。
- CAN の応答だけを確かめたいときは、ros2_control を使わずに `bxi_probe` でも見られる（[`README.md`](README.md) の「1. 前提の確認」）。

## 指令の引数（B・C 共通）

| 引数 | 意味・単位 |
|---|---|
| `ramp` の目標位置（`ramp_command.py <目標>`、`hil_trajectory.py ... ramp <目標>`） | **絶対位置 [rad]。現在位置からの移動量ではない** |
| `--speed` | 目標を動かす速さ [rad/s] |
| `--amplitude` | 中心から片側への振幅 [rad]。**往復の幅はその2倍**（振幅 0.1 rad なら中心 ±0.1 rad） |
| `--frequency` | 周波数 [Hz] |
| `--cycles` | 振幅一定で動かす周期の数（単位なし） |
| `--ramp-cycles` | 振幅を立ち上げる周期の数と、立ち下げる周期の数（それぞれ。単位なし） |
| `--hold` | 保持の時間 [s]。ramp・SIN 波では、軌道の終わりの後に保持する時間 |
| `--kp` | 位置ゲイン [N·m/rad] |
| `--kd` | 速度ゲイン [N·m·s/rad] |
| `--release` | 終わるときに力を抜く（Kp=Kd=0）。値は渡さない |
| `--lease`（`hil_trajectory.py` だけ） | 指令コマンドの keepalive が途切れてから controller が力を抜くまでの時間 [s]（既定 0.5） |

モーターが出すトルクは、Kp × （目標位置 − 位置）＋ Kd × （目標速度 − 速度）＋ 上乗せトルク [N·m]（MIT の指令）。

## B. 指令スクリプトで動かす（トピック経由）

A と同じ launch を起動したまま、別の端末で指令スクリプトを実行する。スクリプトは `/joint1_command/commands` に指令を送り、`joint1_command` controller がそれを 1 kHz で書き続ける（新しい値が来るまで同じ値）。

端末2（コンテナ）：

```bash
ros2 run bxi_hardware ramp_command.py 0.0 --kp 10 --kd 2              # 今の位置付近から 0 rad の方へ動かして保持
ros2 run bxi_hardware ramp_command.py 0.2 --kp 10 --kd 2              # +0.2 rad の方へ 0.3 rad/s で動かして保持
ros2 run bxi_hardware ramp_command.py 0.0 --kp 10 --kd 2 --release    # 0 rad の方へ戻し、終わりに力を抜く
ros2 run bxi_hardware sine_command.py --amplitude 0.1 --frequency 0.5 --kp 20 --kd 1 --release   # SIN 波
```

- `ramp_command.py <目標位置>`：目標位置は絶対位置 [rad]。今の位置から目標まで、目標を `--speed`（既定 0.3 rad/s）で動かし、`--hold`（既定1秒）待って終わる。**`--release` なしで終わると、目標位置で保持したまま（力を出したまま）になる。** 最後は必ず `--release` 付きで終える。
- `sine_command.py`：今の位置を中心に、中心 ± `--amplitude`（片側の振幅。上の例では ±0.1 rad、往復の幅 0.2 rad）で往復する。振幅を `--ramp-cycles`（既定1周期）で立ち上げ、`--cycles`（既定5周期）往復して、立ち下げる。
- どちらのスクリプトも、Kp × 位置の誤差（SIN 波は上乗せトルクを足す）が 5 N·m を超える、`/joint_states` が 0.2 秒途切れる、Ctrl+C のときに Kp=Kd=0 を送って終わる。
- 静止摩擦（約 1 N·m）のため、目標の手前で止まる（Kp 10 で +0.2 rad を目標にすると、約 +0.155 rad）。

止めるときは、スクリプトを Ctrl+C（力を抜いて終わる）→ 端末1の launch を Ctrl+C → `joint1 disable (deactivate): acknowledged` を確かめる。

## C. RT 軌道 controller で動かす

指令を controller の中で 1 kHz で作る構成。軌道は hold・ramp・sine の3種類で、指令コマンド `hil_trajectory.py` は開始の要求と keepalive を送るだけ。各操作は、終わると力を抜く。

端末1（コンテナ）：記録の書き出し先を作ってから起動する。**`BXI_WORKER_TIMING_DIR` を指定しないと、指令コマンドは記録の保存を確かめられずに NG で終わる。**

```bash
OUT=/ws/log/rt_manual_$(date +%Y%m%d_%H%M%S) && mkdir -p $OUT
BXI_WORKER_TIMING_DIR=$OUT ros2 launch bxi_hardware single_joint_hil.launch.py \
  config_file:=$H/bxi_hardware_stream_1khz.yaml controllers_file:=$H/controllers_1khz_rt.yaml \
  controller_name:=hil_trajectory
```

端末2（コンテナ）：

```bash
ros2 run bxi_hardware hil_trajectory.py --kp 10 --kd 2 --hold 1 hold                          # 今の位置で1秒保持し、力を抜く
ros2 run bxi_hardware hil_trajectory.py --kp 10 --kd 2 --speed 0.3 --release ramp 0.2         # +0.2 rad の方へ動かし、力を抜く
ros2 run bxi_hardware hil_trajectory.py --kp 20 --kd 1 --amplitude 0.1 --frequency 0.5 --release sine   # SIN 波
```

- `ramp` の目標は絶対位置 [rad]、`sine` の `--amplitude` は中心（開始の周期の位置）からの片側の振幅 [rad]。引数の単位は上の表のとおり。
- 終わると `終了時: 終了で力を抜いた`（`--release` なら `release で力を抜いた`）と、記録の CSV のパスが出る。SIN 波では追従の集計も出る。
- 上限は controller が持つ（Kp 20、Kp × 誤差 + 上乗せ 5 N·m、目標速度 2 rad/s、位置 ±0.5 rad、1回 60 秒）。範囲外の要求は受け付けない。
- 異常や Ctrl+C では `[中止]` と終了コード1で終わり、controller は力を抜く。**中止で終わった後は、同じ launch では次の操作を始められない**（launch を止めて起動し直す）。
- 操作の間は力が抜けている。次の操作は、その時点の位置から始まる。

止めるときは、端末1の launch を Ctrl+C → `joint1 disable (deactivate): acknowledged` を確かめる。

## D. 正式な試験（ランナー）

記録と自動の判定を残す試験は、**ホストの端末1つ**で次のスクリプトを実行し、表示される質問に答える。事前確認、CAN と割り込みの記録、launch と bag の起動、現在位置の確認、操作、停止と disable の確認、CAN・bag の確認、電源の扱いまでをスクリプトが行う。

| スクリプト | 内容 | 手順書（ワークスペース側） |
|---|---|---|
| `tools/hil_stage3.sh` | 段階3：`ramp_command.py` で保持 → +0.2 rad → 0 rad（Kp 10・Kd 2） | [`docs/bxi/worker-hil/procedures/2026-09-29-hil-stage3-procedure.md`](../../../docs/bxi/worker-hil/procedures/2026-09-29-hil-stage3-procedure.md) |
| `tools/hil_stage4.sh` | 段階4：保持と `sine_command.py` の SIN 波（Kp 20・Kd 1、0.1 rad・0.5 Hz） | [`docs/bxi/worker-hil/procedures/2026-09-29-hil-stage4-procedure.md`](../../../docs/bxi/worker-hil/procedures/2026-09-29-hil-stage4-procedure.md) |
| `tools/hil_stage3r.sh` | 段階3R：段階3と同じ動きを RT 軌道 controller で | [`docs/bxi/worker-hil/procedures/2026-09-30-hil-stage3r-4r-procedure.md`](../../../docs/bxi/worker-hil/procedures/2026-09-30-hil-stage3r-4r-procedure.md) |
| `tools/hil_stage4r.sh` | 段階4R：段階4と同じ動きを RT 軌道 controller で | 同上 |

```bash
cd ~/devel/minibexkai_ros2_ws
tools/hil_stage3r.sh            # 例
```

- 開発コンテナが動いていて、A〜C の launch が残っていないこと。
- 記録は `log/hil_<段階>_<日時>/` に残り、最後に結果のまとめ（緑の「すべて OK」か赤の「NG がある」）が出る。
- 現在位置が開始範囲（−0.05〜+0.05 rad）の外なら、指令を出さずに止まる。そのときは、手順書の「開始位置が範囲外のとき」に従って手で戻してからやり直す。

## 止め方（まとめ）

| 状況 | すること |
|---|---|
| 正常に止める | 指令スクリプトを Ctrl+C（または終わるのを待つ）→ launch を Ctrl+C → ログの `joint1 disable (deactivate): acknowledged` を確かめる |
| 想定外の動き・止まらない動き、スクリプトやホストの異常、端末が閉じた | **先に電源装置の出力を切る。** その後で launch を止める |
| disable の応答が出ない、launch が止まらない | 電源装置の出力を切る |
| 異常な温度、CAN のエラー、`Overrun detected` の多発（動きは制御下） | Ctrl+C で止める。disable の応答を確かめられなければ出力を切る |
| bus-off（`ip -details link show can0` が `BUS-OFF`） | 出力を切り、[`README.md`](README.md) の「bus-off からの復帰」に従う |

「力を抜いた」という表示は、Kp=Kd=0 の指令を送ったことを示すだけで、モーターに届いたことの確認ではない。止めた後は、disable の応答で確かめる。

## 指令の上限（0以外の指令）

| 項目 | 上限 |
|---|---|
| 位置 | ±0.5 rad の内側 |
| Kp / Kd | 20 / 2 |
| Kp × 位置の誤差 + 上乗せトルク | 5 N·m（Kd の項を含まない。モーターが出す総トルクの上限ではない） |
| 目標速度 | 2 rad/s |
| 上乗せトルク（摩擦の打ち消し） | 1 N·m |

**このガイドの送受信分離・1 kHz 構成で実機確認したのは**、Kp 10・Kd 2・0.3 rad/s の ramp（0〜+0.2 rad）と、Kp 20・Kd 1 の SIN 波（振幅 0.1 rad・0.5 Hz、摩擦の打ち消しなし）まで（2026-09-29 の段階1〜4、2026-09-30 の段階3R・4R）。2026-09-25 の Classic・200 Hz・逐次の構成（交換前のモーター）では、より大きい振幅（0.3 rad）・Kp 30・摩擦の打ち消し（一定 0.6〜0.9 N·m）も使った（[`README.md`](README.md) の結果）。それより大きい値は、小さい値から段階的に上げ、先に [`README.md`](README.md) の「0以外の指令の範囲」に根拠と手順を書く。

## うまくいかないとき

| 症状 | 見るところ・対処 |
|---|---|
| launch がすぐ終わる、`起動を中止` | `can0` が UP で `ERROR-ACTIVE` か、モーターの電源が入っているか、ほかに `can0` を使うプロセスがないか |
| `stopping all motors` が出て止まった | 送受信分離の安全停止（応答の途絶、`command stale` など）。ログの `fault=` を見る |
| `hil_trajectory.py` が「CSV の保存」で NG | launch に `BXI_WORKER_TIMING_DIR` を渡していない（C） |
| `hil_trajectory.py` が「前の実行の記録を書き出した正常終了でもない」で始まらない | 前の実行が中止で終わった。launch を起動し直す |
| ランナーが「現在位置の確認が NG」で止まる | 開始範囲の外。手順書の「開始位置が範囲外のとき」 |
| `ros2 topic echo` が何も出さない | launch と同じコンテナの中で、`source` を済ませて実行しているか |

## 関連する文書

- [`README.md`](README.md)：設定・RT スケジューリング・計測・bus-off からの復帰・結果の詳細
- [`../docs/CONTROLLERS.md`](../docs/CONTROLLERS.md)：controller の一覧と違い
- [`../docs/MOTOR_CONFIG_AND_STATUS.md`](../docs/MOTOR_CONFIG_AND_STATUS.md)：モーターの設定（yaml とモーター本体）とステータスの確認方法
- [`../docs/PROTOCOL_AND_SAFETY.md`](../docs/PROTOCOL_AND_SAFETY.md)：MIT プロトコルと安全上の判断
- ワークスペース側の `docs/bxi/worker-hil/reports/`：各段階の試験レポート

## 起動ログの見方

起動ログは次の3か所で見られる。

### 1. launch を起動した端末（A〜C の端末1）

`single_joint_hil.launch.py` は `ros2_control_node` の出力を画面に出すので、HWIF と controller_manager のログがそのまま端末1に流れる。

起動の直後に出ていることを確かめる行：

| 行 | 意味 |
|---|---|
| `Enable joint1 pos=... temp=...` | モーターの enable に成功。その時点の位置 [rad] と温度 [℃] |
| `送受信分離を開始 (1 motors, broadcast, 1000Hz, ...)` | 1 kHz の送受信が始まった |
| `Configured and activated joint_state_broadcaster`、`Configured and activated joint1_command`（C では `hil_trajectory`） | 2つの controller が有効になった |
| `Successful set up FIFO RT scheduling policy with priority 50.` | controller_manager の更新スレッドが RT の優先度で動いている |

出ていたら異常な行：

| 行 | 意味 |
|---|---|
| `起動を中止` | 起動の途中で止めた。同じ行に理由が出る（enable の失敗、両方の温度センサーの故障、送受信分離の最初の応答が揃わない、など） |
| `stopping all motors` | 送受信分離の安全停止。同じ行か後の `送受信分離を停止` の行の `fault=` に理由が出る |
| `Unable to lock the memory` | メモリを固定できなかった（コンテナの設定。[`README.md`](README.md) の「controller_manager の RT 更新スレッド」） |
| `Overrun detected`（何度も出る） | controller_manager の周期が 1 ms を超えた |

止めたときに確かめる行：

| 行 | 意味 |
|---|---|
| `joint1 disable (deactivate): acknowledged` | disable の応答があった（力が抜けたことの確認） |
| `送受信分離を停止: ... fault=none ...` | 異常なく止まった。`skipped`・`no_new` が 0、`max_age` が 3 ms 未満であること |

### 2. 端末を閉じた後、または流れて見失ったとき（コンテナの中のファイル）

ROS のログは、コンテナの `~/.ros/log/` にも残る。

- `ros2_control_node` のログ（上の表の行はこちらに残る）：`~/.ros/log/ros2_control_node_<PID>_<時刻>.log`

  ```bash
  F=$(ls -t ~/.ros/log/ros2_control_node_*.log | head -1)   # いちばん新しいもの
  grep -E "Enable joint1|送受信分離|disable \(|stopping all motors|起動を中止|Overrun" "$F"
  ```

- launch 全体のログ（プロセスの起動と終了、spawner の行）：`~/.ros/log/<日時>-<ホスト名>-<番号>/launch.log`

### 3. 正式な試験（D、ランナー）

ランナーは launch の出力をすべて記録先の `log/hil_<段階>_<日時>/launch.log` に保存し、判定もその中身で自動に行う。結果のまとめ（`summary.txt`）に「OK launch の起動」「OK disable の応答」などとして出る。ホストから直接見るとき：

```bash
grep -E "Enable joint1|送受信分離|disable \(" ~/devel/minibexkai_ros2_ws/log/hil_current/launch.log
```
