# bxi_hardware で使う controller

`bxi_hardware` が提供する controller と、設定で使っている外部の controller、その組み合わせ方をまとめる（2026-09-30 時点）。`BxiSystemInterface` は controller ではなく、ハードウェアのプラグイン（`hardware_interface::SystemInterface`）である。

## 1. `bxi_hardware` が提供する controller

| controller（プラグイン名） | ライブラリ | 用途 | 確認の状況 |
|---|---|---|---|
| `bxi_hardware/HilTrajectoryController` | `bxi_hil_controllers` | 1軸の HIL 用。保持（hold）・ramp・SIN 波の目標を、`update()` の中で毎周期（1 kHz）計算して HWIF に書く | 単体試験、vcan、実機の段階3R・4R（2026-09-30 に合格） |
| `bxi_hardware/HilUpdateStallController` | `bxi_hil_test_controllers`（本番とは別のライブラリ） | **試験専用**。`~/stall` の後の `update()` で `stall_ms` だけ止まり、controller_manager の更新の停止を再現する | 単体試験、vcan。環境変数 `BXI_ALLOW_TEST_FAULTS=1` がないと設定でエラーになる。実機では使わない |

### `HilTrajectoryController` の要点

- インターフェース：command は `<joint>/position`・`velocity`・`kp`・`kd`・`effort`、state は `<joint>/position`・`velocity`・`effort`。扱える関節は1つ。
- 受け渡し：サービス `~/start`（軌道のパラメータ）・`~/finish`・`~/stop`、トピック `~/keepalive`（リース）、`~/status`。指令コマンドは `hil/single_joint/hil_trajectory.py`。
- 上限：Kp 20（最大 30）、Kp × \|目標 − 位置\| + \|上乗せ\| 5 N·m、目標速度 2 rad/s、位置 ±0.5 rad、周期 5 ms、1回の実行 60 秒。中止・停止ではその周期のうちに Kp=Kd=0 を書く。
- 記録：毎周期の記録（最大120秒分）を CSV に書き出す。
- 試験専用のパラメータ `test_fault.start_delay`・`stop_delay`・`finish_delay`（サービスの遅延）は、`BXI_ALLOW_TEST_FAULTS=1` のときだけ使える。
- 詳細は [`hil/README.md`](../hil/README.md) の「RT 軌道 controller」。計画と結果はワークスペース側の `docs/bxi/worker-hil/plans/2026-09-29-rt-trajectory-controller-plan.md` と `docs/bxi/worker-hil/reports/2026-09-30-hil-stage3r-rt-ramp.md`・`2026-09-30-hil-stage4r-rt-sine.md`。

## 2. 設定で使っている外部の controller（ros2_controllers）

| controller | 設定での名前 | 用途 | 確認の状況 |
|---|---|---|---|
| `joint_state_broadcaster/JointStateBroadcaster` | `joint_state_broadcaster` | 位置・速度・effort を `/joint_states`（`sensor_msgs/msg/JointState`）に出す。温度は `/joint_states` には含まれない（下記） | すべての構成で使う |
| `forward_command_controller/MultiInterfaceForwardCommandController` | `joint1_command` | `/joint1_command/commands`（`data = [position, velocity, kp, kd, effort]`）の最後の値を、毎周期そのまま HWIF に書く。`ramp_command.py`・`sine_command.py` と組み合わせる | 実機（2026-09-25 の 200 Hz、2026-09-29 の段階1〜4 の 1 kHz） |

- 温度（HWIF の state インターフェース `temperature`）は、`/joint_states` ではなく `/dynamic_joint_states`（`control_msgs/msg/DynamicJointState`）の追加のインターフェースとして出る。`joint_state_broadcaster` のパラメータ `publish_dynamic_joint_states` の既定は `true` で、このパッケージの設定は `interfaces` を指定していないので、使えるすべての state インターフェースが対象になる（ワークスペースの ros2_controllers の実装と設定から判断。HIL の bag は `/joint_states` だけを記録していて、`/dynamic_joint_states` は確かめていない）。

`forward_command_controller` は、apt 版では ABI が合わなかったので、ワークスペースの ros2_control の fork に対してビルドする（[`hil/README.md`](../hil/README.md)）。

## 3. 構成（controller の設定ファイルと launch）

launch は `hil/single_joint/single_joint_hil.launch.py` の1つで、`controllers_file` と `controller_name` の引数で切り替える。

| 設定ファイル | 周期 | 組み合わせる HWIF の設定 | command インターフェースに書く controller | 使った段階 |
|---|---|---|---|---|
| `controllers.yaml`（launch の既定） | 200 Hz | `bxi_hardware.yaml`（Classic・逐次） | `joint1_command` | 2026-09-25 の HIL |
| `controllers_1khz.yaml` | 1 kHz | `bxi_hardware_stream_1khz.yaml`（FD・送受信分離） | `joint1_command` | 段階1〜4（2026-09-29） |
| `controllers_1khz_rt.yaml` | 1 kHz、更新スレッドを CPU 11 に固定 | 同上 | `hil_trajectory` | 段階3R・4R（2026-09-30） |

どの構成でも、launch は `ros2_control_node` に `LD_BIND_NOW=1` を渡す。

## 4. command インターフェースに書く controller

ros2_control の controller は、HWIF のインターフェースとの関わり方で2つに分けられる。

| 種類 | すること | この構成での例 |
|---|---|---|
| state を読むだけ | state インターフェースを読んで外に出す（位置・速度・effort は `/joint_states`、温度は `/dynamic_joint_states`）。モーターへの指令には関わらない | `joint_state_broadcaster` |
| command インターフェースに書く | command インターフェース（`position`・`velocity`・`kp`・`kd`・`effort`）を確保（claim）し、毎周期値を書く。HWIF はその値を MIT の指令として CAN で送る | `joint1_command`、`hil_trajectory` |

- 1つの command インターフェースを確保できる controller は、同時に1つだけである。`joint1_command` と `hil_trajectory` は `joint1` の同じ5つの command インターフェースを使うので、同時には有効にできない。launch の `controller_name`（既定 `joint1_command`、段階3R・4Rでは `hil_trajectory`）でどちらか一方を選ぶ。

2つの controller は、HWIF に同じ形の指令を書くが、目標を作る場所が違う。

| | `joint1_command`（forward_command_controller） | `hil_trajectory`（HilTrajectoryController） |
|---|---|---|
| 目標を作る場所 | controller の外。別プロセスのスクリプト（`ramp_command.py` 100 Hz、`sine_command.py` 200 Hz）がトピックで送る | controller の中。`update()` で、毎周期（1 kHz）その時刻の軌道の値を計算する |
| controller がすること | 最後に受け取った値を、次が届くまで毎周期そのまま書く（0次ホールド） | 軌道の計算、上限の検査、中止・脱力、記録 |
| 安全の判定 | 主にスクリプト側（Kp × 誤差の上限、状態の途切れなど） | controller の中で毎周期 |
| CAN に載った目標 | 階段状（段階3の移動中、位置が前のフレームから変わったのは 10 %） | ほぼ毎周期変わる（段階3Rで 79 %。100 % でないのは MIT の位置の分解能 約 0.38 mrad のため） |

指令が決まる流れ：

- 段階3・4：スクリプト → `/joint1_command/commands`（トピック） → `joint1_command` → HWIF → CAN
- 段階3R・4R：`hil_trajectory.py` →（開始の要求と keepalive だけ）→ `hil_trajectory`（軌道を計算） → HWIF → CAN

## 5. `hil_trajectory` は補間ではない

`hil_trajectory` は、外から指令値の列を受け取って1 kHz に補間しているのではない。外から受け取るのは軌道のパラメータだけで、指令値はその場で式から計算する。

- 開始のときに1回だけ、`hil_trajectory.py` が `~/start` で軌道のパラメータを渡す。

  | 種類 | パラメータ（例） |
  |---|---|
  | hold | Kp、Kd、保持の時間 |
  | ramp | 目標位置 0.2 rad、速さ 0.3 rad/s、Kp、Kd、保持の時間 |
  | sine | 振幅 0.1 rad、周波数 0.5 Hz、周期数、立ち上げの周期数、摩擦の打ち消し、Kp、Kd |

- 開始の後に外から届くのは、keepalive（20 Hz、指令コマンドが生きているという合図）と `~/finish`・`~/stop` だけで、目標位置の値は届かない。
- controller は、開始の周期の現在位置を ramp の開始位置・SIN 波の中心にし、毎周期、`period` を積算した経過時間 t で、その時刻の目標位置と目標速度を計算する。
  - ramp：開始位置から目標へ t × 速さだけ進んだ位置（速度は一定）
  - sine：中心 + 包絡線(t) × sin(2π f t)（立ち上げ・立ち下げは cos 形状）。速度はその微分
- 式は `ramp_command.py`・`sine_command.py` の式を C++ に移したもので、Python 版と差 1e-12 以下で一致することを単体試験で確かめている。

補間との違い：

- 補間は、外から粗い点列（例：100 Hz の目標位置）を受け取り、その間を埋める。受け取る点の遅れや抜けがそのまま影響する。
- `hil_trajectory` は軌道そのもの（式とパラメータ）を持ち、その時刻の値を直接計算するので、DDS の遅れや別プロセスの周期のずれが入らない。
- 段階3・4の `joint1_command` も補間していない。届いた最後の値を次が届くまで書き続ける（0次ホールド）。

限界：外から任意の指令列（例：学習したポリシーの出力）を受け取って 1 kHz で追う用途には、そのままでは使えない。`hil_trajectory` は hold・ramp・sine の3種類だけを扱う HIL 用の controller である。

## 6. HWIF と controller の取り決め（`BxiSystemInterface`）

- 出しているインターフェース：
  - command：`position`・`velocity`・`kp`・`kd`・`effort`
  - state：`position`・`velocity`・`effort`・`temperature`
- `position` の command が NaN（指令なし・controller の無効化の後）のとき、HWIF は Kp=Kd=0 にし、位置はその時点の現在位置を送る。
- `kp`・`kd` の command だけが NaN のときは、設定の `default_kp`・`default_kd` を使う（1 kHz の設定 `bxi_hardware_stream_1khz.yaml` では0、`bxi_hardware.yaml` では 10 と 2）。
- controller_manager の更新が止まったときは、送信スレッドが最後の `write()` から `stream_command_max_age_ms`（5 ms）で検出し、Kp=Kd=0 を送って安全停止する（送受信分離の設定のみ）。

## 7. `bxi_hardware` にないもの・使っていないもの

- 複数の関節を扱う軌道の controller（例：`joint_trajectory_controller`）の設定はない。`HilTrajectoryController` は1軸の HIL 用である。
- `rt_safe_controller/RtSafeController` はワークスペースの `mevius_ros2_ws` 側の別パッケージ（異常を検知して減衰で止める chainable controller）。`position`・`velocity`・`kp`・`kd`・`effort` を確保するので、インターフェースの形は BXI の HWIF と合うが、`bxi_hardware` の設定では使っておらず、BXI のモーターとの組み合わせは確かめていない。
