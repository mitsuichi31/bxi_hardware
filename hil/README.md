# 1軸の実機試験（ベンチ）

モーター1台（BXI7010-19、`can_id` 1）を PCAN の `can0` に直接つないで試験する。bxi_usb2can は使わない。

実機への通電と、モーターを動かす操作は、試験者が安全を確認してから行う。

**手順だけを知りたいときは [`SINGLE_JOINT_GUIDE.md`](SINGLE_JOINT_GUIDE.md)（1軸の動作確認ガイド）を読む。** この README には、設定の詳しい説明、測定の結果、経緯も含めている。

## 0. 準備（ホスト）

```bash
# can0 を CAN FD 1 Mbps / 5 Mbps（サンプル点 0.8 / 0.8125、berr-reporting on）で up する。
# FD で起動しても、通常の CAN（can_frame_format: classic）はそのまま使える。
cd ~/devel/minibexkai_ros2_ws
./docker/can_setup.sh can0
ip -details -statistics link show can0      # state ERROR-ACTIVE を確認
```

`docker/can_setup.sh` はワークスペース（`minibexkai_ros2_ws`）側にある。引数を省略すると can0〜can3 をすべて up する。

- モーターの電源出力が ON で、MENU の状態（`STATE : 01 MENU`）であることを確認する。前の試験から出力を維持した場合も再確認する。
- モーターの `can_timeout` は 0（通信が止まっても最後の指令を出し続ける）。**試験中にプロセスを強制終了すると、最後の指令が残る。** 停止の順序は下記「段階6 実機試験前の確認」の「止めるとき」に従う。
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

### controller_manager の RT 更新スレッド

`hil/single_joint/controllers.yaml` は更新周期 200 Hz、`SCHED_FIFO` 優先度 50、メモリ固定を指定する。
`docker/compose.yaml` の `ulimits.rtprio: 99` と `ulimits.memlock: -1` が、非 root の `ros` ユーザーに RT 優先度とメモリ固定を許可する。`cap_add: SYS_NICE` だけでは、`ros` ユーザーの実効 capability は増えない。

Compose の変更を適用するには、コンテナの**再作成**が必要。VS Code Dev Container を使っている場合は VS Code の「Dev Containers: Rebuild Container」で再作成する（作業中のターミナル接続は切れる）。通常の Compose 起動なら、ホストで `cd docker && docker compose up -d --force-recreate minibex_dev` を実行する。再作成後、コンテナ内で次を確認する。

```bash
uname -r                 # ホストと同じ realtime カーネル
cat /sys/kernel/realtime # 1
ulimit -r                # 99
ulimit -l                # unlimited
```

`ros2 launch bxi_hardware single_joint_hil.launch.py` の起動ログに `Successful set up FIFO RT scheduling policy with priority 50.` があり、`Unable to lock the memory` がないことを確認する。ログに出る `Spawning ... RT thread` は起動の宣言であり、成功の証拠ではない。別端末でプロセスと各スレッドを調べる。

```bash
pgrep -af '/controller_manager/ros2_control_node' # PID を確認
cm_pid=12345                                      # 上で確認した PID に置き換える
ps -T -p "$cm_pid" -o pid,tid,cls,rtprio,comm     # FF / 50 の TID を確認
chrt -p 12346                                     # FF / 50 の TID に置き換える
sed -n '/^VmLck:/p' "/proc/$cm_pid/status"        # メモリ固定量を確認
```

2026-09-27 の設定変更前、稼働中の Dev Container では `ulimit -r` が 0、`ulimit -l` が 8192 KiB で、モーターを使わない `controller_manager` 単体起動はメモリ固定と FIFO 設定の両方に失敗した。再作成後の Dev Container では `ulimit -r` が 99、`ulimit -l` が unlimited。HIL の `controllers.yaml` を使った単体起動で更新周期 200 Hz が読み込まれ、更新スレッドが OS 上で `FF / 50`、`chrt -p` で `SCHED_FIFO / 50`、`VmLck` は約 1.9 GiB だった。実機や CAN を使わない `minibexkai_bringup mock.launch.py` も 200 Hz で起動し、mock hardware と controller 2件が有効化された。約 12 秒の観測では overrun 警告は出ていない。単体起動には robot description を渡しておらず、mock は BXI の CAN worker を含まないため、実機 HIL でも確認した。

同日、`can0` が `ERROR-ACTIVE` で競合する制御プロセスがないことを確認し、`single_joint_hil.launch.py` を指令トピックへの送信なしで約 14 秒起動した。`BxiSystemInterface` がロードされ、`joint1` の enable、CAN worker の 200 Hz 起動、2つの controller の有効化に成功。更新スレッドは `ps` で `FF / 50`、`chrt -p` で `SCHED_FIFO / 50`、`VmLck` は 2,567,316 kB だった。メモリ固定失敗と overrun の警告は出なかった。SIGINT で終了し、worker 停止と `on_deactivate` を経て `ros2_control_node` は正常終了した。指令は送っておらず、実機制御中の周期遅れの分布と CAN worker の遅れはまだ測定していない。

`BxiSystemInterface` の CAN worker はこの更新スレッドとは別で、今回の RT 設定の対象外。CAN 送信周期や指令から実機応答までの遅延は、別途測定する。

### CAN worker の周期を計測する

CAN worker は現在 `SCHED_OTHER`。計測時だけ `BXI_WORKER_TIMING_DIR` に書き込み可能な既存ディレクトリを指定する。周期中は事前確保したバッファへ記録し、軸の disable と CAN バスの close が終わった後に、バスごとの `*_cycles.csv` と `*_transactions.csv` を保存する。通常の起動では計測を行わない。1バスあたり20,000周期分を確保し、超過したサンプル数は CSV 先頭の `dropped_*` に残す。強制終了時は CSV を保存できないため、通常の SIGINT で停止する。

計測を有効にしたときだけSocketCANの `SO_TIMESTAMPNS` を有効にし、`*_transactions.csv` に制御呼び出し直前・カーネル受信・ユーザー空間取込の `CLOCK_REALTIME` 時刻（ns）を追加する。受信時刻が取得できない取引は0になる。既存の `duration_ns` は単調時計の値で、絶対時刻列と直接引き算しない。`tx_call_realtime_ns` は `sendControl()` 呼び出し直前であり、CANフレームの実際の送出時刻ではない。カーネル受信時刻もオンワイヤ時刻ではない。計測しない起動は従来の `read()` 経路を使う。

```bash
mkdir -p /ws/log
BXI_WORKER_TIMING_DIR=/ws/log ros2 launch bxi_hardware single_joint_hil.launch.py
# Ctrl+C で停止後、ログに表示されたファイル名の末尾 _cycles.csv / _transactions.csv を確認する。
python3 /ws/src/bxi_hardware/hil/single_joint/analyze_worker_timing.py \
  /ws/log/bxi_worker_can0_1790496612659  # 番号は今回の起動ログに合わせる
```

`can_transport` で `rx_mode: separate` にしたバスは、上記の worker CSV ではなく、停止時にバスごとの `bxi_stream_<bus>_<時刻>_cycles.csv` と `_rx.csv` を保存する。同じく1バスあたり20,000周期分を事前確保し、周期中はファイルに書かない。

- `_cycles.csv`：周期ごとの結果（`sent`、`skipped_late`、`skipped_burst`、`expired_after_hook`、`partial`、`stopped`）、予定時刻からの起床の遅れ、hook（`data_mutex_` の待ちを含む）の所要時間、予定時刻からの最初のフレームの送信の遅れ、フレーム数、軸ごとの「周期開始時の最新応答経過時間」（列名 `age_ns_<id>`、最初の送信前は -1）と新しい応答の有無（`fresh_<id>`）、最後の列に周期開始時の指令の古さ（最後の `write()` からの経過時間、列名 `command_age_ns`、最初の `write()` の前は -1。2026-09-29 の手順0で追加）。先頭行の `max_command_age_ns` がその停止しきい値（`stream_command_max_age_ms`）。停止ログの `送受信分離を停止` の行にも、その最大（`max_cmd_age`）が出る。

  - **周期開始時の最新応答経過時間**：送信スレッドが周期ごとに起きた時点で、最新の応答を受信してから何 ns 経ったか。送信するかどうかを決める前に測るので、スキップした周期や停止した周期にも記録される。`stream_feedback_max_age_ms` を超えると安全停止する。
  - **受信から次の送信までの時間**：送信した周期だけについて、最新の応答から実際の送信までの時間。`age_ns + (send_lateness_ns − wake_lateness_ns)` で求める（解析スクリプトが計算する）。
  - **新しい応答が届いたか**は `fresh_<id>` で判定する。経過時間が1周期未満でも、周期の起床間隔が揺れると新着の有無とは一致しない。送信スレッド自身が遅れた場合は、新しい応答が届いていても経過時間が大きくなる。
- `_rx.csv`：有効な応答ごとの受信時刻（単調時計、開始からの ns）、カーネル受信・ユーザー空間取込の `CLOCK_REALTIME` 時刻、rawハードウェア受信時刻（取得できないインターフェースでは0）。カーネル受信から取込までの差には、受信スレッドの起床に加えて `recvmsg()` と先行フレームの処理待ちが含まれる。受信スレッドの起床遅れの上限値として扱い、起床遅れそのものとは呼ばない。
- 先頭行の `tx_thread` / `rx_thread` は、各スレッドが開始した時点のスケジューリング方式・優先度・CPUアフィニティ。スレッド名は `bxtx_<bus>` / `bxrx_<bus>` で、`ps -L` や `chrt -p` で確認できる。

```bash
python3 /ws/src/bxi_hardware/hil/single_joint/analyze_stream_timing.py \
  /ws/log/bxi_stream_can0_1790632101395 --tolerance-ms 0.1  # 番号は今回の起動ログに合わせる
```

`--tolerance-ms` を超えた送信の遅れ・間隔だけを数える。1 ms をわずかに超えた正のジッタを合否判定に使わないためである。

### 送受信分離のスケジューリングを vcan で比較する

実機を使わずに、controller_manager（1 kHz・`SCHED_FIFO` 50、`controllers_1khz.yaml`）と送受信分離（`bxi_hardware_stream_1khz.yaml` を vcan 用に書き換えたもの）を同時に動かし、送受信スレッドのスケジューリング条件を比べられる。

- `bxi_vcan_motor_emulator`：vcan 専用の模擬モーター。個別ID宛ての指令にはすぐに応答し、`0x7FF` のブロードキャストには `--delay-us` 後に応答する。既定では応答の位置・速度・トルクは中央値のまま。`--follow 1` では、Kp > 0 の指令を受けると応答の位置を指令の位置へ一次遅れ（時定数 50 ms）で近づける（段階3R・4Rのランナーの bag の判定を vcan で通すための簡単な追従で、力学は模擬しない）。`--replies-per-command` で応答を増やすと、受信側の負荷を上げられる。`ros2 run` 経由では SIGINT がツールに届かないため、`$(ros2 pkg prefix bxi_hardware)/lib/bxi_hardware/bxi_vcan_motor_emulator` を直接起動する。
- `compare_stream_scheduling.py`：次の条件ごとに設定を作り、模擬モーターと `single_joint_hil.launch.py` を起動し、CSV・ログ・`summary.tsv` を出力先にまとめる。
  - 送受信スレッドの条件：SCHED_OTHER、controller_manager より下／上の FIFO、CPU 固定、受信スレッドだけ高い FIFO
  - CPU 負荷：なし／全 CPU にビジーループ
  - 1指令あたりの応答数
  - controller_manager については、ログの `Overrun detected` 警告の件数（`cm_overrun_warnings`）を数える。
  - `summary.tsv` は周期の結果を種類ごと（`sent`・`skipped_late`・`skipped_burst`・`expired_after_hook`・`partial`・`stopped`）に分けて数える。`stopped` は、故障で送信を止めた最後の周期である。
  - 受信記録は1軸あたり周期数の2倍（2万周期なら4万件）で満杯になる。`dropped_rx` が0でなければ、受信間隔と「カーネル受信→取込」の統計は、記録できた前半の応答だけの値である。
  - `--resummarize <出力先>` で、既存の CSV から `summary.tsv` を作り直せる。
  - 模擬モーターは既定で `SCHED_FIFO` 80 で動かす（`--emulator-priority`）。本物のモーターはホストの外にあり、ホストの CPU 負荷で応答が遅れないためである。SCHED_OTHER のままだと、CPU 負荷で模擬モーター自体が遅れ、送受信分離が鮮度切れで安全停止する。

```bash
sudo ip link add vcan0 type vcan && sudo ip link set vcan0 mtu 72 up
python3 /ws/src/bxi_hardware/hil/single_joint/compare_stream_scheduling.py --seconds 10 \
  --out /ws/log/stream_sched_$(date +%Y%m%d_%H%M%S)
sudo ip link del vcan0
```

vcan 上の値は、ホストのスケジューリングの比較であり、実機の CAN の送受信時間を含まない。

2026-09-27 の指令なし1軸 HIL ベースラインでは、6,756周期の平均開始間隔が5.1108 ms（約195.7 Hz）、worker 作業時間の中央値1.3127 ms・最大1.5377 ms、CAN 取引時間の中央値1.3082 ms・最大1.5330 msだった。5 msを超えた作業周期、無効応答、計測バッファ欠測は0。計測オン・オフの短い CAN 時刻比較では明確な悪化は観測されなかった。条件、p99などの分布、生データはワークスペース側の `docs/test-reports/2026-09-27-bxi-worker-baseline.md` に記録した。これは worker の優先度を変更する前の値で、指令送信時やタイムアウト時の遅れは含まない。

同じ条件で `can_frame_format.can0` だけを変えた CAN FD / CAN FD+BRS の実測は、ワークスペース側の `docs/test-reports/2026-09-27-bxi-worker-can-fd.md` に記録した。CAN取引時間の中央値は、Classic 1.3082 ms、FD（BRSなし）1.3272 ms、FD+BRS 1.2572 ms。3モードとも worker の作業時間は5 ms以内だが、周期開始間隔は約5.1 msのままだった。既定の `bxi_hardware.yaml` は Classic CAN のままにしている。

## 段階6 実機試験前の確認（送受信分離、1 kHz）

`bxi_hardware_stream_1khz.yaml` と `controllers_1khz.yaml` で実機を動かす前に、毎回この節を確認し、その結果を試験のレポートに残す。前提と段階は、ワークスペース側の `docs/plans/20260928_bxi_worker_stream_broadcast_plan.md` の「HILの前提と段階」を参照。

**決まっていること（2026-09-29）**

- **独立電源遮断：** 試験者が、電源装置（OWON SPE6103）の出力スイッチでモーターの電源を切る。ソフトウェアの disable には頼らない。
- **bus-off：** 自動で復帰させない。`docker/can_setup.sh` の既定は `restart-ms 0`。復帰は下記「bus-off からの復帰」の手順で行う。
- **指令：** ベンダーの回答を待たずに、下記「0以外の指令の範囲」の中で、安全を確認しながら Kp・Kd・トルクが0でない指令の動作確認を進める（2026-09-29 に決定）。
  - モーターは `can_timeout=0` で、通信が途絶えても最後の指令を保持する。ホストのプロセスが落ちると、最後の指令（目標位置と Kp・Kd・トルク）が残る。その場合は電源装置の出力を切って止める。
  - `bxi_hardware_stream_1khz.yaml` の既定ゲインは 0 のままにする。位置だけの指令では力が出ないので、ゲインは指令スクリプトで明示する。

**試験ごとに記録すること（直前に確認し、確認した時刻を書く）**

| 項目 | 確認の内容 |
|---|---|
| 日時・試験者 | 誰が電源遮断を担当するか |
| 電源遮断 | 出力スイッチにすぐ手が届く位置にいること。出力 ON を確認した時刻 |
| モーター | 固定されていること。周囲に人や物がないこと |
| CAN | `ip -details -statistics link show can0` で、`ERROR-ACTIVE`、TEC/REC 0/0、`restart-ms 0`。ほかに `can0` へ送信するプロセスがないこと |
| 割り込みスレッド | `ps -eLo pid,tid,cls,rtprio,psr,comm \| grep irq/16` で、PCAN の割り込みスレッドの方式・優先度・CPU（`can0` を UP にした後） |
| 版と設定 | ワークスペースと `src/bxi_hardware` のコミット、使う設定ファイル、送受信スレッドの優先度と CPU（2026-09-30 から `thread_cpus: [9]`）。RT 版（段階3R・4R）では controller_manager の CPU（`cpu_affinity: 11`）と `LD_BIND_NOW=1` |
| 計測 | `BXI_WORKER_TIMING_DIR` の出力先。試験前後の CAN 統計を記録すること |

**0以外の指令の範囲**

2026-09-25 の HIL（classic・200 Hz・逐次）で使った値と、指令スクリプトの中止条件を上限にする。送受信分離・1 kHz では、2026-09-29 に次の値だけを使った。HIL 段階3の `ramp_command.py`（Kp 10・Kd 2・0.3 rad/s、0〜+0.2 rad）と、段階4の `ramp_command.py`（Kp 20・Kd 1、保持のみ）・`sine_command.py`（Kp 20・Kd 1、振幅 0.1 rad・0.5 Hz、摩擦の打ち消しなし）である（ワークスペース側の `docs/test-reports/2026-09-29-bxi-hil-stage3-ramp.md`、`2026-09-29-bxi-hil-stage4-sine.md`）。それより大きい振幅・周波数や摩擦の打ち消しはまだ使っていないので、小さい値から段階的に上げる。

| 項目 | 上限 | 根拠 |
|---|---|---|
| 指令の手段 | `ramp_command.py` / `sine_command.py`。段階3R・4R（RT 軌道 controller、2026-09-30 に実機で実施）では `hil_trajectory.py` | 下記の中止条件が組み込まれている（RT 版では controller の中止条件）。トピックに直接指令を送らない |
| 位置 | `bxi_hardware.yaml` の `lower` / `upper`（±0.5 rad）の内側 | ドライバとスクリプトの両方でクランプ・確認する |
| Kp | 20（スクリプトの `--max-kp` の既定値）。30 までは `--max-kp` を明示して上げる | 2026-09-25 に Kp 30 まで実施 |
| Kd | 2 | 2026-09-25 に Kd 1〜2 で実施 |
| スクリプトが監視する式 | 5 N·m（`--max-torque` の既定値）。超えたらスクリプトが力を抜く。`ramp_command.py` は Kp × 位置の誤差の絶対値、`sine_command.py` はそれに上乗せトルクの絶対値を足した値を監視する。**どちらも Kd の項（速度の誤差）を含まないので、モーターが出す総トルクの上限ではない** | スクリプトの中止条件 |
| 上乗せトルク（摩擦の打ち消し） | 1 N·m | 2026-09-25 に 0.9〜1.0 N·m で実施 |
| 速度 | 2 rad/s（`sine_command.py` の `--max-velocity` の既定値） | スクリプトの中止条件 |
| 電源 | 電源装置の電流制限 5 A のまま | 2026-09-25 の構成 |

- 最初は `ramp_command.py 0.0 --kp 10 --kd 2`（今の位置付近での保持）から始め、`ramp_command.py 0.2`、`sine_command.py --amplitude 0.1 --frequency 0.5` の順に進める。
- 終わるときは `--release` で力を抜く。
- `/joint_states` が 0.2 秒以上途切れると、スクリプトが力を抜いて止まる。
- 上限を超える値を使うときは、先に根拠と手順をこの節に追記する。

### 2026-10-02追記：±90°・0.5 Hz専用の指令範囲（準備レビュー待ち）

本人が目標速度上限5.0 rad/s・実速度事後判定6.0 rad/s（自動停止なし）を事前指定したため、この条件だけ目標速度上限を5.0 rad/sへ見直す。固定中心0 rad・振幅π/2・0.5 Hz、前後ramp各2周期（各4秒）・定常1周期2秒・保持1秒後release。専用 `controllers_1khz_rt_90deg_05hz.yaml` の**目標速度上限は5.0 rad/s**。定常最大4.934802 rad/s、rampを含む解析的上界4.973206 rad/sで上限以内（余裕約0.026794 rad/s）。速度上界は A·hypot(2πf, π/(2Tr))、A=π/2、Tr=2/f。定常最大加速度は15.503138 rad/s²で0.3 Hzの約2.78倍。承認済み0.3 Hz実機から新条件の安全性・追従・再現性を保証しない。本人指定と計算を根拠に専用設定を準備し、準備レビューを受ける。

位置±1.65 rad、Kp20・Kd1、上乗せ／摩擦補償0、トルク保護式5 N·m（Kdを含む総トルク上限ではない）、電源24 V・電流制限5 A、周期・心拍・lease・disable等を維持する。旧0.05／0.1／0.15／0.2／0.3 Hzの設定・判定は変更しない。実速度は操作B全区間のbagで**事後に6.0 rad/s以内**を判定する。実速度6.0超・実位置±1.65超それ自体による自動停止はない。異常時は本人が先に電源出力を切る。NGは収録・停止の成否と分けて報告し、事後の基準緩和・同条件再試行を行わず、原因・対応のレビュー後に新しいpreflightで再開する。

境界・controller全軌道と拒否試験、正常／操作B途中Ctrl+Cのvcan、復元可能rawを用意し、準備レビュー承認後に本人が実機を行う。実機前は0 radの物理姿勢、±1.65 rad全域の接触・配線巻き込みなし、重力負荷なし、電源出力スイッチへの即時アクセスを本人が実物で確認する。今回動画なし。実機結果は別途レビューする。ワークスペースの `docs/plans/20261002_bxi_90deg_05hz_plan.md` を参照。

0.5 Hzの最初の正常vcanは、既定50 msの模擬追従遅れにより約4.018秒でトルク保護中止となった。失敗原本を保存し、正常合格とは扱わない。vcan専用の模擬モーターは既定50 msを維持し、今回の正常経路／途中Ctrl+C検証だけ `--follow-time-constant-ms 10` を明示した。10 msは検証用の仮定で、実機の追従性能・安全性を示さない。実機controllerの保護値・軌道・旧設定を変更して通したものではない。時定数を条件JSON・emulator.logへ記録し、模擬ソース・実行ファイルSHA256も保存する。

### 2026-10-02追記：±90°・0.3 Hz専用の指令範囲（準備レビュー待ち）

本人が目標速度上限3.0 rad/s・実速度事後判定3.5 rad/sを事前指定したため、この条件だけ従来の目標速度上限2.0 rad/sを見直す。固定中心0 rad・振幅π/2・0.3 Hz、前後ramp各2周期（各6.666667秒）・定常1周期3.333333秒・保持1秒後release。専用 `controllers_1khz_rt_90deg_03hz.yaml` で**目標速度上限3.0 rad/s**を使う。定常最大2.960881 rad/s、rampを含む解析的上界2.983924 rad/sで上限以内（余裕約0.016076 rad/s）。速度上界は A·hypot(2πf, π/(2Tr))、A=π/2、Tr=2/f。0.2 Hz実機は承認済みだが、この上限見直しは本人の新条件指定に基づくもので、旧実機の安全性・追従・再現性を外挿しない。

位置±1.65 rad、Kp20・Kd1、上乗せ／摩擦補償0、トルク保護式5 N·m（Kdを含む総トルク上限ではない）、電源24 V・電流制限5 A、周期・心拍・lease・disable等を維持する。0.05／0.1／0.15／0.2 Hzの設定・判定は変更しない。実速度は操作B全区間のbagで**事後に3.5 rad/s以内**を判定する。実速度3.5超・実位置±1.65超それ自体に基づく自動停止はない。異常時は本人が先に電源出力を切る。NGは収録・停止の成否と分けて報告し、事後の閾値緩和・同条件再試行は行わず、原因・対応のレビュー後に新しいpreflightで再開する。

CSV／bag境界試験、controller全軌道・拒否試験、正常／途中Ctrl+Cのvcanと復元可能rawを準備し、準備レビュー承認後に本人が実機を行う。実機前は0 radの物理姿勢、±1.65 rad全域の接触・配線巻き込みなし、重力で回る負荷なし、電源出力スイッチへの即時アクセスを本人が実物で確認する。今回動画なし。実機結果は別途レビューする。ワークスペースの `docs/plans/20261002_bxi_90deg_03hz_plan.md` を参照。

### 2026-10-02追記：±90°・0.2 Hz専用の指令範囲

本人指定により、固定中心0 rad・振幅π/2・0.2 Hz、前後ramp各2周期10秒・定常1周期5秒・保持1秒後releaseを追加準備する。専用 `controllers_1khz_rt_90deg_02hz.yaml` の**目標速度上限は2.0 rad/s**。定常最大1.973921 rad/s、rampを含む解析的上界1.989282 rad/sであり上限以内。位置±1.65 rad、Kp20・Kd1、上乗せ／摩擦補償0、トルク保護式5 N·m、電源24 V・電流制限5 A、既存周期・心拍・lease・disableを維持する。トルク保護式はKdを含む総トルク上限ではない。

実速度は操作B全区間のbagで**事後に2.5 rad/s以内**を判定する。実速度の大きさによる自動停止はない。超過NGと収録・停止を分けて報告し、事後の閾値緩和・同条件自動再試行をしない。異常を感じたら本人が先に電源出力を切り、原因・対処のレビュー後に新しいpreflightで再開する。この本人指定は新速度の安全性や追従を旧実機・vcanから保証するものではない。0.05／0.1／0.15 Hzの設定・判定を変更しない。今回動画なし、準備レビュー承認後に本人が実機を行う。

2026-10-02：親62c7fc8・HWIFa21e65cの0.2 Hz準備レビュー承認済み。実機は本人が実物4点を確認後に実施し、結果を別途レビューする。実速度2.5 rad/s超・実位置±1.65 rad超それ自体で自動停止する機能はない。目標速度・トルク保護式等の停止とは区別し、異常時は本人が先に電源出力を切る。NG後は判定値を緩めず同条件で再試行しない。実機未実施。

### 2026-10-01追記：固定0°中心の低速大振幅（実機移行のレビュー待ち）

ユーザー本人が出力軸0°中心の−90°〜＋90°を指定し、障害物なし、初回0.05 Hzを選択した。これは既存の小振幅動画計画とは別の拡張である。専用設定 `--sine-90deg` のみで位置安全境界±1.65 rad、指令振幅最大π/2 rad、速度上限1 rad/sを使用する。従来設定の±0.5 radと振幅0.1 radは維持する。低速の定常最大目標速度は約0.493 rad/s、前後rampを含む保守的上界約0.617 rad/s。Kp20・Kd1、摩擦補償なし、トルク保護式5 N·m、心拍・lease・disableを維持する。保護式はKdを含む総トルクの上限ではない。

初回の専用軌道は定常1周期20秒、立ち上げ・立ち下げ各20秒、計60秒＋保持1秒後に脱力。開始位置を0へ戻す操作Aの後も、controllerが操作B開始周期に固定中心との差を検査し、0.05 radを超えたらゲインを適用する前に開始を拒否する。

2026-10-01、ユーザー本人が「±0.5 radを省略し±90°へ進む」と決定した。推奨した中間段階を省略する明示的な判断として記録する。従来0.1 radからπ/2 radへ広げるこの選択は実機合格を意味せず、以下のレビュー・実物確認・低速・停止条件を満たしてから実行する。

実機移行前に、安全手順の更新と2026-10-01の小振幅実機レポート（19:52:35の通信異常を含む）のレビューを完了する。試験者本人は交換後のモーターの0 radが希望する物理姿勢か、余裕を含む±1.65 rad全域に接触・ストッパー・配線の巻き込みがないか、重力負荷のない固定かを実物で確認する。

`can_timeout=0`なので、ホスト停止・プロセス異常やCAN通信異常では最後の位置・Kp・Kdの指令が残り、±90°付近で力を出し続ける可能性がある。ソフトウェアの心拍・lease・disableだけではホスト故障時の停止を保証しない。制御できない動き、振動・異音、ホスト／指令プロセス異常、CAN ERROR-PASSIVE・bus-offなど通信経路に異常がある場合は**先に電源装置の出力を直ちに切る**。その後でランナー・launchを止める。通常の中止はCtrl+Cで脱力・disableを確認し、確認できなければ電源出力を切る。

通信異常後は電源を切ったままログを保全し、電源・配線・CAN設定と状態を確認する。ERROR-ACTIVEへ戻ったことだけで試験を再開しない。原因と対処を確認・レビューしてから、新しいpreflightで再開する。自動再試行や閾値の緩和は行わない。手順・試験結果はワークスペースの `docs/plans/20261001_bxi_90deg_sine_plan.md` と `docs/test-reports/2026-10-01-bxi-90deg-validation.md` を参照する。


**止めるとき**

- **正常終了：** launch と `ros2_control_node` の終了、disable の応答、モーターの文字出力 `STATE : 01 MENU`、CAN の正常状態をその場で確認できた場合は、条件間と最終の正常終了時に電源装置の出力を切る必要はない。出力を維持したことと確認時刻を記録し、次の試験の直前に出力 ON と電源遮断を担当する試験者を改めて確認する。MENU 表示などを確認できない場合は出力を切り、原因を調べてから次の試験を検討する。
- **制御できていない動き**（想定外の動き、止まらない動き）や、**指令スクリプトの異常終了・ホストの異常**のときは、最後の指令が残っている可能性がある。**先に電源装置の出力を直ちに切り**、その後で launch を止める。
- **動きが制御下にある異常**（異常な温度、CAN のエラー、`Overrun detected` の多発など）のときは、Ctrl+C で止める。disable の応答が確認できないときは、電源装置の出力を切る。
- bus-off になったら、下記の手順で止めて復帰する。

**bus-off からの復帰**

`restart-ms 0` では、bus-off になると `can0` は自動では復帰しない。そのあいだは disable を含めてフレームを送れないので、モーター（`can_timeout=0`）は最後の指令を保持する。HWIF 側は送信に失敗して安全停止する想定だが、実機では確かめていない。

1. **電源装置の出力を切る。** モーターを止める唯一の確実な手段である。
2. HIL の launch を Ctrl+C で止める。
3. 状態を記録する。bus-off の直後の値を残し、復帰で消える前に保存する。
   ```bash
   ip -details -statistics link show can0   # state BUS-OFF、TEC/REC、bus-off / bus-error の回数
   sudo dmesg | grep -i -E "can0|peak" | tail -20
   ```
4. 原因を調べる。
   - 両機器の電源を切った状態で、CAN_H–CAN_L 間の抵抗を測る（約 60 Ω が正常）。
   - コネクタと配線を見る。
   - `can0` へ送信しているほかのプロセスがないか確かめる。
   - `can_setup.sh` のビットレートとサンプル点を確かめる。
5. `can0` を設定し直して UP する。`can_setup.sh` は、いったん DOWN にしてから設定し直して UP する。これでコントローラの状態とエラーカウンタが初期化される想定だが、bus-off からの復帰では実機で確かめていない。復帰後の値を必ず確認し、`ERROR-ACTIVE`・TEC/REC 0/0 でなければ先に進まない。
   ```bash
   ./docker/can_setup.sh can0
   ip -details -statistics link show can0   # ERROR-ACTIVE、TEC/REC 0/0、restart-ms 0 を確かめる
   ```
6. モーターの電源を入れ、何も送らずに 10 秒ほど見て、エラーが出ないことを確かめる。これで分かるのは、送信していない間にエラーが出ないことだけである。
   ```bash
   timeout 10 candump -e 'can0,0~0,#FFFFFFFF'   # エラーフレームだけを表示。何も出なければよい
   ip -details -statistics link show can0     # カウンタが増えていないこと
   ```
7. 通信が成り立つことを、ゲイン0の低レート診断で確かめる。`bxi_probe` の `--menu-only` は enable を送らず、disable の応答を確かめてから、ゲイン0・トルク0の指令だけを送る。前後の CAN 統計を比べ、次を確かめる。
   - 20回送って20回応答があること
   - TX・RX が送受信した分だけ増えていること
   - TEC/REC が 0 のままで、bus-error・bus-off・drop が増えていないこと
   ```bash
   ip -details -statistics link show can0
   ros2 run bxi_hardware bxi_probe --iface can0 --can-id 1 --format fd --broadcast --menu-only \
     --stream --count 20 --rate 50
   ip -details -statistics link show can0
   ```
8. この節の「試験ごとに記録すること」からやり直す。再開後の最初の HIL は、ゲイン0の短時間から始める。
9. 同じ日に2回目の bus-off が起きたら、その日の試験は中止し、原因が分かるまで再開しない。

## 1. 前提の確認（bxi_probe、モーターは力を出さない）

```bash
ros2 run bxi_hardware bxi_probe --iface can0 --can-id 1 --format classic --count 500 \
  | tee /ws/log/bxi_probe_classic_$(date +%Y%m%d_%H%M%S).log
# can0 が fd on なら、CAN FD でも確認する
ros2 run bxi_hardware bxi_probe --iface can0 --can-id 1 --format fd_brs --count 500 \
  | tee /ws/log/bxi_probe_fd_brs_$(date +%Y%m%d_%H%M%S).log
```

12軸・1 kHzの[事前検証計画](../../../docs/plans/20260927_bxi_1000hz_12axis_feasibility_plan.md)では、ベンダーCAN通信ガイド§3のFDブロードキャストを1軸で確認する。`bxi_probe` の `--broadcast` はCAN FDのID `0x7FF` にモーターID 1用の8バイト無トルク指令を載せる。最初は `--menu-only` を併用する。このモードはEscの後に個別IDへdisableを送り、モーターID 1のMIT応答がある場合だけ無トルク指令を送る。明示的に `02 MOTOR` が表示された場合は送らない。enableは送らない。文字の `STATE` 行がない場合はMENU表示の確認はできていないと記録する。2026-09-28の実機では、disableの応答はあったが `STATE` 行は出なかった。実機の固定・電源遮断手段・競合プロセス不在を確認し、まず短い低レート試験で応答の有無を記録する。モーターID 2以上や24バイトの3軸指令は、このオプションの対象外。

`bxi_probe` は実送信開始間隔と有効応答取込間隔から、設定値とは別に実測Hzと間隔分布を表示する。`--menu-only` では無応答または送信失敗を検出した時点で追加の指令を止める。間隔の値はユーザー空間の単調時計によるもので、CAN上の送出・到着時刻ではない。

逐次送受信の `--rate 1000` は実機で1,000送信/1,000応答だったが、実送信は平均856.7 Hzだった。各応答を待つ方式のままでは1 ms周期に届かない。送信周期と受信を切り分ける `--stream` は、`--menu-only --broadcast` と併用したときだけ使える診断モードである。独立した受信ソケットを使い、予定送信数、実送信数、応答数、スキップ数と両方の時刻間隔を報告する。無応答が指定timeoutを超えた場合や異常温度・送信失敗では追加送信を止め、disableを試みる。まず低レートの短時間試験で動作を確かめる。応答に要求番号がないため、送信数と応答数が一致しても個別の指令との対応は確認できない。

実機の送受信分離診断は1 kHz・100周期で100送信/100応答・スキップ0だったが、応答取込間隔の最大は2.081 msだった。以後の出力にはカーネル受信時刻の間隔も表示し、受信がカーネルに届く段階の遅れとユーザー空間への取込遅れを分ける。カーネル受信時刻は`CLOCK_REALTIME`、送信開始とユーザー空間取込は単調時計なので、異なる時計の絶対値を直接比較しない。

新表示を使った1 kHz・100周期の再測定でも100送信/100応答・スキップ0だったが、応答取込の最大間隔は2.009 ms、カーネル受信の最大間隔も2.011 msだった。平均送信1 kHzと全数応答だけで「毎1 msに新しいフィードバック」を達成したとは判断しない。ソフトウェア受信時刻はオンワイヤ到着時刻ではないため、モーター・アダプタ・ドライバの切り分けにはハードウェアタイムスタンプまたは外部計測が必要となる。[診断レポート](../../../docs/test-reports/2026-09-28-bxi-1000hz-feasibility.md)に現在の判定をまとめた。すべてdisable応答後、enableなしの試験である。

`peak_pciefd` の上流Linux 6.8ソースには、受信DMA割り込みの既定条件に約1 msの時限がある。原因かどうかは、次の診断で同じフレームのrawハードウェア受信間隔とソフトウェア受信間隔を比較する。新しい出力の「周期開始時の最新受信経過」は、制御側がその時点で読める最後のフィードバックの経過時間で、モーター内部の測定時刻は分からない。ハードウェア受信時刻が欠測すれば失敗扱いになる。実機の固定は確認済み。電源遮断は試験者が担当するが、その手段と直前の確認時刻を記録してから実行する。`restart-ms 100` の設定意図は未確認であり、bus-off後の自動復帰があり得る。

最初の切り分けは、既存と同じ無トルク・enableなしの20周期・200 Hzで行う。コンテナ内で最新の `bxi_hardware` をビルドしたうえで、実機担当者が実行する。

```bash
ros2 run bxi_hardware bxi_probe --iface can0 --can-id 1 --format fd_brs \
  --broadcast --menu-only --stream --hardware-timestamps --count 20 --rate 200
```

2026-09-28の200 Hz実機結果は20送信/20応答、ハードウェア時刻20/20、ハードウェア受信間隔の中央値4.999 ms・最大5.119 msだった。周期開始時の最新受信経過は19件で中央値3.723 ms・最大3.859 ms（初回1件欠測）。この試験で時刻取得を確認した。電源遮断手段と確認時刻はまだ記録されていない。

同条件の1 kHz・100周期では100送信/100応答、ハードウェア時刻100/100、送信スキップ0。ハードウェア受信間隔は0.911〜1.048 ms、カーネルソフトウェア受信間隔は0〜2.032 msだった。周期開始時の最新受信経過は最大1.712 ms、1 ms超過38/98。ハードウェア受信時刻付与後のホスト経路で受信のまとまりが発生している。受信DMA割り込み時限が有力な原因だが、変更前後の比較はまだ行っていない。詳しくは[診断レポート](../../../docs/test-reports/2026-09-28-bxi-1000hz-feasibility.md)。

```bash
ros2 run bxi_hardware bxi_probe --iface can0 --can-id 1 --format fd_brs \
  --broadcast --menu-only --stream --count 20 --rate 200
```

```bash
ros2 run bxi_hardware bxi_probe --iface can0 --can-id 1 --format fd_brs \
  --broadcast --menu-only --count 20 --rate 50
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
ros2 topic echo /joint_states --once                     # 位置の確認（温度は /dynamic_joint_states。起動ログの Enable 行にも出る）
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
  目標速度の上界が `--max-velocity`（既定 2 rad/s）を超える（上界は 振幅 × 2π × 周波数 ＋ 振幅 × π /（2 × 立ち上げの時間）で、振幅を立ち上げ・立ち下げる間の包絡線の微分を含めた真の最大より小さくならない。2026-09-29 に Python 版・C++ 版とも修正。それまでは振幅 × 2π × 周波数だけを見ていて、立ち上げを短くすると、例えば振幅 0.1 rad・0.5 Hz・立ち上げ 0.000001 周期では立ち下げ中の目標速度が約 78,540 rad/s になっても通っていた）、`--center` が今の位置と違う。
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

### C++ 版: 等周期の SIN 波指令

同じ launch を起動した状態で、`colcon build --packages-select bxi_hardware --symlink-install` 後に別の端末から実行する。

```bash
source /ws/install/setup.bash
ros2 run bxi_hardware sine_command_cpp --amplitude 0.1 --frequency 0.5 --cycles 5 \
  --kp 20 --kd 1 --rate 200 --release
ros2 run bxi_hardware sine_command_cpp --amplitude 0.3 --frequency 0.2 --cycles 5 \
  --kp 20 --kd 1 --rate 200 --release
```

既定の中心は開始時の現在位置。目標位置・目標速度は Python 版と同じ立ち上げ／立ち下げ付きの SIN 波で、計算は `bxi_hardware::hil::SineTrajectory`（`include/bxi_hardware/hil_trajectory.hpp`。Python 版と同じ値になることを `test/test_hil_trajectory.cpp` の表の試験で確かめている）、`/joint1_command/commands` に送る。`--friction-ff` などの摩擦補償オプション、位置・速度・トルクの上限も Python 版に合わせてある。`--help` で引数一覧を確認できる。

送信予定は `steady_clock` の絶対時刻で 5 ms 間隔に置く。処理が遅れて次の予定を過ぎた場合は古い指令を連続送信せず、その周期を飛ばして現時刻の目標を送る。実行後に予定時刻を過ぎた周期数と最大遅れを表示する。実際の指令時刻・状態を `sine_cpp_<日時>.csv` に記録する。CSV の先頭 7 列は Python 版と同じで、末尾の `lateness_ms` は予定時刻からの遅れ。振幅一定区間の RMS 誤差、最大誤差、振幅比、位相遅れも表示する。

状態が 0.2 秒以上届かない、指令の上限を超える、Ctrl+C の場合は Kp=Kd=0 の指令を 5 回送って終了する。通常終了時は中心位置を保持し、`--release` を指定すると力を抜く。C++ と `sleep_until` だけでハードリアルタイムになるわけではない。ROS 2 の配送、controller_manager と CAN の周期も独立しているため、CSV の時刻は CAN バス上の送信時刻を表さない。最初は可動範囲と荷重を確認して小振幅から実行する。

### RT 軌道 controller（`bxi_hardware/HilTrajectoryController`）

ramp と SIN 波の目標を、トピック経由のスクリプトではなく、controller_manager の `update()`（1 kHz）の中で毎周期計算する controller。計画はワークスペース側の `docs/plans/20260929_bxi_rt_trajectory_controller_plan.md`、vcan での結果は `docs/test-reports/2026-09-29-bxi-rt-trajectory-vcan-step4.md`（手順4、2026-09-30 に完了）。段階3R・4R用の `controllers_1khz_rt.yaml` で、2026-09-30 に実機の段階3R（ramp）・段階4R（SIN 波の最初の条件）に合格した（ワークスペース側の `docs/test-reports/2026-09-30-bxi-hil-stage3r-rt-ramp.md`・`2026-09-30-bxi-hil-stage4r-rt-sine.md`）。既存の `controllers_1khz.yaml` は変更していない。controller の一覧と組み合わせ方は [`docs/CONTROLLERS.md`](../docs/CONTROLLERS.md)。

- 実行環境：`single_joint_hil.launch.py` は `ros2_control_node` に `LD_BIND_NOW=1` を渡す（遅延バインディングのままだと、開始の周期の `update()` で動的リンカがシンボルを解決し、35〜92 µs かかった）。`controllers_1khz_rt.yaml` は controller_manager の更新スレッドを CPU 11 に、`bxi_hardware_stream_1khz.yaml` は送受信スレッドを CPU 9 に固定する（`cpu_affinity`・`thread_cpus`。ワークスペース側の `docs/plans/20260930_bxi_cpu_affinity_plan.md`）。vcan での `update()` の最大は 31.1 µs。

- 使うインターフェース：command は `<joint>/position`・`velocity`・`kp`・`kd`・`effort`、state は `<joint>/position`・`velocity`・`effort`（`joint1_command` と同じ command を使うので、同時には有効にできない）。
- パラメータ：`joint`（既定 `joint1`）、`lower` / `upper`（±0.5 rad）、`max_kp`（20、30 まで）、`max_torque`（5 N·m）、`max_velocity`（2 rad/s）、`max_period`（5 ms）、`arm_timeout`（1 秒）、`record_tail`（0.5 秒）、`record_seconds`（120 秒）、`max_run_seconds`（60 秒）、`output_dir`（空なら環境変数 `BXI_WORKER_TIMING_DIR`）。
- 試験専用のパラメータ：`test_fault.start_delay`・`stop_delay`・`finish_delay`（秒、既定0、30 まで）。サービスの処理が要求を扱う前にその時間だけ止まる（vcan で応答の遅れ・応答なしを再現するため）。0 でない値は、環境変数 `BXI_ALLOW_TEST_FAULTS=1` がないと設定でエラーになる。HIL のランナーは launch の前にこの変数を消す。実機では使わない。
- サービス：`~/start`（`bxi_hardware/srv/StartHilTrajectory`。種類 hold / ramp / sine と数値。受け付けたかと要求番号を返す）、`~/finish`（保持中だけ。正常終了）、`~/stop`（いつでも。停止を優先）。
- `~/keepalive`（`std_msgs/UInt32`、要求番号）：適用した要求と同じ番号で、受け付けの後に届いたものだけを数える。最初の1件で軌道を始め（それまでは力を抜いたまま）、1秒来なければ動かずに中止する。実行中と保持中は、最後の keepalive からリースの時間を超えると中止して力を抜く。
- `~/status`（`bxi_hardware/msg/HilTrajectoryStatus`、約 50 Hz）：状態、適用した要求番号と停止の数、終わりの理由、経過時間、目標と位置、記録の状態と CSV のパス。受け付けと適用の区別はここで確かめる。
- 中止の条件（その周期のうちに位置 = 現在位置、速度 0、Kp = Kd = 0、トルク 0 を書く）：Kp × |目標 − 位置| + |上乗せ| > `max_torque`、|目標速度| > `max_velocity`、位置・速度・トルクのどれかが有限でない、周期（単調時計の `period`）が 0 以下・`max_period` 超、リースの途切れ、開始の周期での範囲外（SIN 波の中心 ± 振幅）と実行時間の超過。
- command への書き込みが1つでも失敗したら（`set_value()` がロックを取れない）、実行中・保持中はその周期のうちに中止（`write failed`）して、力を抜く値を書き直す。`write_failures` は書き込みに失敗した周期を1周期1回だけ数え、同じ周期の脱力値の書き込みが成功しても CSV の `write_ok` は 0 にする。脱力値の書き込みは毎周期試みる。今回の実行で一度も5項目をすべて書けていない間は、`~/status` の `release_write_pending` を true にする。成功するまで記録を完了せず、次の開始も受け付けない。
- `release_write_pending` が続く間は、controller だけでは脱力を保証できない。`hil_trajectory.py` は `write_failures` の増加を検出すると `[中止]` と終了コード1で終わる。段階3R・4Rのランナーは、これを NG として launch を止め、HWIF の disable 応答を確認する。disable を確認できなければ電源装置の出力を切る。ランナーのこの経路は vcan で確かめたが、実機ではまだ行っていない。
- `~/start` と `~/finish` の受け付けの判断は、`update()` が毎周期必ず書く atomic の状態で行う（`~/status` の写しは try_lock なので古いことがある）。開始要求を取り出した周期から、記録を書き出し終えるまでは次の開始を受け付けない。
- 経過時間は `period` の積算で数える（`update()` の `time` は ROS の時計で飛ぶことがあるので使わない）。
- 毎周期の記録：開始要求を適用した周期から、力を抜いた周期の `record_tail` 後まで、あらかじめ確保した領域に書き、RT ではないタイマーが CSV（`hil_trajectory_<要求番号>_<時刻>.csv`）に書き出す。書き出すまで次の開始は受け付けない。書き出しに失敗したら記録を残したまま1秒ごとに再試行し（`~/status` の記録の状態は「失敗」）、その間も次の開始は受け付けない。実行の途中で controller が無効化されると（launch の停止）、記録は終わりの理由「deactivated」で締まり、タイマーが書き出す前にプロセスが終わっても、controller のデストラクタで書き出す（controller_manager はハードウェアの disable の後に controller を破棄するので、disable は遅れない）。CSV の最後の列 `write_ok` は、その周期の書き込みがすべて通ったか（1 / 0）。満杯になったら欠落した周期を数える（制御は続ける）。
- `update()` はメモリを確保せず（単体試験で確かめている）、止まる可能性のあるロックを取らない（try_lock と atomic だけ）。

`hil_trajectory.py` は手順3の指令コマンドで、controller に開始を要求して要求番号付き keepalive を20 Hzで送り、正常終了・CSV 保存まで確認する。SIN 波では保存した controller CSV の振幅一定区間から追従誤差、振幅比、位相遅れ、停止時間を集計する。`~/start` の応答なし、`~/finish` / `~/stop` の異常、心拍切れ、書き込み失敗、Ctrl+C は終了コード1と `[中止]` で区別する。異常を検出したら、`~/stop` を呼ぶ前に keepalive を止める（`~/stop` が応答しなくても、リースの時間で力が抜ける）。停止の適用は保存した要求番号で確かめる。開始は、controller が待機中か、前の実行が正常に終わって（`finish` か `release`）記録を書き出した後にだけ行う。前の実行が中止で終わっていれば開始しない（launch からやり直す）。引数の例（controller を別途起動した vcan 用）:

```bash
ros2 run bxi_hardware hil_trajectory.py --kp 10 --kd 2 --hold 1 hold
ros2 run bxi_hardware hil_trajectory.py --kp 10 --kd 2 --speed 0.3 --release ramp 0.2
ros2 run bxi_hardware hil_trajectory.py --kp 20 --kd 1 --amplitude 0.1 --frequency 0.5 --release sine
```

段階3R・4Rのランナー（ワークスペース側の `tools/hil_stage3r.sh`・`hil_stage4r.sh`）は、指令コマンドの異常終了を NG として launch を止め、HWIF の disable の応答を確かめる。この経路は vcan で確かめた（ワークスペース側の `tools/verify_hil_runner_vcan.py`）。指令コマンド単体の正常終了はモーターの disable を示さない。

vcan での確かめ方（開発コンテナの中、vcan0 を mtu 72 で作ってから）：

- `python3 /ws/tools/verify_rt_trajectory_vcan.py --scenario <名前>`：launch・模擬モーター・`candump` を起動し、指令コマンドを動かして判定する。`normal`・`sequence` のほか、停止と故障の経路（`sigint`・`heartbeat`・`kill9`・`kill_holding`・`start_lost`・`start_delay`・`stop_delay`・`stall`・`stop_all_blocked`）。故障の経路は、上記の `test_fault.*` と、試験専用の `bxi_hardware/HilUpdateStallController`（別ライブラリ `bxi_hil_test_controllers`。`~/stall` の後の `update()` で `stall_ms` だけ止まり、controller_manager の更新の停止を再現する。`BXI_ALLOW_TEST_FAULTS=1` がないと設定でエラー）を使う。
- `python3 /ws/tools/verify_hil_runner_vcan.py --stage stage3r|stage4r [--scenario normal|motion_no|disconnect|kill_command]`：ランナーのコンテナ側を、ホスト側の代わりをしながら動かす。ランナーには `BXI_HIL_VCAN_IFACE=vcan0` を渡し、CAN 設定の can0 を vcan に差し替えさせる（ホスト側の入口は設定しない）。

### SIN 波の結果（2026-09-25、交換前のモーター、Kp 20 / Kd 1、classic 200 Hz）

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

- 送受信分離・1 kHz・ブロードキャスト（2026-09-29、HIL 段階4）では、振幅 0.1 rad・0.5 Hz・打ち消しなしで、誤差 RMS / 最大 0.0225 / 0.0391 rad、振幅比 0.84、位相の遅れ 16.1°、止まっている時間 29 % だった。上の表の同じ条件より目標に近いが、各1回で日や中心の位置なども異なるため、1 kHz 化による改善とは結論していない。
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

## 3. 結果（2026-09-25、BXI7010-19（交換前のモーター）、can0 classic、200 Hz）

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

### ±90°・0.1 Hz追加準備（2026-10-02）

ユーザー指定の0.1 Hzは定常1周期10秒、前後ramp各2周期20秒、保持1秒後releaseとする。速度上限1 rad/sと位置±1.65 rad等の保護は維持する。C++/Pythonの解析的速度上界はCauchy-Schwarzにより A*hypot(ω,π/(2Tr)) とし、今回約0.994642 rad/s。max_velocityは目標速度の保護で、実速度の大きさによる自動停止はない。0.1 Hzの実速度は操作Bのbagで事後に1.5 rad/s以内を判定し、超過はNGとする。0.05 Hzは従来1 rad/s以内を維持する。実機追従・実速度は未確認であり、実機は準備レビュー後に行う。従来0.05 Hzの既定を保持し、ホストstage4rの `--sine-90deg --sine-frequency 0.1 --video-confirm --trace-cycles 620000` で明示選択する。

現行条件は本人の追加指定「実速度を1.5 rad/sまで許容」に基づく(a)。0.1 Hzのみ操作Bの実速度事後判定1.5 rad/s、0.05 Hzは1 rad/sとする。目標速度上限1 rad/sと既存保護を維持し、実速度の大きさによる自動停止は追加しない。1.5 rad/s超はNG、事後閾値変更・自動再試行はしない。2026-10-02に親a48ecb9・HWIF3749633の準備レビュー承認済み。実機未実施。試験者による実物確認後に実施し、結果は別途レビューする。

### ±90°・0.15 Hz追加準備（2026-10-02）

本人指定0.15 Hzと実速度2.0 rad/s許容に対応する専用controller設定 `controllers_1khz_rt_90deg_015hz.yaml` を用意。固定中心0 rad、振幅π/2、定常1周期約6.667秒、前後ramp各2周期約13.333秒、保持1秒後release。目標速度上限1.5 rad/s、定常最大1.480441、解析的上界1.491962 rad/s。実速度は操作B全区間のbagで事後に2.0 rad/s以内を判定し、実速度の大きさによる自動停止はない。位置±1.65 rad・トルク保護式5 N·m・既存停止経路を維持。0.05／0.1 Hzの目標上限1.0と実速度判定1.0／1.5は変更しない。超過NG・収録・停止を区別し、事後閾値変更・自動再試行を行わない。準備レビュー待ち、実機は本人が行う。ホストstage4rで `--sine-90deg --sine-frequency 0.15 --trace-cycles 620000` を明示し、撮影する場合のみ `--video-confirm` を追加する。
