# write 起点の送信を比較する

`tx_trigger` の既定値は `periodic`。既存のHIL設定はそのまま旧方式を使う。
新方式は `rx_mode: separate` と `tx_trigger: write` を選び、測定した
`stream_write_grace_ms` を明示した設定で使う。受信の直接公開も同時に有効になる。

## 実機でGを決める必須手順

新方式で実モータの試験を始める前に、**実際のcontroller_managerと実モータを使い、周期方式でwrite間隔を再測定する**。
模擬read/writeの2000周期、仮想モータ、今回の0.15 msを、この測定の代わりにしない。

1. 既存HILの前提を満たし、まず周期方式・ゼロゲインで起動停止を確認する。
2. controller_managerの版、制御周期、controller、モータID、軸/バス数、CPU/IRQ配置を記録・固定する。
   比較対象の動作・負荷・準備待機を含めて周期方式で採取する。既存RTランナーの `--control-trace` で
   実際の `read()/update()/write()` のうちハードウェアread/writeの時刻を記録する。
   実機で使用するoverlayへ今回のコードをビルド/installし、その環境の版を記録して両方式を比較する。
3. `analyze_write_stream.py` で実測write間隔の超過のp99/p99.9/最大とread→writeの時間を確認し、
   観測した裾と余裕からGを決定する。決定用の実行ログと選択理由を保存する。
4. `T+G+TX遅延+最も遅い軸の応答公開まで < 応答鮮度上限` と `T+G < 指令鮮度上限` を実測で確認する。
   両立しなければ新方式の評価へ進まない。しきい値を暗黙に伸ばさない。
5. 別の評価データで、まず新方式のゼロゲイン、小さい動作、最後に90°試験の順で確認する。
   正常運転の再送・再送直後の新write・各種保留・次readへの期限違反0件を検査する。
   軸数、モータ、controller、CPU/IRQ配置、負荷条件が変われば再測定する。

## 設定の作成

ワークスペースで、既存設定から別ファイルを生成する。元ファイルも既存の出力先も上書きしない。

```bash
python3 src/bxi_hardware/scripts/make_write_stream_config.py \
  src/bxi_hardware/hil/single_joint/bxi_hardware_stream_1khz.yaml \
  /tmp/bxi_hardware_direct_1khz.yaml --direct-only

python3 src/bxi_hardware/scripts/make_write_stream_config.py \
  src/bxi_hardware/hil/single_joint/bxi_hardware_stream_1khz.yaml \
  /tmp/bxi_hardware_write_1khz.yaml --grace-ms "$MEASURED_G_MS"
```

`MEASURED_G_MS` は、その実機の旧方式のwrite間隔の超過から決め、別の評価データで確かめる。
1 kHzでは `1 ms + G + 送信遅延 + 最後の軸の応答公開まで < 3 ms` を確認する。
設定検証では `T + G < feedback_max_age` と `T + G < command_max_age` を検査するが、
バス・モータの応答時間の余裕は実測で確認する必要がある。仮想CANの0.15 msは実機用の値ではない。

生成したwrite設定は `stream_first_write_timeout_ms: 20` を明示する。
これは全バスの初期応答確認後、通常writeの受付を開始してからの期限。
最初のNaN指令から作るゼロゲインwriteも正常な初回writeに含める。
実機起動で受付開始→初回writeを測り、必要なら設定を調整する。
write方式のホスト事前確認では、実CM・実モータの再測定と応答予算確認を必須の確認項目として記録する。

従来の1軸HIL手順に沿って、まずゼロゲインで確認する。
`single_joint_hil.launch.py` の `config_file` に生成ファイルを指定し、
controllerの設定は既存の `controllers_1khz_rt.yaml` を使う。
既存RTランナーは、`--write-grace-ms "$MEASURED_G_MS"` を明示すると、選択した設定からwrite方式の設定を
記録先へ生成して使う。省略時は周期方式。例えば既存90°・0.5 Hz試験の設定選択と62万周期を維持できる。

```bash
# 周期方式で実CM・実モータのG決定用データを採取（既存の実機前提・操作確認に従う）
bash tools/hil_stage4r.sh --sine-90deg --sine-frequency 0.5 --trace-cycles 620000 --control-trace
# Gを選び、独立した新方式の評価を実施するとき
bash tools/hil_stage4r.sh --sine-90deg --sine-frequency 0.5 --trace-cycles 620000 --write-grace-ms "$MEASURED_G_MS"
```

## 計測

`BXI_WORKER_TIMING_DIR` に既存の書き込み可能なディレクトリを指定すると、
write方式・直接RX方式ではTX/RXに加えて `*_control.csv` を出力する。
旧方式で同じ制御側の記録を取るときは `BXI_CONTROL_TRACE=1` を追加する。
これらの条件ではTX/RXの形式はversion 3になる。

```bash
python3 src/bxi_hardware/scripts/analyze_write_stream.py /path/to/run
```

公開→send、write間隔、軸ごとの取得失敗・取得した値の古さ、再送、保留をまとめる。
`next_read_temporal_envelope_*` は送信後の最初の軸別RX公開が次readに間に合ったかの時刻判定。
MIT応答には指令通番がないため、実機で指令と応答が対応したことを証明する値ではない。
`published_ns` は格納直前の時刻、`notify_before/after_ns` は公開・通知APIの前後、
`wake_steady_ns` はTXの評価ループ開始を記録する。スケジューラの起床だけを分離した値ではない。

多軸のreadは各軸の最新値を個別に読む。全軸の応答を待たない。
更新中の競合では最大2回の取得で戻り、前回の整合した値を使う。
3制御周期連続の取得失敗は診断フラグを立てる。これだけでは自動停止しない。
通信鮮度と指令鮮度は別に送信スレッドが監視する。

トレース合計は256 MiBまで。CSVの値・列を維持し、RAM内の重複情報・アラインメントの余白を減らした。
1軸62万周期のTX/RX/制御/診断ペイロードは263,508,192 bytes（約251.3 MiB）で収まる。
既存90°ランナーの62万周期条件を変える必要はない。多軸・多バスは引き続き合計をenable前に検査する。

unicastは送信前に最新値を選ぶ。1フレームでも送り始めた一式は、途中のwriteで変更・打ち切りせず、
同じ指令のまま全軸へ送る。次の一式で最新値を採用し、T/2は一式の先頭同士で守る。
停止要求、温度異常、指令鮮度切れ、送信失敗は各フレーム前に確認し、中断・部分送信を記録する。
温度の初回異常値・軸・受信時刻は正常フレームで上書きせず保持し、停止後に軸名/ID付きログと
`.complete`へ出力する。両センサー故障は温度nullと専用フラグで区別する。

## 仮想CANの再現

Jazzyコンテナ内で基準と評価を別ディレクトリに採取する。
検証スクリプトは新しいvcan97だけを作成し、終了時に削除する。
既存のvcan97がある場合は実行を拒否する。

```bash
bash tools/verify_bxi_write_vcan.sh log/write_calibration periodic
# 基準データからGを選び、次の独立データで確認する。
bash tools/verify_bxi_write_vcan.sh log/write_evaluation all "$MEASURED_G_MS"
bash tools/verify_bxi_write_cm_vcan.sh log/write_cm "$MEASURED_G_MS"
```

制御相当のread/writeはCPU 11/FIFO 50、TX/RXはCPU 9/FIFO 45/46。
仮想モータはCPU 18/FIFO 60、応答予定は300 µs後。実機の割り込み配置の検証は別に行う。
前後のスレッド・割り込み記録も残す。実機の使用条件は [HIL手順](../README.md) を参照する。

既存90°ランナーの仮想CAN通し検証は `verify_hil_runner_vcan.py` の `--write-grace-ms` でも行える。
検証用overlayを選ぶ場合は `BXI_HIL_OVERLAY_SETUP=/ws/install/bxi_write_jazzy/local_setup.bash`、
仮想モータの実行ファイルは `BXI_HIL_EMULATOR=/ws/build/bxi_write_jazzy/bxi_hardware/bxi_vcan_motor_emulator` と明示する。
通常の実機overlayを自動で置き換えない。
