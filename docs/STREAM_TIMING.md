# 送受信分離の詳細記録（v2）

`BXI_WORKER_TIMING_DIR` を既存の書き込み可能なディレクトリに設定すると、停止・joinとdisableの後に、周期・受信CSVとJSONの `.complete` を書き出す。運転中は起動前に確保したメモリへ記録するだけで、ディスク出力・記録領域の拡張はしない。

## 容量

容量の既定値は20,000周期、受信は周期数×軸数×2。`BXI_STREAM_TRACE_CYCLES` を指定すると拡張容量を選ぶ。許可値はASCII数字のみの20,001〜620,000。明示的な20,000以下、不正値、記録先なし、sequential経路、複数バス・多軸・非ブロードキャストはenable前に拒否する。埋め草付き1軸ブロードキャストも対象。容量未指定の既存構成は維持する。

例えば開発コンテナで1軸の別途確認済みHIL設定を使う場合、環境変数を明示して起動側へ渡す。

```sh
export BXI_WORKER_TIMING_DIR=/ws/log/UNIQUE_RUN
export BXI_STREAM_TRACE_CYCLES=620000
```

記録先は実行ごとに一意にして、事前に作る。620,000周期は1 kHzで620秒分であり、別周期の収録時間は異なる。全バスの記録領域・書き出しバッファ・ハードリンク対応・stemの準備を最初のenable前に済ませる。要素領域は合計256 MiBまでで、checked演算・確保例外・実capacityを検査する。実効座標情報はoffset補正とsetAngleRange後にドライバから取得し、originは受信スレッド開始直前に取る。

## CSVと時刻

既存列は順序・意味を維持し、追加列を末尾に置く。metadataは従来解析用の `#period_ns=` で始まり、`format_version=2` を含む。

1軸ブロードキャストの周期CSVには、エンコード前の関節指令（command_position_rad、command_velocity_rad_s、command_kp、command_kd、command_effort_nm）、エンコード後の8バイト（command_block_hex）、send()直前・復帰直後のsteady時刻、有効性・send実行・成功のフラグを追加する。command_sourceはowner、command_stale、none。doubleは安全停止による置き換え後で、encoder内部の飽和・方向換算前。失敗や未実行を成功した指令と区別する。非ブロードキャスト・多軸の送信追加列はtx_detail_supported=0とし、値は空欄。

受信CSVには同じフレームのデコード値（motor_position_rad、motor_velocity_rad_s、motor_torque_nm）とreceived_steady_nsを追加する。元のreceived_nsはoriginからの相対時刻。対象IDのデコード成功分だけを記録する。単位はモーター座標のrad、rad/s、N·m。実効offset・方向を使い `joint_position=(motor_position-offset_rad_effective)*motor_dir` と換算する。受信スレッドからapplyFeedbackは呼ばない。

metadataにはtrace_origin_steady_ns、send_loop_start_steady_ns、stream_stop_steady_ns、容量・確保量・確保時間、実効offset・上下限・codec limits、ブロック数・埋め草8バイトを保存する。多軸の座標情報にはcan_idごとの接尾辞を付ける。無効なsend時刻は空欄。send_loop未開始やstop未実施は-1。外部停止を送信ループ先頭で検知した周期は行を作らないため、最終stopped行を必須にしない。

`trace_synthetic=0` は実際の送受信記録、`1` は書き出し負荷試験の合成データ。workspaceのv2収録検査はこのキーを必須とし、合成データは明示的に許可したvcan負荷試験だけで評価対象外として扱う。キーがなかった旧v2記録は、新しい検査ではNGになる。過去のrawを補完・書き換えず、当時の検査結果と区別する。

`BUILD_TESTING=ON`（colconの既定）のビルドでは、試験用 `HilTraceBenchmarkSystem` pluginも通常のinstall先へ入る。launchの既定は本番pluginのまま。試験pluginは明示的な `BXI_ALLOW_TEST_FAULTS=1` と単一の仮想vcan構成を要求し、実機構成をenable前に拒否する。合成データを実際のCAN周期や操作範囲の評価に使わない。

## 完了印と停止

各CSVはexclusive create、flush・closeの成功を検査する。`.complete.tmp` を同様に書き、linkで `.complete` を上書きなしに公開する。過去ファイルや競合は拒否し、途中ファイルを診断用に残す。公開後のtmp削除失敗は警告となるが、公開済みのcompleteは有効。fsyncによる電源断への永続化は保証しない。

completeは書き出しの完了だけを示し、制御正常や打ち切りなしを意味しない。行数、dropped_cycles・dropped_rx、実際のfault、fault_axis、stopped_externally、stop_reason、時計の起点、CSV書き出し時間を保存する。faultありはstop_reason=fault、それ以外はexternalまたはrequested_stop。

合否時間はreportStream開始からcomplete公開までのreport_to_complete_ns（2秒以下）をログへ出す。complete内のcsv_write_elapsed_nsは診断値。書き出しスレッドのpolicy・priority・許可CPUと、enable前のRSS・VmLck、activation時間もログへ出す。制御周期や実機の安全基準は変更していない。

## workspaceの停止後解析

workspace側の `tools/hil_video_analyze.py RUN --iface can0 --out NEW_DIRECTORY` は、保存・停止後にv2実記録の完了印・行数・時計・操作範囲を確認し、controller・成功send・RXのsteady時系列、skip／失敗／置換、ageと応答再利用、受信空白、指令／応答の図とSIN fitを生成する。NumPy・Matplotlibの依存と使い方はworkspaceの段階3R・4R手順書、手順5の解析レポートを参照する。手順5は実装・検証完了、ユーザーレビュー待ち。

収録範囲検査が使う最終予定slotの終端は実際のstream_stop_steady_nsより後になり得る。TX/RXの解析は実停止で打ち切り、controllerが後から書いた異常検出行は元の操作終端まで保持する。図はslot範囲終端と実停止を区別する。受信値は実効offset・方向を使って関節座標へ換算する。

全8バイトの再エンコードと埋め草を含むcandump候補照合を行うが、候補再利用・複数候補があるため厳密な一対一遅延とはしない。合成記録・旧v2のflag欠落は拒否。CAN欠落は完全性partial、制御faultと書き出し・解析の成否は分ける。取得時と後処理時それぞれの版・ソースsnapshot・SHA256を保存する。
