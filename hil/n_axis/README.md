# G専用N軸最小セット

このディレクトリはN=2／3の設定生成・ROS入口・簡易評価の準備です。実機、CAN、vcan、enable、電源操作の実施承認を与えません。既存single_joint／two_jointとは別の構成です。実配線・所有者・変換前値・RX鮮度・新着静止は確認できません。

## 設定の生成（通信なし）

CAN IDに既定値はありません。個体、現在ID、joint順を別手順で確認し、変数`CAN_IDS`に明示したN個のIDを渡してください。broadcast ABIはID1..Nの順序を要求しますが、それを現在の実機のIDとして仮定・検出・変更しません。

```sh
# 既存ディレクトリと区別できる、新しいrun専用パスを明示する
mkdir "$RUN_DIR"
ros2 run bxi_hardware make_n_axis_config.py --axes "$N" --ids $CAN_IDS --destination "$CONFIG_DIR" --output-dir "$RUN_DIR"
xacro "$(ros2 pkg prefix --share bxi_hardware)/hil/n_axis/n_axis.urdf.xacro" axes:="$N" config_file:="$CONFIG_DIR/hardware.yaml" spec_dir:="$(ros2 pkg prefix --share bxi_hardware)/config/specs" > "$CONFIG_DIR/robot.urdf"
ros2 run bxi_hardware n_axis_check_config.py "$CONFIG_DIR" "$CONFIG_DIR/robot.urdf"
```

生成スクリプトは既存validate_config.pyも実行します。設定生成先は新規のみ、output_dirは既存かつconfigure時に書込可能である必要があります。manifestは明示引数の申告と生成結果の対応を保存するだけで、改ざんされていないことやIDの物理的由来を証明しません。構成確認スクリプトは任意の他plugin・所有者・実配線を検知できず、完全な構成検査の代替ではありません。起動対象はjoint_state_broadcasterと単一のn_axis controllerだけです。

## 別承認後の起動入口

次はG3／G4の別承認後に使う入口の説明であり、この実装の単体試験ではlaunchを起動していません。

```sh
ros2 launch bxi_hardware n_axis_hil.launch.py axes:="$N" config_dir:="$CONFIG_DIR"
ros2 run bxi_hardware n_axis_command.py start --axes "$N" --kp 20 --kd 1
# holding時のみfinish。stopはcontroller内部の正常脱力要求。
ros2 run bxi_hardware n_axis_command.py finish
ros2 run bxi_hardware n_axis_command.py stop
```

launchファイルはパッケージの`hil/n_axis/`へinstallされます。環境によってbasename解決されない場合は同ディレクトリのlaunchファイルを明示してください。launchは`RUN_DIR/hwif`を新規作成し、既存`BXI_WORKER_TIMING_DIR`へ渡します。既存runディレクトリの再利用を避けてください。3台目のSN、現在CAN ID、配線は未確認です。ここに既定値や推定値を入れていません。

ROS入口はcontroller相対の`~/n_axis/`（既定controller名`n_axis`なら`/n_axis/n_axis/`）。startはStartNAxis、finish／stopはTrigger、keepaliveはNAxisKeepalive、statusはNAxisStatusです。keepalive／statusのQoSはreliable、depth1、volatile。status周期の暫定既定は50ms（20Hz、設定範囲20〜50ms）。ツールは同一ホストのmonotonic_nsを送信時刻に使います。別ホスト時計は対象外です。

startはIDLEから1回だけ、固定±pi/6、0.2Hz、定常5周期、前後1周期、hold1秒、release=true。keepaliveは20Hz、nonceは実行ごとのランダム64bit、sequenceは単調増加。異常・通信途絶・例外後に自動再startしません。Ctrl-C／例外では心拍を止め、stopを最大1回だけ試みます。電源・enable操作はありません。

正常released／stoppedでも、外部のCM全stop／deactivateまではcontrollerが脱力commitを続け、record_completeはfalseです。ツールの終了表示に`not saved`と出た場合は保存成功ではありません。外部全stopの運用とexecutorが残る時間はG3で確認します。deactivate中にCSVを書かず、非RT timerがrecordCompleteを見てsaveRecordを1回だけ試みます。ファイル名にactivation／nonce／steady時刻を含み、statusのrecord_stateは0未保存／1保存済み／2失敗です。保存失敗はNGで、再試行しません。unloadやプロセス終了でtimerが走らず原本が欠落する場合もNGです。

## 簡易評価

```sh
ros2 run bxi_hardware n_axis_evaluate.py "$CONTROLLER_CSV" --json "$RESULT_JSON"
# 任意: --pair-commits PATH --pair-safety PATH --pair-feedback-stale PATH
```

各項目をPASS／FAIL／NOT_EVALUATEDで出します。原本欠落・列不一致・dropは成功扱いにせず、測定区間不足はNOT_EVALUATEDです。q/v独立再計算差の暫定閾値は1e-9、定常区間の全軸対位置差0.10rad・位相差10度は設計の仮値（CLIで変更可能）。軸別RMS／最大誤差、振幅比、中心も出しますが、個別追従の承認済み閾値はなく、これらは記述統計です。位相の定義できない振幅、fit不能はNOT_EVALUATED。平均だけで片軸・軸対の違反を消しません。

HWIFファイルがある場合は採用gの被覆・欠落と安全／stale原本の内容を要約します。gだけではHWIF epoch／controller run対応を証明できないため、HWIF全体項目はNOT_EVALUATEDのままです。内容要約はsend結果の合格判定ではありません。readbackはHWIFのackではなく、物理安全・モータ適用・24バイト通信の成立を評価しません。

暫定設定：first_write timeout1000ms、feedback3ms、command5ms、write grace0.20ms、controller1kHz・優先度50／CPU11、TX45／RX46／CPU9、API試行1、record120秒。launchは単一spawnerに`n_axis`、`joint_state_broadcaster`の順で渡します。使用中のros2_controlのspawnerは各controllerのload・configure・activateを終えてから次へ進むため、n_axisのactivateが先になります。joint_state_broadcasterの起動失敗・遅延を先に待つ構成にはしません。

従来の並列spawnerでのvcan起動20試行では、起動順とA0からの初回世代採用の遅延が対応し、初回write timeoutも1回発生しました。遅延の因果は未確定です。単一spawnerでも1000msの起動期限に常に収まる保証はなく、起動余裕の分布、RT配置・負荷、最終保存までのexecutor寿命はG3で評価します。これらは物理停止時間や50µs達成の保証ではなく、実機での効果も未評価です。

実行前は[CHECKLIST.md](CHECKLIST.md)を確認し、独立した実機実施承認を得てください。
