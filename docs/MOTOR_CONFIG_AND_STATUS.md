# モーターの設定とステータスの確認方法

BXI モーター（1軸の HIL では BXI7010-19、`can_id` 1）の設定とステータスを、どこで・どう確かめるかをまとめる（2026-09-30 時点）。「設定」には、`bxi_hardware` 側の設定（yaml）と、モーター本体（ファームウェア）の設定の2種類がある。

手順の全体は [`../hil/SINGLE_JOINT_GUIDE.md`](../hil/SINGLE_JOINT_GUIDE.md)、プロトコルの詳細は [`PROTOCOL_AND_SAFETY.md`](PROTOCOL_AND_SAFETY.md)。

## 1. 設定

### 1a. `bxi_hardware` 側の設定（yaml）

モーターをどう扱うかは、HWIF の設定ファイルに書く。1 kHz の構成では `hil/single_joint/bxi_hardware_stream_1khz.yaml`、200 Hz・Classic の構成では `hil/single_joint/bxi_hardware.yaml`。

| 項目（`joints.<関節名>`） | 内容 | 1 kHz 構成の値 |
|---|---|---|
| `can_bus` / `can_id` | つなぐバスとモーターの ID | `can0` / 1 |
| `motor_type` | モーターの機種。MIT の値の範囲を `config/specs/<機種>.yaml` から読む | `MOTOR_70` |
| `motor_dir` | 回転の向き | 1 |
| `lower` / `upper` | 関節の範囲 [rad]（出力軸、ユーザーゼロ点基準） | ±0.5（ベンチ用の狭い範囲） |
| `default_kp` / `default_kd` | 位置だけを指令し、Kp・Kd を指令しなかったときのゲイン | 0 / 0 |

バスごとの設定（フレーム形式 `can_frame_format`、送受信分離 `can_transport`、停止のしきい値、送受信スレッドの優先度と CPU）と、全体の設定（`control_timeout_ms`、温度センサー故障のしきい値など）も同じファイルにある。各項目の説明はファイルのコメントにある。

- **MIT の範囲（spec）：** `config/specs/MOTOR_70.yaml` は、位置 ±12.5 rad、速度 ±45 rad/s、Kp 0〜500、Kd 0〜5、トルク ±80 N·m。モーター本体の設定（下記の `max_pos`・`max_vel`・`max_tor`・`kp_max`・`kd_max`）と一致している必要がある（範囲が違うと、指令と応答の値の換算がずれる）。
- **設定の検査：** `python3 scripts/validate_config.py <yaml>` で、書式（`config/bxi_hardware.schema.json`）に加えて、同じバスでの CAN ID の重複、関節の範囲や回転の向きの不正、機種や spec の不足を調べる。
- **正式な試験の記録：** ランナー（`tools/hil_stage*.sh`）は、使った設定ファイルの写しを記録先に残す。

### 1b. モーター本体の設定（ファームウェア）

モーター本体にも設定がある。**HIL のモーターは、2026-09-25 の確認の後、2026-09-27 の起動ログの前に交換した**（交換後も HW の表示は `Bxi_motor_70`、ファームウェアは同じ版）。今のモーターの値は、2026-09-30 に `scripts/show_bxi_motor_info.py` で読んだもの（記録はワークスペースの `log/bxi_motor_info_20260930.txt`）。

| 設定 | 今のモーター（2026-09-30） | 交換前のモーター（2026-09-24〜25） | 意味 |
|---|---|---|---|
| ファームウェア | code 0.2.2、config 0.2（BUILD Aug 19 2026。2026-09-27 の起動ログ） | 同じ | |
| `can_id` / `master_id` | 1 / 0x11 | 同じ | 指令の ID と応答の ID |
| `can_timeout` | 0 | 同じ | タイムアウト無効。**通信が止まっても最後の指令を保持し続ける** |
| `mit_mode` / `mit_aux_enable` | 1 / 0 | 同じ | |
| `max_pos` / `max_vel` / `max_tor` | 12.5 / 45 / 80 | 同じ | MIT の範囲（`MOTOR_70.yaml` と一致） |
| `kp_max` / `kd_max` | 500 / 5 | 同じ | 同上 |
| `reduction_ratio` / `motor_pole_pairs` | 19.545706 / 10 | 同じ | 減速比と極対数 |
| `current_limit` / `torquet_limit` | 48 / 48 | 同じ | |
| `protect_temperature_low` / `high` | 100 / 110 ℃ | 同じ | モーター側の温度保護 |
| `protect_under_voltage` / `over_voltage` | 12 / 72 V | 同じ | |
| `encoder_offset` / `enco_offset` | 36714 / 317.596313 | 37415 / 243.09（2026-09-24 のキャリブレーションの後） | キャリブレーションの結果（モーターごとに違う） |
| `usr_enc1_offset` / `usr_enc2_offset` | −19669 / −2.919834 | −5647 / 0.283631（2026-09-25 に `x` でゼロ点を設定し直した後） | ユーザーゼロ点の基準（モーターごとに違う） |
| `crc` | 0x5cc1e803 | 0x54ab600f | 設定の CRC |
| NTC2（温度センサー） | 正常（2026-09-27 の起動ログで 31.1 ℃、2026-09-30 の `p` で 31.0 ℃） | 故障（約 −28 ℃ を示した） | |

`bxi_hardware` が前提にしている設定（ID、`can_timeout`、MIT の範囲）は、交換の前後で同じだった。

**`can_timeout` は、モーターごとに違うことがある。** FRの3台（SN-0016／0014／0010、ID 1〜3）は、2026-10-21 に 200 ms に設定した（ワークスペース側の `docs/bxi/fr-leg-champ-jtc/reports/2026-10-21-can-timeout-c1-c6.md`）。ほかのモーターは 0 として扱い、`scripts/bxi_can_timeout.py read` で確かめる。意味は [`PROTOCOL_AND_SAFETY.md`](PROTOCOL_AND_SAFETY.md) の「モーター側の通信の期限」。

`bxi_hardware` には、モーター本体の設定を読む機能はない。ワークスペースの `scripts/` に、CAN で文字コマンドを使うスクリプトがある（[`scripts/README.md`](../../../scripts/README.md)）。**設定の一覧は、文字コマンド `s` で読む**（下記「文字コマンドで読む」）。

| 方法 | 内容 | 注意 |
|---|---|---|
| 文字コマンド `s`（Setup、CAN の `0x7E1`）：[`scripts/show_bxi_motor_info.py`](../../../scripts/show_bxi_motor_info.py) | 全設定の一覧（`[0] config_version` 〜 `[559] crc`。下記）。2026-09-24〜25 は bxi_usb2can 経由で `ESC s ESC` の順に送って読んだ | 送るとモーターが `07 SETUP` に入り、Motor Mode が解除される。**SETUP の間に送った文字は設定の書き込み（`項目 = 値`）として扱われるので、`s` の後は Esc だけを送る** |
| BXI_Tool（Windows、USB シリアル 115200 bps）の Config 画面 | すべての設定を一覧で読める | 読み書きはシリアルのコマンドで行われ、モーターの状態が `07 SETUP` → `01 MENU` に移り、Motor Mode が解除される |
| 電源を入れたときの起動ログ：[`scripts/capture_bxi_boot.py`](../../../scripts/capture_bxi_boot.py)（CAN、受信だけ） | BOOT INFO（ビルド、ファームウェアの版、起動時の CAN ID、HW、センサーの確認、エンコーダー）、状態、文字コマンドの一覧（下記）。**設定の一覧は出ない**（`s` で読む）。2026-09-27 に PCAN から記録した（`log/bxi_boot_20260927_*.log`） | 起動スクリプトを動かしてから、モーターの電源を入れる。何も送らない |
| CAN のレジスタ読み出し（`REG_READ`、ID `(0x17 << 4) + can_id` = `0x171`） | 2026-09-25 の調査スクリプト（`minibexkai_docs/research/logs/20260925_bxi_socketcan/scripts/can_motor.py reg-read <アドレス>`、python-can）で読める。`REG_INFO`（`0x1A1`）では公開レジスタ52、書き込めるもの46 | レジスタの表は `minibexkai_docs/reference/bxi/0.2.2 - BXI_Wiki.pdf`（2026-10-21 に入手。公開52・書ける46で REG_INFO と一致）。書き込み（`0x18`）と保存（`0x19`）は、[`scripts/bxi_can_timeout.py`](../../../scripts/bxi_can_timeout.py) で `can_timeout`（0x63）に 0 か 200 を書くときだけ使う（2026-10-21 に FR の3台で実施）。**ほかのレジスタは書かない**（値の範囲をモーターが確かめないため） |

### 文字コマンドで読む（`s` と `p`）

モーターには文字のコンソールがあり、CAN で使える。入力は `0x7E0 + can_id`（`0x7E1`）に ASCII を8バイトずつ送り、コマンドの文字には CR LF（`0D 0A`）を付ける。Esc は `1B` の1バイト。出力は `0x7F0 + can_id`（`0x7F1`）に ASCII で返る。

**`s` の出力（2026-09-30、今のモーター、`scripts/show_bxi_motor_info.py` で PCAN から読んだもの。抜粋）：**

```text
STATE : 07 SETUP
Configuration Options
 Reg map
  [0] config_version = 0x00000002 (read only)
  [1] motor_pole_pairs = 10
  [5] reduction_ratio = 19.545706
  [7] encoder_offset = 36714
  [16-143] offset_lut[128]
  [144-503] enco_offset_lut[360]
  [504] calib_valid = 1
  [508] control_mode = 1
  [514] pos_gain = 160.000000
  [518] current_limit = 48.000000
  [519] torquet_limit = 48.000000
  [523] protect_under_voltage = 12.000000
  [526] protect_temperature_low = 100.000000
  [527] protect_temperature_high = 110.000000
  [528] can_id = 1
  [529] can_timeout = 0
  [531] mit_mode = 1
  [533] max_pos = 12.500000
  [536] kp_max = 500.000000
  [537] kd_max = 5.000000
  [538] usr_enc1_offset = -19669
  [539] usr_enc2_offset = -2.919834
  [541] master_id = 0x11  (rewrite use 0d)
  [559] crc = 0x5cc1e803 (read only)
To change a value, type 'prefix' = 'value''ENTER'
```

（実際には `[0]`〜`[559]` のすべての項目が出る。全体はワークスペースの `log/bxi_motor_info_20260930.txt`。キャリブレーションやゼロ点の設定の後は `encoder_offset`・`usr_enc*_offset` などが変わる。）

**モーターを1台ずつ検査して記録を残すとき**は、[`scripts/inspect_bxi_motor.py`](../../../scripts/inspect_bxi_motor.py) を使う。電源を入れたときの起動ログから `s`・`p` の出力までを、`log/bxi_motor_inspection/<日付>/<識別ID>_<時刻>.txt` に残す（使い方は [`scripts/README.md`](../../../scripts/README.md)）。

**送り方：[`scripts/show_bxi_motor_info.py`](../../../scripts/show_bxi_motor_info.py)**（ホストで、launch が動いていない MENU の状態で）

```bash
cd ~/devel/minibexkai_ros2_ws
./scripts/show_bxi_motor_info.py --iface can0 --can-id 1 | tee motor_info.txt
```

- 送る順番は `p` → （状態が `01 MENU` のときだけ）`s` → Esc。`s` の後は Esc 以外を送らず、`s` の応答が読めなくても Esc は送る。
- `p` の応答が SYSTEM INFO でない、MENU でない、`s` で `07 SETUP` と設定の一覧が返らない、Esc で `01 MENU` に戻らない、CAN のエラーカウンタが増えた、のどれかで失敗（終了コード1）にする。
- `ros2_control_node` が動いていたら、何も送らずに終了コード2で止める（`p`・`s`・Esc は Motor Mode を解除するため）。
- CAN チャンネルは 1 Mbps・`ERROR-ACTIVE` であること（スクリプトが確かめる）。表示だけで、ファイルには残さない（`| tee` で残す）。
- 同じフォルダの [`find_bxi_can_id.py`](../../../scripts/find_bxi_can_id.py) は、`p` を ID 1〜15 に送ってモーターの CAN ID を調べる。[`bxi_single_joint_comm_test.sh`](../../../scripts/bxi_single_joint_comm_test.sh) は `p` と `bxi_probe`（ゲイン0）で1軸の通信を試験する（2026-09-27 に実機で実施）。
- 試験は `scripts/test_bxi_text_console_scripts.py`（vcan の模擬コンソール）。**2026-09-30 に実機（今のモーター、PCAN の `can0`）で使い、`p` と `s` の出力を読み、Esc で `01 MENU` に戻った。**

`cansend`・`candump` で同じことをするとき：

```bash
candump -L can0,7F1:7FF > /tmp/bxi_text.log &      # 文字出力を記録する
cansend can0 7E1#1B                                 # Esc：MENU に戻す
cansend can0 7E1#730D0A                             # 's' + CR LF（p は 700D0A）
sleep 2
cansend can0 7E1#1B                                 # Esc：MENU に戻す（SETUP のまま文字を送らない）
kill %1
```

- 実機の PCAN（SocketCAN）からは、`p` と Esc（2026-09-25・27）、受信だけの起動ログ（2026-09-27）、`s`（2026-09-30、`show_bxi_motor_info.py`）を確かめた。
- `p` は MENU の状態で送る（SETUP のままだと `Invalid state!get :70` になる）。

**文字コマンドの一覧（起動ログのメニュー）と扱い：**

| コマンド | 内容（メニューの表示） | 扱い |
|---|---|---|
| `p` | Print sys info | 読むだけ。Motor Mode は解除される |
| `s` | Setup（設定の一覧と変更） | 一覧を読むだけなら可。**後は Esc だけを送る** |
| Esc | Exit to Menu | MENU に戻る |
| `m` | Motor Mode | **使わない**（Motor Mode に入る。指令は ros2_control から送る） |
| `u` | Update Configs | **使わない**（フラッシュへの書き込みの可能性） |
| `z` | Clear Err | 使わない（効果を確かめていない） |
| `x` | Enc zero set | **使わない**（今の位置をゼロ点にしてフラッシュに保存する。設定全体も保存される） |
| `d` | Set default config | **使わない**（設定を既定に戻す） |
| `r` | reset sys | 使わない（モーターが再起動する） |
| `o`、`j`、`k`、`l`、`a`、`c`、`v` | Open Loop Mode、SLS、エンコーダーのキャリブレーション | **使わない**（モーターが回る・設定が変わる） |

## 2. ステータス

### 2a. モーターの状態（MENU / Motor Mode など）と内部の値

モーターは文字の出力（CAN ID `0x7F0 + can_id` = `0x7F1`、ASCII を8バイトずつ）に状態を出す。文字の入力は `0x7E0 + can_id` = `0x7E1`。

| 方法 | 分かること |
|---|---|
| 文字コマンド `p`（[`scripts/show_bxi_motor_info.py`](../../../scripts/show_bxi_motor_info.py)、上記） | SYSTEM INFO：状態（`STATE: 01 MENU` など）、電源電圧、FOC / PWM、温度（NTC1・NTC2・巻線・ドライバ）、エンコーダーの位置。2026-09-25 は `can_motor.py send p` で読んだ（49フレーム） |
| `bxi_probe --menu-only`（[`../hil/README.md`](../hil/README.md) の「1. 前提の確認」） | disable の応答と `STATE` の行（力を出さない） |
| 正式な試験の記録 | ランナーが `candump` から `STATE` の行を取り出して `state_lines.txt` に残し、起動時の `STATE : 02 MOTOR` と停止後の `STATE : 01 MENU` を判定する |

- 状態の例：`00 STARTUP`（起動中）、`01 MENU`（待機。力を出さない）、`02 MOTOR`（Motor Mode。MIT の指令で動く）、`07 SETUP`（設定の操作中）。
- MIT の応答は MENU の状態でも返る。**応答があるだけでは Motor Mode に入ったことは分からない。** 状態は `STATE` の行で確かめる。
- 2026-09-30 の SYSTEM INFO の例（今のモーター）：

  ```text
  ========== SYSTEM INFO ==========
  APP: NORMAL | STATE: 01 MENU
  BUS: 23.867 V
  FOC: DISARMED | PWM: OFF
  TEMP[C]: NTC1=32.2 NTC2=31.0 WND=30.0 DRV=43.5 coeff=1.00
  ENC1: raw=25975 pos=0.196
  ENC2: angle=-163.105 deg pos=0.012 turn
  TICK: 7217525
  =================================
  Invalid state!get :d
  Invalid state!get :a
  ```

  - 末尾の `Invalid state!get :d`・`:a` は、コマンドに付けた CR（0x0D）と LF（0x0A）をモーターが受け付けなかったという表示で、問題ない。
  - 交換前のモーターは NTC2 が故障していて、約 −28 ℃ を示し、`!!! NTC2 FAIL` と表示していた。HWIF は −20 ℃ を下回るセンサーを除外し、残りのセンサーで温度を監視する（[`PROTOCOL_AND_SAFETY.md`](PROTOCOL_AND_SAFETY.md) の「温度センサー故障」）。今のモーターは NTC1・NTC2 とも正常な値。

**注意：** 文字コマンドは、launch が動いていない（MENU の）ときに送る。1 kHz で指令を送っている間に文字コマンドを混ぜたときの動きは確かめていない。

### 2b. 位置・速度・トルク・温度（MIT の応答）

| 方法 | 内容 |
|---|---|
| `ros2 topic echo /joint_states --once`（launch の起動中） | 位置 [rad]・速度 [rad/s]・トルク（effort）[N·m] |
| `/dynamic_joint_states`（`control_msgs/msg/DynamicJointState`） | 温度も入る（`joint_state_broadcaster` の実装と設定から判断。実際には確かめていない） |
| 起動ログの `Enable joint1 pos=... temp=...` | 起動時の位置と温度（見方は [`../hil/SINGLE_JOINT_GUIDE.md`](../hil/SINGLE_JOINT_GUIDE.md) の「起動ログの見方」） |

### 2c. 通信と異常

| 方法 | 内容 |
|---|---|
| `ip -details -statistics link show can0` | バスの状態（`ERROR-ACTIVE` / `BUS-OFF`）、エラーカウンタ（TEC/REC）、bus-error・bus-off・drop の回数 |
| `candump -e 'can0,0~0,#FFFFFFFF'` | CAN のエラーフレームだけを表示 |
| HWIF のログ | 安全停止（`stopping all motors`、`送受信分離を停止` の行の `fault=`）、温度センサー故障の警告、過熱の停止、起動の中止（`起動を中止`） |

- モーター自身のエラーコードを読む仕組みは、手元の仕様にない（error frame の定義がないため、`bxi_hardware` は実装していない）。異常は、上のログと、文字出力の状態で判断する。

## 3. まだ確かめていないこと・扱わないこと

- モーター本体の設定を CAN から読む方法は、文字コマンド `s`（`scripts/show_bxi_motor_info.py`、2026-09-30 に実機で確認）。 CAN のレジスタ読み出しには、このファームウェアのアドレスの一覧が必要（入手の方法は未定）。
- 設定の書き込み・保存は、このパッケージでは扱わない（BXI_Tool で行う）。2026-09-25 の調査スクリプトは、ワークスペースの外（`minibexkai_docs/research/logs/20260925_bxi_socketcan/scripts/`、python-can の仮想環境）にある。
