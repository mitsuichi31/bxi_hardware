# HilTrajectoryController の ~/status（約 50 Hz、RT ではないタイマーが配信する）
uint8 IDLE=0
uint8 ARMED=1       # 開始要求を適用し、要求番号の一致する keepalive を待っている（出力は力を抜いたまま）
uint8 RUNNING=2
uint8 HOLDING=3     # 軌道と hold が終わり、release なしで保持している
uint8 RELEASED=4    # release・finish・stop で力を抜いた（正常な終わり方は release と finish だけ）
uint8 ABORTED=5     # 中止して力を抜いた
uint8 state
uint32 applied_request_id   # 適用した開始要求の番号（0：なし）
uint32 applied_stop_count   # 適用した ~/stop の数
string end_reason           # release / finish / stop / 中止の理由
float64 elapsed             # 軌道の経過時間 [s]
float64 target_position
float64 position
uint8 RECORD_NONE=0
uint8 RECORD_RECORDING=1
uint8 RECORD_WRITTEN=2
uint8 RECORD_FAILED=3
uint8 record_state
string csv_path
uint64 recorded_cycles
uint64 dropped_cycles
uint64 write_failures        # command interface への書き込みに失敗した周期数（1周期1回）
bool release_write_pending   # この実行の脱力値が command interface に一度もすべて反映されていない
