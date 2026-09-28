#!/usr/bin/env bash
# Run after creating and enabling an isolated vcan0 interface.
set -eo pipefail

reply_ids="${BXI_VCAN_REPLY_IDS:-1}"
if [[ "${BXI_VCAN_THREE_REPLIES:-0}" == "1" && -z "${BXI_VCAN_REPLY_IDS:-}" ]]; then
  reply_ids="1,2,3"
fi
python3 /ws/src/bxi_hardware/test/manual_probe_stream_vcan.py --iface vcan0 --seconds 5 \
  --broadcast-replies "${BXI_EMULATOR_REPLIES:--1}" \
  --broadcast-reply-ids "$reply_ids" \
  --expect-broadcast-bytes "$((8 * ${BXI_PROBE_BLOCKS:-1}))" &
emulator_pid=$!
trap 'wait "$emulator_pid" || true' EXIT
sleep 0.2
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
set -u
extra_args=()
if [[ "${BXI_PROBE_HARDWARE_TIMESTAMPS:-0}" == "1" ]]; then
  extra_args+=(--hardware-timestamps)
fi
if [[ "${BXI_VCAN_THREE_REPLIES:-0}" == "1" ]]; then
  extra_args+=(--vcan-three-replies)
fi
ros2 run bxi_hardware bxi_probe --iface vcan0 --can-id 1 \
  --format "${BXI_PROBE_FORMAT:-fd_brs}" \
  --broadcast --menu-only --stream --broadcast-blocks "${BXI_PROBE_BLOCKS:-1}" \
  --count "${BXI_PROBE_COUNT:-20}" \
  --rate "${BXI_PROBE_RATE:-200}" "${extra_args[@]}"
wait "$emulator_pid"
trap - EXIT
