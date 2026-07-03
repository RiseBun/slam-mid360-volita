#!/usr/bin/env bash
set -euo pipefail

iface="${1:-enP8p1s0}"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
config="${script_dir}/../config/ptp4l_software_master.conf"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run with sudo: sudo $0 ${iface}" >&2
  exit 1
fi

if ! command -v ptp4l >/dev/null 2>&1; then
  echo "ptp4l is missing. Install it with: sudo apt install linuxptp" >&2
  exit 1
fi

ip link set "${iface}" up
echo "Starting software PTP master on ${iface}"
echo "Keep this terminal open while DP180-Pro is syncing."
exec ptp4l -2 -i "${iface}" -S -m --step_threshold=1 -f "${config}"
