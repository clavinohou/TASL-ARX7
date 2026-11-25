#!/usr/bin/env bash
# Brings up the robot's three serial CAN adapters (1 Mbit/s, slcan) after a boot.
# Run on the robot:  bash ~/can-up.sh
# Creating the interfaces does not move any motor.
#   can0 -> left arm, can1 -> right arm, can5 -> base, lift, head
set -euo pipefail

sudo modprobe slcan
for n in 0 1 5; do
    iface="can$n" dev="/dev/arxcan$n"
    if [[ ! -e "$dev" ]]; then
        printf '%s: %s is missing. Check the USB CAN adapter and the udev rule.\n' "$iface" "$dev" >&2
        continue
    fi
    if ! ip link show "$iface" >/dev/null 2>&1; then
        sudo slcand -o -f -s8 "$dev" "$iface"
        for _ in $(seq 20); do ip link show "$iface" >/dev/null 2>&1 && break; sleep 0.1; done
    fi
    sudo ip link set dev "$iface" up
done
ip -brief link show type can
