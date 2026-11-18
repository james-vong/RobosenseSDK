#!/usr/bin/env bash
# Force a fresh enumeration on usb1-port1 (disable/enable) and report. Usage: p1try.sh <label>
P=/sys/bus/usb/devices/usb1/1-0:1.0/usb1-port1; T=$(date +%H:%M:%S)
echo 1 | sudo tee $P/disable >/dev/null; sleep 1; echo 0 | sudo tee $P/disable >/dev/null; sleep 11
J=$(sudo journalctl -k -b --no-pager --since "$T")
r="FAIL"; echo "$J" | grep -q "usb 1-1: new high-speed" && r="WORKS (high speed)"
echo "$1 -> $r  [cannot-enable=$(echo "$J" | grep -c "Cannot enable"), lsusb AC1=$(lsusb | grep -ci 3840)]"
[ "$r" != "FAIL" ]
