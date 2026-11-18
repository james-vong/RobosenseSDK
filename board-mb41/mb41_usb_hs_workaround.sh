#!/usr/bin/env bash
# MB41 (Amlogic SC2, kernel 6.19.6-rwsdev): make USB 2.0 HIGH SPEED work on the first host port (usb1-port1).
#
# Symptom this works around: any high-speed device (e.g. RoboSense AC1, 3840:1010) connects at full speed, then at
# port reset the xHCI port reports a disconnect and the kernel logs
#     usb usb1-port1: Cannot enable. Maybe the USB cable is bad?   ... unable to enumerate USB device
#
# What it does: replays, in order, the runtime USB2-PHY / dwc3 register writes that were found (2026-11-18) to
# leave the port working, then puts every register back to its boot value. Nothing is written to storage; the
# effect lasts until the next reboot, so run it once after every boot (takes ~40 s). Run as root (sudo).
#
# Status: empirical. The sequence was verified twice on one board (2026-11-18): after a clean reboot the AC1 fails;
# after the sequence it enumerates at 480 Mb/s with no physical change. THIS single-file packaging of it has NOT
# been run yet; the exact files that were run are in validated_original/ (replay_all.sh + three helpers, also in
# /home/debian on reworld-mb41-ac1-1). The registers read identical before and after (only the PLL lock status
# bits R16[30] / R19[30] differ), a PLL re-lock alone does NOT fix it, and each step alone does NOT fix it, so the
# minimal step is still unknown. The second port (usb1-port2) still fails afterwards.
# The proper fix is in the kernel's SC2 USB2 PHY init; this is only a stop-gap.
#
# Register map: USB2 PHYs at 0xfe03c000 and 0xfe03e000 (both are written), dwc3 GUSB2PHYCFG(0/1) at 0xfde0c200/204.
set -euo pipefail
[ "$(id -u)" = "0" ] || { echo "run as root: sudo $0"; exit 1; }

PY=/tmp/mb41phy.py
cat > "$PY" <<'EOF'
import mmap, os, struct, sys, time
fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
PHYS = (0xfe03c000, 0xfe03e000)
def rd(m, o): return struct.unpack_from("<I", m, o)[0]
cmd = sys.argv[1]
if cmd == "w":                                   # w <offset_hex> <value_hex> : write one PHY register on both PHYs
    o, v = int(sys.argv[2], 16), int(sys.argv[3], 16)
    maps = [mmap.mmap(fd, 0x1000, offset=b) for b in PHYS]
    for m in maps: struct.pack_into("<I", m, o, v)
    for m in maps: [rd(m, r) for r in (0x0c, 0x10, 0x34, 0x38, 0x40, 0x48, 0x50, 0x54)]
elif cmd == "pll":                               # pll <r17_hex> : PLL reset + lock sequence on both PHYs
    r17 = int(sys.argv[2], 16)
    for b in PHYS:
        m = mmap.mmap(fd, 0x1000, offset=b)
        struct.pack_into("<I", m, 0x40, 0x30000000 | 0x09400414); struct.pack_into("<I", m, 0x44, 0x927e0000)
        struct.pack_into("<I", m, 0x48, r17); time.sleep(0.001)
        struct.pack_into("<I", m, 0x40, 0x10000000 | 0x09400414); time.sleep(0.005)
        rd(m, 0x40); rd(m, 0x48)
elif cmd == "g":                                 # g <value_hex> : dwc3 GUSB2PHYCFG(0) and (1)
    m = mmap.mmap(fd, 0x1000, offset=0xfde0c000)
    for o in (0x200, 0x204):
        struct.pack_into("<I", m, o, int(sys.argv[2], 16)); rd(m, o)
elif cmd == "show":
    for b in PHYS:
        m = mmap.mmap(fd, 0x1000, offset=b)
        print(hex(b), " ".join("%02x:%08x" % (o, rd(m, o)) for o in (0x0c, 0x10, 0x34, 0x38, 0x40, 0x48, 0x4c, 0x50, 0x54)))
    m = mmap.mmap(fd, 0x1000, offset=0xfde0c000); print("GUSB2PHYCFG %08x %08x" % (rd(m, 0x200), rd(m, 0x204)))
EOF

w()   { while [ $# -ge 2 ]; do python3 "$PY" w "$1" "$2"; shift 2; done; sleep 0.3; }
pll() { python3 "$PY" pll "$1"; python3 "$PY" pll "$1"; }
pll1(){ python3 "$PY" pll "$1"; }
g()   { python3 "$PY" g "$1"; }
B="0c 00000034 10 080f0f0f 34 00070000 38 00000000 54 00000022"     # boot values of the tuning registers

echo "before:"; python3 "$PY" show
w 0c 000000f4
w $B 54 0000002a
w $B 38 000e000c 34 00078000
w $B 54 0000002a 38 000e000c 34 00078000
w $B 10 08000fff
w $B 10 08000fff 54 0000002a 38 000e000c 34 00078000
w 0c 00000034 10 080f0f0f 34 00070000 54 00000022 38 000e000c 34 00078000
w $B
pll ac5f69e5; w $B
pll ac5f69e5; w $B 54 0000002a
pll1 ac5f49e5; w 54 00000022; w 38 00000000; w 10 080f0f0f; w 0c 00000034
pll ac5f69e5; w 50 0000fe18 54 0000002a 10 0000003f 38 00000000 0c 0000003c 34 00078000
pll ac5f69e5; w 50 0000fe18 54 00000022 10 80000fff 38 00000000 0c 0000003c 34 00078000
pll ac5f69e5; w 50 0000fe18 54 0000002a 10 080f0f3f 38 00000000 0c 0000003c 34 00078000
g 40101508; pll ac5f49e5; w $B
pll ac5f69e5; w 50 0000fe18 54 0000002a 10 0000003f 38 00000000 0c 0000003c 34 00078000
# restore every register to its boot value
g 40102500; pll1 ac5f49e5; w 0c 00000034; w 10 080f0f0f; w 34 00070000; w 38 00000000; w 54 00000022; w 50 0000fe18
echo "after (boot values restored):"; python3 "$PY" show

# a device that already failed stays in the failed state until the port sees a new connect: re-arm port 1
P=/sys/bus/usb/devices/usb1/1-0:1.0/usb1-port1
if [ -e "$P/disable" ]; then echo 1 > "$P/disable"; sleep 1; echo 0 > "$P/disable"; sleep 10; fi
echo "devices on the bus now:"; lsusb | grep -v "1d6b:" || echo "  (none; plug the device into the first USB port)"
