#!/usr/bin/env bash
# Replays, in the original order, every runtime PHY/controller change made on 2026-11-18 before the AC1 first enumerated.
w() { while [ $# -ge 2 ]; do sudo python3 ~/phyreg.py w $1 $2 >/dev/null; shift 2; done; sleep 0.3; }
pll() { sudo python3 ~/phypll.py $1 >/dev/null; sudo python3 ~/phypll.py $1 >/dev/null; }
g() { sudo python3 ~/mempoke.py fde0c200 $1 >/dev/null; sudo python3 ~/mempoke.py fde0c204 $1 >/dev/null; }
B="0c 00000034 10 080f0f0f 34 00070000 38 00000000 54 00000022"
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
sudo python3 ~/phypll.py ac5f49e5 >/dev/null; w 54 00000022; w 38 00000000; w 10 080f0f0f; w 0c 00000034
pll ac5f69e5; w 50 0000fe18 54 0000002a 10 0000003f 38 00000000 0c 0000003c 34 00078000
pll ac5f69e5; w 50 0000fe18 54 00000022 10 80000fff 38 00000000 0c 0000003c 34 00078000
pll ac5f69e5; w 50 0000fe18 54 0000002a 10 080f0f3f 38 00000000 0c 0000003c 34 00078000
g 40101508; pll ac5f49e5; w $B
pll ac5f69e5; w 50 0000fe18 54 0000002a 10 0000003f 38 00000000 0c 0000003c 34 00078000
g 40102500; sudo python3 ~/phypll.py ac5f49e5 >/dev/null; w 0c 00000034; w 10 080f0f0f; w 34 00070000; w 38 00000000; w 54 00000022; w 50 0000fe18
