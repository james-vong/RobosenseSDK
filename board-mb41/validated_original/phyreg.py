#!/usr/bin/env python3
"""Runtime-only peek/poke of the Amlogic USB2 PHY registers via /dev/mem (lost on reboot).
usage: phyreg.py r | phyreg.py w <offset_hex> <value_hex>   (applies to both PHYs fe03c000, fe03e000)"""
import mmap, os, struct, sys
BASES = (0xfe03c000, 0xfe03e000)
fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
maps = [mmap.mmap(fd, 0x1000, offset=b) for b in BASES]
def rd(m, o): return struct.unpack_from("<I", m, o)[0]
if sys.argv[1] == "w":
    o, v = int(sys.argv[2], 16), int(sys.argv[3], 16)
    for m in maps: struct.pack_into("<I", m, o, v)
for b, m in zip(BASES, maps):
    print(hex(b), " ".join(f"{o:02x}:{rd(m,o):08x}" for o in (0x0c, 0x10, 0x34, 0x38, 0x40, 0x48, 0x50, 0x54)))
