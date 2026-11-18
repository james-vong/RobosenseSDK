#!/usr/bin/env python3
"""Runtime-only: re-program the USB2 PHY PLL (both PHYs) with a given R17 value using the vendor reset sequence."""
import mmap, os, struct, sys, time
r17 = int(sys.argv[1], 16); r16 = 0x09400414; r44 = 0x927e0000
fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
for b in (0xfe03c000, 0xfe03e000):
    m = mmap.mmap(fd, 0x1000, offset=b)
    w = lambda o, v: struct.pack_into("<I", m, o, v)
    w(0x40, 0x30000000 | r16); w(0x44, r44); w(0x48, r17); time.sleep(0.001); w(0x40, 0x10000000 | r16); time.sleep(0.005)
    print(hex(b), "R16=%08x R17=%08x lock=%d" % (struct.unpack_from("<I", m, 0x40)[0], struct.unpack_from("<I", m, 0x48)[0], struct.unpack_from("<I", m, 0x40)[0] >> 31))
