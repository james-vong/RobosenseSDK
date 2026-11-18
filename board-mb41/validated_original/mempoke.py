#!/usr/bin/env python3
"""Runtime-only 32-bit peek/poke: mempoke.py <addr_hex> [value_hex]"""
import mmap, os, struct, sys
a = int(sys.argv[1], 16); pg = a & ~0xfff
fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC); m = mmap.mmap(fd, 0x1000, offset=pg)
if len(sys.argv) > 2: struct.pack_into("<I", m, a - pg, int(sys.argv[2], 16))
print("%08x: %08x" % (a, struct.unpack_from("<I", m, a - pg)[0]))
