"""Numeric sanity check of a raw NV12 frame: per-plane stats, plus a luma-only PNG.

Usage: nv12_stats.py <image_NNN_WxH_nv12.raw> [...]
"""
import re
import sys
from pathlib import Path

import cv2
import numpy as np

NAME_RE = re.compile(r"image_(\d+)_(\d+)x(\d+)_nv12\.raw$")


def stats(name, a):
    a = a.astype(np.float32)
    print(f"  {name:6s} mean={a.mean():7.2f} std={a.std():6.2f} min={a.min():3.0f} max={a.max():3.0f} "
          f"zeros={(a == 0).mean() * 100:5.1f}%  unique={len(np.unique(a))}")


def main(argv):
    rc = 0
    for p in map(Path, argv):
        m = NAME_RE.search(p.name)
        if not m:
            print(f"skip {p.name}")
            continue
        w, h = int(m.group(2)), int(m.group(3))
        buf = np.fromfile(p, dtype=np.uint8)
        print(f"{p.name}: {buf.size} bytes (expect {w * h * 3 // 2})")
        y = buf[: w * h].reshape(h, w)
        uv = buf[w * h:].reshape(h // 2, w)
        stats("Y", y)
        stats("U", uv[:, 0::2])
        stats("V", uv[:, 1::2])
        # row-wise luma variation tells us whether Y is an actual image or a flat/striped buffer
        print(f"  Y row-mean spread: {y.mean(axis=1).min():.1f}..{y.mean(axis=1).max():.1f}; "
              f"col-mean spread: {y.mean(axis=0).min():.1f}..{y.mean(axis=0).max():.1f}")
        out = p.with_name(p.stem + "_Y.png")
        cv2.imwrite(str(out), y)
        print(f"  wrote luma-only {out.name}")
        if y.std() < 2:
            rc = 1
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
