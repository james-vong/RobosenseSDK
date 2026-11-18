"""Convert raw image frames dumped by demo_ac1_dump (image_NNN_WxH_fmt.raw) to PNG.

Usage: raw_to_png.py <capture_dir_or_raw_file> [...]
"""
import re
import sys
from pathlib import Path

import cv2
import numpy as np

NAME_RE = re.compile(r"image_(\d+)_(\d+)x(\d+)_(nv12|bgr24|rgb24|yuyv|any)\.raw$")


def convert(raw: Path) -> Path | None:
    m = NAME_RE.search(raw.name)
    if not m:
        print(f"skip {raw.name}: unrecognised name")
        return None
    w, h, fmt = int(m.group(2)), int(m.group(3)), m.group(4)
    buf = np.fromfile(raw, dtype=np.uint8)
    expect = {"nv12": w * h * 3 // 2, "bgr24": w * h * 3, "rgb24": w * h * 3, "yuyv": w * h * 2}.get(fmt)
    if expect is not None and buf.size != expect:
        print(f"skip {raw.name}: {buf.size} bytes, expected {expect} for {fmt} {w}x{h}")
        return None
    if fmt == "nv12":
        bgr = cv2.cvtColor(buf.reshape(h * 3 // 2, w), cv2.COLOR_YUV2BGR_NV12)
    elif fmt == "bgr24":
        bgr = buf.reshape(h, w, 3)
    elif fmt == "rgb24":
        bgr = cv2.cvtColor(buf.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
    elif fmt == "yuyv":
        bgr = cv2.cvtColor(buf.reshape(h, w, 2), cv2.COLOR_YUV2BGR_YUYV)
    else:
        print(f"skip {raw.name}: unknown format")
        return None
    out = raw.with_suffix(".png")
    cv2.imwrite(str(out), bgr)
    print(f"wrote {out}  mean BGR={bgr.reshape(-1, 3).mean(axis=0).round(1).tolist()}")
    return out


def main(args):
    outs = []
    for a in args:
        p = Path(a)
        files = sorted(p.glob("image_*.raw")) if p.is_dir() else [p]
        for f in files:
            o = convert(f)
            if o:
                outs.append(o)
    if not outs:
        print("no images converted")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
