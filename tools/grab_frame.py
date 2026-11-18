"""Grab a few full-grid AC1 point clouds (NaNs kept) and one camera image via demo_ac1_stream, then
analyze the grid layout / coordinate frame and write a camera-vs-range-image comparison PNG.

Usage: grab_frame.py [out_dir] [--width W --height H --fps F]
"""
import subprocess
import sys
import time
import warnings
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from live_view import ROOT, Stream, front_view, load_settings  # noqa: E402


def nanmean_quiet(a, axis):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        return np.nanmean(a, axis=axis)


def describe(name, v):
    f = v[np.isfinite(v)]
    if len(f) < 2:
        print(f"  {name}: no data")
        return
    print(f"  {name}: first={f[0]:7.2f}  last={f[-1]:7.2f}  min={f.min():7.2f}  max={f.max():7.2f}  (n={len(f)})")


def main(argv):
    out = ROOT / "captures" / "grab"
    width, height, fps = 640, 480, 30
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--width":
            width = int(argv[i + 1]); i += 2
        elif a == "--height":
            height = int(argv[i + 1]); i += 2
        elif a == "--fps":
            fps = int(argv[i + 1]); i += 2
        else:
            out = Path(a); i += 1
    out.mkdir(parents=True, exist_ok=True)

    exe = ROOT / "rs_driver" / "build" / "demo" / "demo_ac1_stream.exe"
    proc = subprocess.Popen([str(exe), "--usb200", "--width", str(width), "--height", str(height), "--fps", str(fps)],
                            cwd=str(ROOT), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    s = Stream(5577)
    if not s.connect():
        proc.terminate()
        print("could not connect")
        return 2
    t0 = time.time()
    clouds, img = [], None
    while time.time() - t0 < 20 and (len(clouds) < 3 or img is None):
        time.sleep(0.05)
        with s.lock:
            if s.cloud is not None and (not clouds or s.cloud_ts != clouds[-1][0]):
                clouds.append((s.cloud_ts, s.cloud.copy(), s.grid))
            if s.image is not None and img is None:
                img = s.image.copy()
    s.close()
    try:
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        proc.terminate()
    print(f"got {len(clouds)} clouds, image={'yes' if img is not None else 'no'}")
    if not clouds:
        return 3
    for k, (ts, pts, grid) in enumerate(clouds):
        np.save(out / f"cloud_{k:03d}_grid{grid[1]}x{grid[0]}.npy", pts)
    if img is not None:
        cv2.imwrite(str(out / "image.png"), img)

    ts, pts, (gw, gh) = clouds[0]
    xyz = pts[:, :3].reshape(gh, gw, 3)
    r = np.linalg.norm(xyz, axis=2)
    valid = np.isfinite(r) & (r > 0.01)
    az = np.where(valid, np.degrees(np.arctan2(xyz[:, :, 1], xyz[:, :, 0])), np.nan)
    el = np.where(valid, np.degrees(np.arctan2(xyz[:, :, 2], np.hypot(xyz[:, :, 0], xyz[:, :, 1]))), np.nan)
    print(f"grid rows={gh} cols={gw}  valid={int(valid.sum())}/{gh * gw}")
    print("per-row means (index = row 0..%d):" % (gh - 1))
    describe("azimuth  ", nanmean_quiet(az, axis=1))
    describe("elevation", nanmean_quiet(el, axis=1))
    print("per-col means (index = col 0..%d):" % (gw - 1))
    describe("azimuth  ", nanmean_quiet(az, axis=0))
    describe("elevation", nanmean_quiet(el, axis=0))
    st = load_settings()
    ops = [k for k in ("y_right", "z_down") if st.get(k)]

    # coordinate-frame sanity: planar peaks per axis (floor/ceiling should show up on the 'up' axis)
    v = pts[np.isfinite(pts[:, :3]).all(axis=1) & (np.abs(pts[:, :3]).sum(axis=1) > 0), :3]
    print("axis ranges: x %.2f..%.2f  y %.2f..%.2f  z %.2f..%.2f" % (v[:, 0].min(), v[:, 0].max(), v[:, 1].min(), v[:, 1].max(), v[:, 2].min(), v[:, 2].max()))
    for name, col in (("x", 0), ("y", 1), ("z", 2)):
        h, edges = np.histogram(v[:, col], bins=np.arange(-6, 10, 0.05))
        top = np.argsort(h)[::-1][:3]
        peaks = ", ".join(f"{edges[t]:+.2f}m:{100 * h[t] / len(v):.1f}%" for t in top)
        print(f"  {name} top 5cm-bin peaks: {peaks}")

    ri, nvalid = front_view(pts, 8.0, st.get("y_right", False), st.get("z_down", False))
    ri = cv2.resize(ri, (ri.shape[1] * 3, ri.shape[0] * 3), interpolation=cv2.INTER_NEAREST)
    cv2.imwrite(str(out / "front_view.png"), ri)
    if img is not None:
        hh = 360
        cam = cv2.resize(img, (int(img.shape[1] * hh / img.shape[0]), hh))
        rr = cv2.resize(ri, (int(ri.shape[1] * hh / ri.shape[0]), hh), interpolation=cv2.INTER_NEAREST)
        canvas = np.zeros((hh + 30, cam.shape[1] + rr.shape[1] + 10, 3), np.uint8)
        canvas[30:, :cam.shape[1]] = cam
        canvas[30:, cam.shape[1] + 10:] = rr
        cv2.putText(canvas, "camera", (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(canvas, f"LiDAR front view, az/el projection ({'+'.join(ops) or 'default: +y left, +z up'})", (cam.shape[1] + 16, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.imwrite(str(out / "compare.png"), canvas)
        print("wrote", out / "compare.png")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
