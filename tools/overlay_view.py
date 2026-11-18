"""Overlay demo: live AC1 camera image with the LiDAR points projected onto it as small dots (color = range).

Uses the factory-style calibration (tools/ac1_calib.py). Launches demo_ac1_stream.exe like live_view.py.
Keys: q / Esc quit, s snapshot, c toggle intrinsics scaling mode (squash <-> crop, pick the one that lines up),
      [ / ] dot size, p hide/show points, r cycle color (range / height / intensity).

Usage: overlay_view.py [--width 640 --height 480 --fps 30] [--mode squash|crop] [--dot 2] [--max-range 8]
                       [--seconds N] [--snapshot-on-exit] [--calib PATH] [--no-spawn] [--port 5577]
"""
import argparse
import datetime as dt
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ac1_calib import Ac1Calib  # noqa: E402
from live_view import ROOT, Stream, rate, valid_xyz  # noqa: E402

FONT = cv2.FONT_HERSHEY_SIMPLEX
COLOR_MODES = ("range", "height", "intensity")


def point_colors(xyz, inten, mode, max_range):
    if mode == "height":
        v = np.clip((xyz[:, 2] + 1.0) / 3.0, 0, 1)
    elif mode == "intensity":
        v = np.clip(inten / 255.0, 0, 1)
    else:
        v = 1.0 - np.clip(np.linalg.norm(xyz, axis=1) / max_range, 0, 1)   # red = near
    return cv2.applyColorMap((v * 255).astype(np.uint8), cv2.COLORMAP_TURBO).reshape(-1, 3)


def draw_points(img, uv, depth, colors, dot):
    """Splat dots (dot x dot px), far points first so near ones win."""
    h, w = img.shape[:2]
    order = np.argsort(-depth)
    px, py, col = uv[order, 0].astype(int), uv[order, 1].astype(int), colors[order]
    r = dot // 2
    for dy in range(-r, dot - r):
        for dx in range(-r, dot - r):
            qx, qy = np.clip(px + dx, 0, w - 1), np.clip(py + dy, 0, h - 1)
            img[qy, qx] = col


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--exe", default=str(ROOT / "rs_driver" / "build" / "demo" / "demo_ac1_stream.exe"))
    ap.add_argument("--port", type=int, default=5577)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--mode", choices=("squash", "crop"), default="squash")
    ap.add_argument("--dot", type=int, default=2)
    ap.add_argument("--max-range", type=float, default=8.0)
    ap.add_argument("--color", choices=COLOR_MODES, default="range")
    ap.add_argument("--calib", default=None)
    ap.add_argument("--seconds", type=float, default=0, help="auto-quit after N seconds (0 = run until q)")
    ap.add_argument("--snapshot-on-exit", action="store_true")
    ap.add_argument("--no-usb200", action="store_true")
    ap.add_argument("--no-spawn", action="store_true")
    args = ap.parse_args()

    calib = Ac1Calib(args.calib)
    print(calib.describe())
    mode, dot, show_pts, cmode = args.mode, max(1, args.dot), True, args.color

    proc = None
    if not args.no_spawn:
        cmd = [args.exe, "--port", str(args.port), "--width", str(args.width), "--height", str(args.height), "--fps", str(args.fps)]
        if not args.no_usb200:
            cmd.append("--usb200")
        print("launching:", " ".join(cmd))
        proc = subprocess.Popen(cmd, cwd=str(ROOT))
    stream = Stream(args.port)
    if not stream.connect():
        print("could not connect to the streamer", file=sys.stderr)
        if proc:
            proc.terminate()
        return 2

    win = "AC1 camera + LiDAR overlay  (q quit, s snapshot, c squash/crop, [ ] dot, p points, r color)"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    t0, canvas, last_img_id = time.time(), None, None
    try:
        while stream.alive:
            with stream.lock:
                pts = stream.cloud
                img = None if stream.image is None else stream.image.copy()
                f_cloud, f_img = rate(stream.t_cloud), rate(stream.t_image)
            if img is None:
                img = np.zeros((args.height, args.width, 3), np.uint8)
            h, w = img.shape[:2]
            n_drawn = 0
            xyz = valid_xyz(pts)
            if show_pts and xyz is not None and len(xyz):
                inten = pts[np.isfinite(pts[:, :3]).all(axis=1) & (np.abs(pts[:, :3]).sum(axis=1) > 0), 3]
                uv, inb, depth = calib.project(xyz, w, h, mode)
                if inb.any():
                    draw_points(img, uv[inb], depth[inb], point_colors(xyz[inb], inten[inb], cmode, args.max_range), dot)
                    n_drawn = int(inb.sum())
            hud = [f"{w}x{h} {f_img:4.1f} fps | clouds {f_cloud:4.1f} Hz | {n_drawn} pts in view | mode={mode} dot={dot} color={cmode}",
                   f"calib: {calib.path.name} (device {calib.device_id})"]
            for i, line in enumerate(hud):
                cv2.putText(img, line, (6, 18 + 18 * i), FONT, 0.45, (0, 0, 0), 3)
                cv2.putText(img, line, (6, 18 + 18 * i), FONT, 0.45, (255, 255, 255), 1)
            canvas = img
            cv2.imshow(win, canvas)
            key = cv2.waitKey(20) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                out = ROOT / "captures" / ("overlay_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S") + ".png")
                out.parent.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(out), canvas)
                print("snapshot ->", out)
            if key == ord("c"):
                mode = "crop" if mode == "squash" else "squash"
            if key == ord("["):
                dot = max(1, dot - 1)
            if key == ord("]"):
                dot = min(9, dot + 1)
            if key == ord("p"):
                show_pts = not show_pts
            if key == ord("r"):
                cmode = COLOR_MODES[(COLOR_MODES.index(cmode) + 1) % len(COLOR_MODES)]
            if args.seconds and time.time() - t0 > args.seconds:
                break
            try:
                if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:
                break
        if stream.error:
            print("stream ended:", stream.error)
    finally:
        if args.snapshot_on_exit and canvas is not None:
            out = ROOT / "captures" / ("overlay_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S") + ".png")
            out.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(out), canvas)
            print("snapshot ->", out)
        stream.close()
        cv2.destroyAllWindows()
        if proc:
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                proc.terminate()
    return 0


if __name__ == "__main__":
    sys.exit(main())
