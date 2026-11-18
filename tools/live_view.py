"""Live viewer for the RoboSense AC1: camera image + LiDAR front view + top-down view + IMU.

Launches demo_ac1_stream.exe (rs_driver dev_opt_AC1), connects to its localhost socket and
renders in an OpenCV window.

Sensor frame (verified 2026-10-07 from data + IMU gravity): x forward, y left, z up.
The LiDAR front view is an angular projection (azimuth = atan2(y, x) horizontal, elevation vertical),
NOT the raw 288x96 grid, because the AC1 grid is not a plain raster.

Range coloring is logarithmic from 0.2 m to --max-range (default 50 m) with a legend, so far returns
stay distinguishable; points beyond --far (default 15 m) are drawn bigger with a white ring.

Keys: q / Esc quit, s snapshot, h mirror left/right, v flip front view, - / = zoom the top-down map,
      k top-down color (range / height), l log / linear range scale. Settings persist in live_view_settings.json.

Usage: live_view.py [--width 640 --height 480 --fps 30] [--no-image] [--max-range 50] [--extent 30] [--far 15]
                    [--seconds N] [--snapshot-on-exit] [--port 5577] [--no-spawn] [--exe PATH]
"""
import argparse
import collections
import datetime as dt
import json
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
SETTINGS = ROOT / "live_view_settings.json"
HDR = struct.Struct("<4sB3xId")  # magic, type, pad, payload_len, timestamp (20 bytes)
FMT_NAMES = {0: "any", 1: "nv12", 2: "bgr24", 3: "rgb24", 4: "yuv422"}
FONT = cv2.FONT_HERSHEY_SIMPLEX
RANGE_MIN = 0.2          # AC1 blind zone / decoder minimum
STALE_SECONDS = 86400    # cloud timestamps this far from the wall clock are stale device buffers (seen: 2015 dates)
EXTENTS = [8, 15, 30, 60]
DEFAULT_SETTINGS = {"y_right": False, "z_down": False, "extent": 30, "td_color": "range", "log_scale": True}


class Stream:
    def __init__(self, port):
        self.port = port
        self.sock = None
        self.cloud = None       # (n,4) float32  x y z intensity, full grid order, NaN = invalid
        self.grid = (96, 288)
        self.cloud_ts = 0.0
        self.image = None       # BGR np.uint8
        self.image_meta = ""
        self.imu = collections.deque(maxlen=200)
        self.t_cloud = collections.deque(maxlen=30)
        self.t_image = collections.deque(maxlen=60)
        self.t_imu = collections.deque(maxlen=400)
        self.n_cloud = self.n_image = self.n_imu = self.n_stale = 0   # monotonic counters
        self.lock = threading.Lock()
        self.alive = False
        self.error = ""

    def connect(self, timeout_s=20.0):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            try:
                s = socket.create_connection(("127.0.0.1", self.port), timeout=2.0)
                s.settimeout(5.0)
                self.sock = s
                self.alive = True
                threading.Thread(target=self._reader, daemon=True).start()
                return True
            except OSError:
                time.sleep(0.3)
        return False

    def _recv_exact(self, n):
        buf = bytearray(n)
        view = memoryview(buf)
        got = 0
        while got < n:
            k = self.sock.recv_into(view[got:], n - got)
            if k <= 0:
                raise ConnectionError("socket closed")
            got += k
        return buf

    def _reader(self):
        try:
            while self.alive:
                magic, typ, plen, ts = HDR.unpack(self._recv_exact(HDR.size))
                if magic != b"AC1F":
                    raise ValueError(f"bad magic {magic!r}")
                payload = self._recv_exact(plen)
                now = time.time()
                if typ == 1:
                    if abs(ts - now) > STALE_SECONDS:      # stale buffer the device emits right after start
                        with self.lock:
                            self.n_stale += 1
                        continue
                    n, gw, gh = struct.unpack_from("<III", payload, 0)
                    pts = np.frombuffer(payload, dtype=np.float32, count=n * 4, offset=12).reshape(n, 4)
                    with self.lock:
                        self.cloud, self.grid, self.cloud_ts = pts.copy(), (gw, gh), ts
                        self.t_cloud.append(now)
                        self.n_cloud += 1
                elif typ == 2:
                    w, h, fmt, nb = struct.unpack_from("<IIII", payload, 0)
                    raw = np.frombuffer(payload, dtype=np.uint8, count=nb, offset=16)
                    bgr = decode_image(raw, w, h, fmt)
                    with self.lock:
                        if bgr is not None:
                            self.image = bgr
                        self.image_meta = f"{w}x{h} {FMT_NAMES.get(fmt, fmt)}"
                        self.t_image.append(now)
                        self.n_image += 1
                elif typ == 3:
                    vals = struct.unpack_from("<10f", payload, 0)
                    with self.lock:
                        self.imu.append((ts,) + vals)
                        self.t_imu.append(now)
                        self.n_imu += 1
        except Exception as e:  # noqa: BLE001
            self.error = str(e)
        finally:
            self.alive = False

    def close(self):
        self.alive = False
        try:
            if self.sock:
                self.sock.close()
        except OSError:
            pass


def decode_image(raw, w, h, fmt):
    try:
        if fmt == 1 and raw.size == w * h * 3 // 2:
            return cv2.cvtColor(raw.reshape(h * 3 // 2, w), cv2.COLOR_YUV2BGR_NV12)
        if fmt == 2 and raw.size == w * h * 3:
            return raw.reshape(h, w, 3).copy()
        if fmt == 3 and raw.size == w * h * 3:
            return cv2.cvtColor(raw.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
        if fmt == 4 and raw.size == w * h * 2:
            return cv2.cvtColor(raw.reshape(h, w, 2), cv2.COLOR_YUV2BGR_YUYV)
    except cv2.error:
        return None
    return None


def rate(ts):
    if len(ts) < 2:
        return 0.0
    span = ts[-1] - ts[0]
    return (len(ts) - 1) / span if span > 0 else 0.0


def valid_xyz(pts):
    if pts is None:
        return None
    xyz = pts[:, :3]
    ok = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).sum(axis=1) > 0)
    return xyz[ok]


# ---------------------------------------------------------------- range coloring
def range_to_unit(r, rmax, log_scale=True):
    r = np.asarray(r, float)
    if log_scale:
        v = np.log(np.maximum(r, RANGE_MIN) / RANGE_MIN) / np.log(rmax / RANGE_MIN)
    else:
        v = r / rmax
    return np.clip(v, 0, 1)


def range_colors(r, rmax, log_scale=True):
    """BGR uint8 (n,3): red = near, blue = far, on a log (default) or linear scale up to rmax."""
    v = range_to_unit(r, rmax, log_scale)
    return cv2.applyColorMap(((1.0 - v) * 255).astype(np.uint8), cv2.COLORMAP_TURBO).reshape(-1, 3)


def draw_range_legend(img, rmax, log_scale=True, x=None, y=None, w=240, h=10):
    H, W = img.shape[:2]
    x = W - w - 12 if x is None else x
    y = H - h - 20 if y is None else y
    rs = np.exp(np.linspace(np.log(RANGE_MIN), np.log(rmax), w)) if log_scale else np.linspace(0, rmax, w)
    img[y:y + h, x:x + w] = np.repeat(range_colors(rs, rmax, log_scale).reshape(1, w, 3), h, axis=0)
    for t in (0.5, 1, 2, 5, 10, 20, 50, 100):
        if RANGE_MIN < t <= rmax:
            px = int(x + range_to_unit(t, rmax, log_scale) * (w - 1))
            cv2.line(img, (px, y + h), (px, y + h + 4), (255, 255, 255), 1)
            cv2.putText(img, f"{t:g}", (px - 7, y + h + 15), FONT, 0.35, (255, 255, 255), 1)
    cv2.putText(img, f"range [m] {'log' if log_scale else 'linear'} scale, max {rmax:g}", (x, y - 4), FONT, 0.38, (200, 200, 200), 1)


def splat(img, px, py, colors, size):
    h, w = img.shape[:2]
    r = size // 2
    for dy in range(-r, size - r):
        for dx in range(-r, size - r):
            img[np.clip(py + dy, 0, h - 1), np.clip(px + dx, 0, w - 1)] = colors


# ---------------------------------------------------------------- panels
def front_view(pts, rmax, y_right=False, z_down=False, az_half=60.0, el_half=40.0, deg_per_px=0.5,
               log_scale=True, far=15.0):
    """LiDAR seen from the sensor, like the camera: horizontal = azimuth atan2(y, x), vertical = elevation.
    Default puts +y (left) on the left and +z (up) on top. Nearest point wins per pixel; color = range.
    Returns (image, stats dict)."""
    W, H = int(round(2 * az_half / deg_per_px)), int(round(2 * el_half / deg_per_px))
    rng = np.full(H * W, np.inf, np.float32)
    xyz = valid_xyz(pts)
    stats = {"valid": 0, "max_range": 0.0, "n_far": 0, "n_20": 0}
    far_px = far_py = far_r = None
    if xyz is not None and len(xyz):
        x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        r = np.sqrt(x * x + y * y + z * z)
        stats.update(valid=len(x), max_range=float(r.max()), n_far=int((r > far).sum()), n_20=int((r > 20).sum()))
        az = np.degrees(np.arctan2(y, x))
        el = np.degrees(np.arctan2(z, np.hypot(x, y)))
        u = (az + az_half) if y_right else (az_half - az)
        v = (el + el_half) if z_down else (el_half - el)
        px, py = (u / deg_per_px).astype(int), (v / deg_per_px).astype(int)
        inb = (px >= 0) & (px < W) & (py >= 0) & (py < H)
        px, py, r = px[inb], py[inb], r[inb]
        for dy in (-1, 0, 1):              # 3x3 splat so the view is not a sparse dot field
            for dx in (-1, 0, 1):
                np.minimum.at(rng, np.clip(py + dy, 0, H - 1) * W + np.clip(px + dx, 0, W - 1), r)
        m = r > far
        if m.any():
            far_px, far_py, far_r = px[m], py[m], r[m]
    rng = rng.reshape(H, W)
    valid = np.isfinite(rng)
    img = range_colors(np.where(valid, rng, RANGE_MIN).ravel(), rmax, log_scale).reshape(H, W, 3)
    img[~valid] = 0
    for a in range(-int(az_half), int(az_half) + 1, 30):            # angle grid
        c = int(round((az_half - a) / deg_per_px))
        cv2.line(img, (c, 0), (c, H - 1), (70, 70, 70) if a else (140, 140, 140), 1)
    for e in range(-int(el_half), int(el_half) + 1, 20):
        rr_ = int(round((el_half - e) / deg_per_px))
        cv2.line(img, (0, rr_), (W - 1, rr_), (70, 70, 70) if e else (140, 140, 140), 1)
    if far_px is not None:                                           # far points on top, bigger, white ring
        order = np.argsort(-far_r)
        splat(img, far_px[order], far_py[order], np.full((len(order), 3), 255, np.uint8), 5)
        splat(img, far_px[order], far_py[order], range_colors(far_r[order], rmax, log_scale), 3)
    return img, stats


def topdown(pts, size=480, extent=30.0, y_right=False, rmax=50.0, log_scale=True, color="range", far=15.0):
    """Top-down map: sensor at bottom-center, x forward = up, +y (left) on the left unless y_right."""
    canvas = np.zeros((size, size, 3), np.uint8)
    cx, cy = size // 2, size - 20
    ppm = (size - 40) / extent
    step = 1 if extent <= 10 else 5
    for m in range(step, int(extent) + 1, step):
        cv2.circle(canvas, (cx, cy), int(m * ppm), (40, 40, 40), 1)
        cv2.putText(canvas, f"{m}m", (cx + int(m * ppm) - 22, cy - 4), FONT, 0.35, (90, 90, 90), 1)
    cv2.line(canvas, (cx, cy), (cx, 10), (50, 50, 50), 1)
    xyz = valid_xyz(pts)
    if xyz is not None and len(xyz):
        x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        r = np.sqrt(x * x + y * y + z * z)
        px = (cx + y * ppm).astype(int) if y_right else (cx - y * ppm).astype(int)
        py = (cy - x * ppm).astype(int)
        inb = (px >= 0) & (px < size) & (py >= 0) & (py < size)
        px, py, r, z = px[inb], py[inb], r[inb], z[inb]
        if color == "height":
            cols = cv2.applyColorMap((np.clip((z + 1.0) / 3.0, 0, 1) * 255).astype(np.uint8), cv2.COLORMAP_TURBO).reshape(-1, 3)
        else:
            cols = range_colors(r, rmax, log_scale)
        order = np.argsort(-r)                                       # far first, near on top
        canvas[py[order], px[order]] = cols[order]
        m = r > far
        if m.any():
            splat(canvas, px[m], py[m], np.full((int(m.sum()), 3), 255, np.uint8), 5)
            splat(canvas, px[m], py[m], cols[m], 3)
    cv2.circle(canvas, (cx, cy), 4, (255, 255, 255), -1)
    cv2.putText(canvas, f"top-down {extent:g} m  (x fwd = up)  color = {color}  -/= zoom", (6, 16), FONT, 0.42, (200, 200, 200), 1)
    return canvas


def load_settings():
    s = dict(DEFAULT_SETTINGS)
    try:
        s.update(json.loads(SETTINGS.read_text()))
    except (OSError, ValueError):
        pass
    return s


def save_settings(s):
    try:
        SETTINGS.write_text(json.dumps(s, indent=2))
    except OSError as e:
        print("could not save settings:", e)


def save_snapshot(stream, canvas, tag="live"):
    out = ROOT / "captures" / (tag + "_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / "view.png"), canvas)
    with stream.lock:
        pts, img = stream.cloud, stream.image
    if img is not None:
        cv2.imwrite(str(out / "image.png"), img)
    if pts is not None:
        np.save(out / "cloud_grid.npy", pts)
        ok = np.isfinite(pts[:, :3]).all(axis=1) & (np.abs(pts[:, :3]).sum(axis=1) > 0)
        p = pts[ok]
        with open(out / "cloud.ply", "w") as f:
            f.write("ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\n"
                    "property uchar intensity\nend_header\n" % len(p))
            np.savetxt(f, np.column_stack([p[:, :3], p[:, 3].astype(int)]), fmt="%.4f %.4f %.4f %d")
    print(f"snapshot -> {out}")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--exe", default=str(ROOT / "rs_driver" / "build" / "demo" / "demo_ac1_stream.exe"))
    ap.add_argument("--port", type=int, default=5577)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--imu-fps", type=int, default=100)
    ap.add_argument("--no-image", action="store_true")
    ap.add_argument("--no-usb200", action="store_true", help="do not pass --usb200 to the streamer")
    ap.add_argument("--no-spawn", action="store_true", help="connect to an already running streamer")
    ap.add_argument("--max-range", type=float, default=50.0, help="top of the range color scale [m]")
    ap.add_argument("--extent", type=float, default=None, help="top-down map extent [m] (default: remembered, 30)")
    ap.add_argument("--far", type=float, default=15.0, help="points beyond this range are highlighted [m]")
    ap.add_argument("--seconds", type=float, default=0, help="auto-quit after N seconds (0 = run until q)")
    ap.add_argument("--snapshot-on-exit", action="store_true")
    args = ap.parse_args()
    settings = load_settings()
    if args.extent:
        settings["extent"] = args.extent

    proc = None
    if not args.no_spawn:
        cmd = [args.exe, "--port", str(args.port), "--width", str(args.width), "--height", str(args.height),
               "--fps", str(args.fps), "--imu-fps", str(args.imu_fps)]
        if not args.no_usb200:
            cmd.append("--usb200")
        if args.no_image:
            cmd.append("--no-image")
        print("launching:", " ".join(cmd))
        proc = subprocess.Popen(cmd, cwd=str(ROOT))

    stream = Stream(args.port)
    if not stream.connect():
        print("could not connect to the streamer", file=sys.stderr)
        if proc:
            proc.terminate()
        return 2

    win = "AC1 live  (q quit, s snapshot, h mirror, v flip, -/= zoom, k color, l scale)"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    FV_W, FV_H = 640, 427  # front view display size (120 x 80 deg)
    t0, canvas = time.time(), None
    try:
        while stream.alive:
            with stream.lock:
                pts = stream.cloud
                img = None if stream.image is None else stream.image.copy()
                meta = stream.image_meta
                imu = stream.imu[-1] if stream.imu else None
                f_cloud, f_img, f_imu = rate(stream.t_cloud), rate(stream.t_image), rate(stream.t_imu)
                n_stale = stream.n_stale
            log_scale = settings["log_scale"]

            cam = np.zeros((480, 640, 3), np.uint8)
            if img is not None:
                cam = cv2.resize(img, (640, 480), interpolation=cv2.INTER_AREA)
            cv2.putText(cam, f"camera {meta}  {f_img:4.1f} fps", (6, 18), FONT, 0.5, (255, 255, 255), 1)
            td = topdown(pts, 480, settings["extent"], settings["y_right"], args.max_range, log_scale, settings["td_color"], args.far)
            top = np.hstack([cam, td])

            fv, st = front_view(pts, args.max_range, settings["y_right"], settings["z_down"], log_scale=log_scale, far=args.far)
            fv = cv2.resize(fv, (FV_W, FV_H), interpolation=cv2.INTER_NEAREST)
            bottom = np.zeros((FV_H + 24, top.shape[1], 3), np.uint8)
            bottom[24:24 + FV_H, :FV_W] = fv
            cv2.putText(bottom, f"LiDAR front view (az +-60, el +-40)  valid={st['valid']}  {f_cloud:4.1f} Hz   "
                                f"MAX RANGE {st['max_range']:5.1f} m   >{args.far:g} m: {st['n_far']}   >20 m: {st['n_20']}",
                        (6, 16), FONT, 0.5, (255, 255, 255), 1)
            draw_range_legend(bottom, args.max_range, log_scale, x=FV_W + 12, y=bottom.shape[0] - 40)
            x0, y = FV_W + 12, 44
            if imu:
                ts, ax_, ay_, az_, gx, gy, gz = imu[:7]
                for line in (f"IMU {f_imu:5.1f} Hz", f"acc {ax_:6.2f} {ay_:6.2f} {az_:6.2f} m/s2",
                             f"gyr {gx:6.3f} {gy:6.3f} {gz:6.3f} rad/s", f"t   {ts:.3f}"):
                    cv2.putText(bottom, line, (x0, y), FONT, 0.5, (200, 255, 200), 1)
                    y += 22
            y += 8
            for line in ("frame: x fwd, y left, z up",
                         f"h mirror [{'on' if settings['y_right'] else 'off'}]   v flip [{'on' if settings['z_down'] else 'off'}]",
                         f"k top-down color [{settings['td_color']}]   l scale [{'log' if log_scale else 'linear'}]",
                         f"far points (> {args.far:g} m) = white ring" + (f"   stale frames skipped: {n_stale}" if n_stale else ""),
                         "s snapshot   q quit"):
                cv2.putText(bottom, line, (x0, y), FONT, 0.45, (160, 160, 160), 1)
                y += 20
            canvas = np.vstack([top, bottom])
            cv2.imshow(win, canvas)
            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                save_snapshot(stream, canvas)
            if key == ord("h"):
                settings["y_right"] = not settings["y_right"]; save_settings(settings)
            if key == ord("v"):
                settings["z_down"] = not settings["z_down"]; save_settings(settings)
            if key in (ord("-"), ord("_")):
                i = EXTENTS.index(settings["extent"]) if settings["extent"] in EXTENTS else 2
                settings["extent"] = EXTENTS[max(0, i - 1)]; save_settings(settings)
            if key in (ord("="), ord("+")):
                i = EXTENTS.index(settings["extent"]) if settings["extent"] in EXTENTS else 2
                settings["extent"] = EXTENTS[min(len(EXTENTS) - 1, i + 1)]; save_settings(settings)
            if key == ord("k"):
                settings["td_color"] = "height" if settings["td_color"] == "range" else "range"; save_settings(settings)
            if key == ord("l"):
                settings["log_scale"] = not settings["log_scale"]; save_settings(settings)
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
            save_snapshot(stream, canvas)
        stream.close()
        cv2.destroyAllWindows()
        if proc:
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                proc.terminate()
            print("streamer exit code:", proc.returncode)
    return 0


if __name__ == "__main__":
    sys.exit(main())
