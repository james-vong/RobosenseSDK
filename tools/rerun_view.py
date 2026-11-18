"""Rerun viewer for the live AC1: 3D point cloud, camera frustum + image with the LiDAR overlay, IMU time series.

Launches demo_ac1_stream.exe (like live_view.py), spawns the Rerun viewer and streams into it.
Stop with Ctrl+C in the terminal or --seconds N. Use --save FILE.rrd to also record.

Usage: rerun_view.py [--width 640 --height 480 --fps 30] [--mode squash|crop] [--max-range 8]
                     [--seconds N] [--save out.rrd] [--calib PATH] [--no-spawn] [--port 5577]
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rerun as rr
import rerun.blueprint as rrb

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ac1_calib import Ac1Calib  # noqa: E402
from live_view import ROOT, Stream, range_colors, valid_xyz  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--exe", default=str(ROOT / "rs_driver" / "build" / "demo" / "demo_ac1_stream.exe"))
    ap.add_argument("--port", type=int, default=5577)
    ap.add_argument("--uuid", default=None, help="AC1 serial to open (e.g. 1111bfaf0060); needed when two units are connected")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--mode", choices=("squash", "crop"), default="squash")
    ap.add_argument("--max-range", type=float, default=50.0, help="top of the log range color scale [m]")
    ap.add_argument("--far", type=float, default=15.0, help="points beyond this range are drawn bigger [m]")
    ap.add_argument("--calib", default=None)
    ap.add_argument("--seconds", type=float, default=0)
    ap.add_argument("--save", default=None, help="also record to this .rrd file")
    ap.add_argument("--image-fps", type=float, default=10.0, help="max camera frames per second logged to Rerun (history costs RAM)")
    ap.add_argument("--overlay", action="store_true", help="also draw the LiDAR points on the camera image (off by default)")
    ap.add_argument("--color", choices=("rgb", "range", "both"), default="rgb",
                    help="3D point color: camera pixel (rgb), log range (range), or both as separate entities")
    ap.add_argument("--uncolored", choices=("range", "gray", "hide"), default="range",
                    help="rgb mode: points outside the camera image are dimmed range colors, gray, or hidden")
    ap.add_argument("--layout", choices=("simple", "full"), default="simple",
                    help="simple = 3D view + camera panel only (default); full = also range stats and IMU time series")
    ap.add_argument("--image-plane", choices=("off", "on", "none"), default="off",
                    help="3D view: 'off' = camera frustum only (default), 'on' = frustum with the image projected on it, 'none' = no camera at all")
    ap.add_argument("--viewer-memory", default="3GiB", help="Rerun viewer RAM cap; oldest data is dropped past it (default 3GiB, Rerun's own default is 75%% of RAM)")
    ap.add_argument("--no-usb200", action="store_true")
    ap.add_argument("--no-spawn", action="store_true")
    args = ap.parse_args()

    calib = Ac1Calib(args.calib)
    print(calib.describe())
    W, H = args.width, args.height
    K = calib.K_for(W, H, args.mode)
    # where the camera image lives: under the pinhole (projected onto the image plane in 3D) or outside "world"
    IMG = "world/camera/image" if args.image_plane == "on" else "camera/image"

    view3d = rrb.Spatial3DView(origin="world", name=f"AC1 3D, {args.color} colors (x fwd, y left, z up)")
    cam2d = rrb.Spatial2DView(origin=IMG, name="camera" + (" + LiDAR overlay" if args.overlay else ""))
    if args.layout == "simple":
        bp = rrb.Blueprint(rrb.Horizontal(view3d, cam2d, column_shares=[3, 2]), collapse_panels=True)
    else:
        bp = rrb.Blueprint(
            rrb.Horizontal(
                view3d,
                rrb.Vertical(
                    cam2d,
                    rrb.TimeSeriesView(origin="stats", name="max range [m] / points beyond 20 m"),
                    rrb.TimeSeriesView(origin="imu", name="IMU"),
                    row_shares=[3, 2, 2],
                ),
                column_shares=[3, 2],
            ),
            collapse_panels=True,
        )
    # Spawn the viewer bundled with THIS rerun-sdk, not whatever `rerun` is on PATH (an older one there
    # silently drops every message with "Bad chunk schema").
    sdk_root = Path(rr.__file__).resolve().parent.parent
    viewer = next((p for p in (sdk_root / "rerun_cli" / "rerun.exe", sdk_root / "rerun_cli" / "rerun") if p.exists()), None)
    rr.init("robosense_ac1_live" + (f"_{args.uuid}" if args.uuid else ""), default_blueprint=bp)
    rr.spawn(executable_path=str(viewer) if viewer else None, default_blueprint=bp,
             memory_limit=args.viewer_memory, server_memory_limit="512MiB", connect=not args.save)
    if args.save:   # viewer + file together (rr.save alone would replace the viewer sink)
        rr.set_sinks(rr.GrpcSink(), rr.FileSink(args.save), default_blueprint=bp)
    # the viewer keeps the last blueprint it had for this app id; force ours so panel origins match what we log
    rr.send_blueprint(bp, make_active=True, make_default=True)
    print(f"rerun viewer: {viewer or '(from PATH)'}  memory cap {args.viewer_memory}, images logged at <= {args.image_fps:g} fps, "
          f"point color = {args.color}")

    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/sensor_axes", rr.Arrows3D(origins=np.zeros((3, 3)), vectors=np.eye(3) * 0.3,
                                            colors=[[255, 60, 60], [60, 255, 60], [60, 60, 255]],
                                            labels=["x fwd", "y left", "z up"]), static=True)
    if args.image_plane != "none":
        rr.log("world/camera", rr.Transform3D(translation=calib.t_lc, mat3x3=calib.R_lc), static=True)
        pin_path = "world/camera/image" if args.image_plane == "on" else "world/camera/frustum"
        rr.log(pin_path, rr.Pinhole(image_from_camera=K, resolution=[W, H], camera_xyz=rr.ViewCoordinates.RDF,
                                    image_plane_distance=0.4), static=True)
    rr.log("imu/acc", rr.SeriesLines(names=["ax", "ay", "az"], colors=[[255, 80, 80], [80, 255, 80], [80, 80, 255]]), static=True)
    rr.log("imu/gyro", rr.SeriesLines(names=["gx", "gy", "gz"], colors=[[255, 80, 80], [80, 255, 80], [80, 80, 255]]), static=True)
    rr.log("stats/max_range_m", rr.SeriesLines(names=["max range [m]"], colors=[[255, 200, 60]]), static=True)
    rr.log("stats/points_beyond_20m", rr.SeriesLines(names=["points > 20 m"], colors=[[120, 200, 255]]), static=True)
    rr.log("info", rr.TextLog(f"{calib.describe()} | intrinsics mode={args.mode} at {W}x{H}"), static=True)

    proc = None
    if not args.no_spawn:
        cmd = [args.exe, "--port", str(args.port), "--width", str(W), "--height", str(H), "--fps", str(args.fps)]
        if args.uuid:
            cmd += ["--uuid", args.uuid]
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

    t0 = time.time()
    last_cloud_n = last_img_n = last_imu_n = 0
    n_clouds = n_imgs = n_imu = 0
    last_report = -1
    last_img_log_t = 0.0
    img_interval = 1.0 / args.image_fps if args.image_fps > 0 else 0.0
    try:
        while stream.alive:
            time.sleep(0.01)
            with stream.lock:
                pts, cloud_n = stream.cloud, stream.n_cloud
                img, img_n = stream.image, stream.n_image
                imu_n = stream.n_imu
                k = min(imu_n - last_imu_n, stream.imu.maxlen)
                imu_new = list(stream.imu)[-k:] if k > 0 else []
            now = time.time()
            # IMU: log every new sample on its own timestamp
            for s in imu_new:
                ts, ax, ay, az, gx, gy, gz = s[:7]
                rr.set_time("time", timestamp=now if ts < 1e9 else ts)
                rr.log("imu/acc", rr.Scalars([ax, ay, az]))
                rr.log("imu/gyro", rr.Scalars([gx, gy, gz]))
                n_imu += 1
            last_imu_n = imu_n
            if pts is not None and cloud_n != last_cloud_n:
                last_cloud_n = cloud_n
                xyz = valid_xyz(pts)
                rr.set_time("time", timestamp=now)
                if xyz is not None and len(xyz):
                    rng = np.linalg.norm(xyz, axis=1)
                    cols = range_colors(rng, args.max_range)[:, ::-1]          # BGR -> RGB, log scale
                    far = rng > args.far
                    need_proj = args.overlay or args.color in ("rgb", "both")
                    uv = inb = None
                    if need_proj:
                        uv, inb, depth = calib.project(xyz, W, H, args.mode)
                    if args.color in ("rgb", "both"):
                        if img is not None and inb is not None and img.shape[1] == W and img.shape[0] == H:
                            rgb = (cols * 0.45).astype(np.uint8) if args.uncolored == "range" else np.full_like(cols, 90)
                            u = np.clip(uv[inb, 0].astype(int), 0, W - 1)
                            v = np.clip(uv[inb, 1].astype(int), 0, H - 1)
                            rgb[inb] = img[v, u][:, ::-1]                        # latest camera frame, BGR -> RGB
                            keep = inb if args.uncolored == "hide" else np.ones(len(xyz), bool)
                            rr.log("world/lidar_rgb", rr.Points3D(positions=xyz[keep], colors=rgb[keep], radii=rr.Radius.ui_points(2.0)))
                        else:                                                   # no image yet: fall back to range colors
                            rr.log("world/lidar_rgb", rr.Points3D(positions=xyz, colors=cols, radii=rr.Radius.ui_points(2.0)))
                    if args.color in ("range", "both"):
                        rr.log("world/lidar", rr.Points3D(positions=xyz[~far], colors=cols[~far], radii=rr.Radius.ui_points(1.5)))
                        rr.log("world/lidar_far", rr.Points3D(positions=xyz[far], colors=cols[far], radii=rr.Radius.ui_points(4.0)))
                    rr.log("stats/max_range_m", rr.Scalars([float(rng.max())]))
                    rr.log("stats/points_beyond_20m", rr.Scalars([float((rng > 20).sum())]))
                    if args.overlay:
                        if inb.any():
                            rr.log(IMG + "/lidar", rr.Points2D(positions=uv[inb & ~far], colors=cols[inb & ~far], radii=rr.Radius.ui_points(1.5)))
                            rr.log(IMG + "/lidar_far", rr.Points2D(positions=uv[inb & far], colors=cols[inb & far], radii=rr.Radius.ui_points(4.0)))
                n_clouds += 1
            if img is not None and img_n != last_img_n and now - last_img_log_t >= img_interval:
                last_img_n, last_img_log_t = img_n, now
                rr.set_time("time", timestamp=now)
                rr.log(IMG, rr.Image(img, color_model="BGR").compress(jpeg_quality=80))
                n_imgs += 1
            if args.seconds and now - t0 > args.seconds:
                break
            if int(now - t0) // 5 != last_report:
                last_report = int(now - t0) // 5
                print(f"t={int(now - t0)}s logged clouds={n_clouds} images={n_imgs} imu={n_imu}")
        if stream.error:
            print("stream ended:", stream.error)
    except KeyboardInterrupt:
        pass
    finally:
        stream.close()
        if proc:
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                proc.terminate()
        print(f"done: clouds={n_clouds} images={n_imgs} imu={n_imu}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
