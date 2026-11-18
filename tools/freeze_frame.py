"""Freeze frame: grab one synchronized AC1 point cloud + camera image, color every LiDAR point with the
camera pixel it projects onto, save it, and open it in Rerun as a static 3D scene you can orbit.

Connects to a running demo_ac1_stream (e.g. one started by rerun_view.py / live_view.py) if there is one,
otherwise launches its own. Output: captures/freeze_<timestamp>/{cloud_rgb.ply, image.png, cloud_grid.npy,
meta.json, freeze.rrd}. Reopen later with:  .venv\\Scripts\\rerun.exe captures\\freeze_<ts>\\freeze.rrd

Usage: freeze_frame.py [--width 640 --height 480 --fps 30] [--mode squash|crop] [--calib PATH]
                       [--uncolored range|gray|hide] [--out DIR] [--no-rerun] [--port 5577]
"""
import argparse
import datetime as dt
import json
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ac1_calib import Ac1Calib  # noqa: E402
from live_view import ROOT, Stream, range_colors  # noqa: E402


def grab(stream, timeout=20.0):
    """Wait for a cloud and the image closest in arrival time to it."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.02)
        with stream.lock:
            if stream.cloud is not None and stream.image is not None and stream.t_cloud and stream.t_image:
                dt_ = abs(stream.t_cloud[-1] - stream.t_image[-1])
                if dt_ < 0.08:                      # cloud at 10 Hz, image at 30 Hz: wait for a near-coincident pair
                    return stream.cloud.copy(), stream.cloud_ts, stream.image.copy(), stream.image_meta, dt_
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--exe", default=str(ROOT / "rs_driver" / "build" / "demo" / "demo_ac1_stream.exe"))
    ap.add_argument("--port", type=int, default=5577)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--mode", choices=("squash", "crop"), default="squash")
    ap.add_argument("--calib", default=None)
    ap.add_argument("--uncolored", choices=("range", "gray", "hide"), default="range",
                    help="what to do with points outside the camera image")
    ap.add_argument("--max-range", type=float, default=50.0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--no-rerun", action="store_true")
    ap.add_argument("--no-usb200", action="store_true")
    args = ap.parse_args()

    calib = Ac1Calib(args.calib)
    print(calib.describe())

    # attach to a running streamer if one is up, else launch our own
    stream = Stream(args.port)
    proc = None
    if stream.connect(timeout_s=1.0):
        print("attached to the running streamer")
    else:
        cmd = [args.exe, "--port", str(args.port), "--width", str(args.width), "--height", str(args.height), "--fps", str(args.fps)]
        if not args.no_usb200:
            cmd.append("--usb200")
        print("launching:", " ".join(cmd))
        proc = subprocess.Popen(cmd, cwd=str(ROOT))
        if not stream.connect():
            print("could not connect to the streamer", file=sys.stderr)
            proc.terminate()
            return 2

    got = grab(stream)
    stream.close()
    if proc:
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            proc.terminate()
    if got is None:
        print("no synchronized cloud + image within the timeout", file=sys.stderr)
        return 3
    pts, cloud_ts, img, meta, sync_dt = got
    H, W = img.shape[:2]
    print(f"frame: {pts.shape[0]} grid points, image {W}x{H}, cloud/image arrival gap {sync_dt * 1000:.0f} ms")

    # colorize
    xyz_all = pts[:, :3]
    ok = np.isfinite(xyz_all).all(axis=1) & (np.abs(xyz_all).sum(axis=1) > 0)
    xyz = xyz_all[ok]
    inten = pts[ok, 3]
    rng = np.linalg.norm(xyz, axis=1)
    uv, inb, depth = calib.project(xyz, W, H, args.mode)
    rgb = np.zeros((len(xyz), 3), np.uint8)
    u = np.clip(uv[inb, 0].astype(int), 0, W - 1)
    v = np.clip(uv[inb, 1].astype(int), 0, H - 1)
    rgb[inb] = img[v, u][:, ::-1]                                    # BGR -> RGB
    if args.uncolored == "range":
        rgb[~inb] = (range_colors(rng[~inb], args.max_range)[:, ::-1] * 0.45).astype(np.uint8)
    elif args.uncolored == "gray":
        rgb[~inb] = 90
    keep = np.ones(len(xyz), bool) if args.uncolored != "hide" else inb
    xyz_k, rgb_k, rng_k, inten_k = xyz[keep], rgb[keep], rng[keep], inten[keep]
    print(f"valid points {len(xyz)}, inside camera image {int(inb.sum())} ({100 * inb.mean():.0f}%), "
          f"max range {rng.max():.2f} m")

    # save
    out = Path(args.out) if args.out else ROOT / "captures" / ("freeze_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / "image.png"), img)
    np.save(out / "cloud_grid.npy", pts)
    with open(out / "cloud_rgb.ply", "w") as f:
        f.write("ply\nformat ascii 1.0\ncomment RoboSense AC1 freeze frame, colors from camera via %s (%s)\n"
                "element vertex %d\nproperty float x\nproperty float y\nproperty float z\n"
                "property uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar intensity\nend_header\n"
                % (calib.path.name, args.mode, len(xyz_k)))
        np.savetxt(f, np.column_stack([xyz_k, rgb_k, inten_k.astype(int)]), fmt="%.4f %.4f %.4f %d %d %d %d")
    json.dump({"cloud_timestamp": cloud_ts, "image_meta": meta, "sync_gap_s": sync_dt, "calib": str(calib.path),
               "calib_device": calib.device_id, "intrinsics_mode": args.mode, "points_valid": int(len(xyz)),
               "points_in_image": int(inb.sum()), "max_range_m": float(rng.max()), "frame": "x fwd, y left, z up"},
              open(out / "meta.json", "w"), indent=2)
    print("saved ->", out)

    if args.no_rerun:
        return 0
    import rerun as rr
    import rerun.blueprint as rrb
    sdk_root = Path(rr.__file__).resolve().parent.parent
    viewer = next((p for p in (sdk_root / "rerun_cli" / "rerun.exe", sdk_root / "rerun_cli" / "rerun") if p.exists()), None)
    bp = rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="world", name="freeze frame 3D (drag to orbit)"),
            rrb.Spatial2DView(origin="camera/image", name="camera"),
            column_shares=[3, 2],
        ),
        collapse_panels=True,
    )
    rr.init("ac1_" + out.name, default_blueprint=bp)
    # launch (or reuse) the viewer, then send to BOTH the viewer and the .rrd file. (rr.save() alone would
    # replace the viewer sink, leaving an empty recording in the viewer.)
    rr.spawn(executable_path=str(viewer) if viewer else None, default_blueprint=bp, memory_limit="1GiB",
             server_memory_limit="256MiB", connect=False)
    rr.set_sinks(rr.GrpcSink(), rr.FileSink(str(out / "freeze.rrd")), default_blueprint=bp)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/sensor_axes", rr.Arrows3D(origins=np.zeros((3, 3)), vectors=np.eye(3) * 0.3,
                                            colors=[[255, 60, 60], [60, 255, 60], [60, 60, 255]], labels=["x fwd", "y left", "z up"]), static=True)
    rr.log("world/lidar_rgb", rr.Points3D(positions=xyz_k, colors=rgb_k, radii=rr.Radius.ui_points(2.5)), static=True)
    # camera frustum only in 3D (no projected image plane); the image goes to its own 2D panel
    rr.log("world/camera", rr.Transform3D(translation=calib.t_lc, mat3x3=calib.R_lc), static=True)
    rr.log("world/camera/frustum", rr.Pinhole(image_from_camera=calib.K_for(W, H, args.mode), resolution=[W, H],
                                              camera_xyz=rr.ViewCoordinates.RDF, image_plane_distance=0.5), static=True)
    rr.log("camera/image", rr.Image(img, color_model="BGR").compress(jpeg_quality=90), static=True)
    rr.log("info", rr.TextLog(f"freeze frame {out.name}: {len(xyz_k)} points, {int(inb.sum())} camera-colored, "
                              f"max range {rng.max():.1f} m, {calib.describe()}"), static=True)
    time.sleep(1.0)      # let the gRPC sink drain before the process exits
    rr.disconnect()
    print("opened in Rerun as recording", "ac1_" + out.name, "; file:", out / "freeze.rrd")
    return 0


if __name__ == "__main__":
    sys.exit(main())
