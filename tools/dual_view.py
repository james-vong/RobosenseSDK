"""Two AC1 units in ONE Rerun recording, with on-disk recording of both clouds and precise timestamps.

Spawns one demo_ac1_stream per unit (selected by serial), connects to both, and logs everything into a single
Rerun recording: world/<unit>/lidar (3D), cams/<unit>/image (2D), imu/<unit>/... . Two timelines are set on
every log: "host" (arrival time on this laptop, time.time()) and "device" (the AC1's own cloud timestamp,
epoch seconds, as sent by the driver). Unit B can be placed relative to unit A with --pose-b.

Recording (default on) writes captures/dual_<ts>/:
  dual.rrd                      the Rerun recording (open later: .venv\\Scripts\\rerun.exe dual.rrd)
  <unit>/cloud_<seq>.npy        every cloud, full 288x96 grid as float32 (x,y,z,intensity), NaN = no return
  <unit>/img_<seq>.jpg          camera frames at --image-fps
  clouds.csv                    unit, seq, device_ts, host_ts, n_valid, max_range  (one row per cloud)
  images.csv                    unit, seq, host_ts, file
  meta.json                     units, settings, calibration

Usage: dual_view.py [--unit SERIAL:PORT ...] [--fps 10] [--image-fps 5] [--color rgb|range]
                    [--pose-b x y z yaw_deg] [--record DIR | --no-record] [--seconds N]
Stop with Ctrl+C (clean) or Stop-Process -Name demo_ac1_stream.
"""
import argparse
import csv
import datetime as dt
import json
import math
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

DEFAULT_UNITS = ["1111bfaf0060:5577", "1111bfa90008:5578"]


def yaw_matrix(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--unit", action="append", help="SERIAL:PORT (repeat per unit; default: the two known units)")
    ap.add_argument("--exe", default=str(ROOT / "rs_driver" / "build" / "demo" / "demo_ac1_stream.exe"))
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=10, help="camera rate requested from the device: 30, 15 or 10")
    ap.add_argument("--image-fps", type=float, default=5.0, help="camera frames per second logged/saved per unit")
    ap.add_argument("--color", choices=("rgb", "range"), default="rgb")
    ap.add_argument("--mode", choices=("squash", "crop"), default="squash")
    ap.add_argument("--calib", default=None)
    ap.add_argument("--max-range", type=float, default=50.0)
    ap.add_argument("--pose-b", nargs=4, type=float, metavar=("X", "Y", "Z", "YAW_DEG"), default=[0, 0, 0, 0],
                    help="place the second unit relative to the first (m, m, m, deg about z)")
    ap.add_argument("--record", default=None, help="output folder (default captures/dual_<timestamp>)")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--seconds", type=float, default=0)
    ap.add_argument("--viewer-memory", default="3GiB")
    ap.add_argument("--layout", choices=("split", "combined"), default="split",
                    help="split = one 3D panel per unit (default); combined = both clouds in one 3D scene")
    ap.add_argument("--no-usb200", action="store_true")
    args = ap.parse_args()
    if args.fps not in (30, 15, 10):
        print("camera supports 30, 15 or 10 fps only; using 10", file=sys.stderr)
        args.fps = 10

    units = []
    for spec in (args.unit or DEFAULT_UNITS):
        serial, port = spec.split(":")
        units.append({"serial": serial.lower(), "port": int(port), "name": f"unit_{serial.lower()[-4:]}"})
    calib = Ac1Calib(args.calib)
    print(calib.describe())
    W, H = args.width, args.height
    K = calib.K_for(W, H, args.mode)

    rec = None
    if not args.no_record:
        rec = Path(args.record) if args.record else ROOT / "captures" / ("dual_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
        for u in units:
            (rec / u["name"]).mkdir(parents=True, exist_ok=True)
        clouds_csv = open(rec / "clouds.csv", "w", newline="")
        cw = csv.writer(clouds_csv)
        cw.writerow(["unit", "seq", "device_ts", "host_ts", "n_valid", "max_range_m"])
        images_csv = open(rec / "images.csv", "w", newline="")
        iw = csv.writer(images_csv)
        iw.writerow(["unit", "seq", "host_ts", "file"])
        json.dump({"units": units, "camera": f"{W}x{H}@{args.fps}", "image_fps_logged": args.image_fps, "color": args.color,
                   "pose_b": args.pose_b, "calib": str(calib.path), "calib_device": calib.device_id,
                   "frame": "x fwd, y left, z up; cloud npy = (27648,4) float32 x,y,z,intensity, NaN = no return",
                   "timestamps": "device_ts = AC1 cloud timestamp (epoch s, driver-synced); host_ts = laptop time.time() at arrival",
                   "started": dt.datetime.now().isoformat()}, open(rec / "meta.json", "w"), indent=2)
        print("recording to", rec)

    # ---- Rerun: one recording, two units
    cams = [rrb.Spatial2DView(origin=f"cams/{u['name']}/image", name=f"camera {u['name']}") for u in units]
    if args.layout == "split":
        rows = [rrb.Horizontal(rrb.Spatial3DView(origin=f"world/{u['name']}", name=f"3D {u['name']} (x fwd, y left, z up)"),
                               cam, column_shares=[3, 2]) for u, cam in zip(units, cams)]
        bp = rrb.Blueprint(rrb.Vertical(*rows), collapse_panels=True)
    else:
        bp = rrb.Blueprint(
            rrb.Horizontal(
                rrb.Spatial3DView(origin="world", name="both AC1s (x fwd, y left, z up)"),
                rrb.Vertical(*cams),
                column_shares=[3, 2],
            ),
            collapse_panels=True,
        )
    sdk_root = Path(rr.__file__).resolve().parent.parent
    viewer = next((p for p in (sdk_root / "rerun_cli" / "rerun.exe", sdk_root / "rerun_cli" / "rerun") if p.exists()), None)
    rr.init("robosense_ac1_dual", default_blueprint=bp)
    rr.spawn(executable_path=str(viewer) if viewer else None, default_blueprint=bp, memory_limit=args.viewer_memory,
             server_memory_limit="512MiB", connect=False)
    sinks = [rr.GrpcSink()] + ([rr.FileSink(str(rec / "dual.rrd"))] if rec else [])
    rr.set_sinks(*sinks, default_blueprint=bp)
    rr.send_blueprint(bp, make_active=True, make_default=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    for i, u in enumerate(units):
        base = f"world/{u['name']}"
        if i == 1:
            x, y, z, yaw = args.pose_b
            rr.log(base, rr.Transform3D(translation=[x, y, z], mat3x3=yaw_matrix(yaw)), static=True)
        rr.log(base + "/axes", rr.Arrows3D(origins=np.zeros((3, 3)), vectors=np.eye(3) * 0.25,
                                           colors=[[255, 60, 60], [60, 255, 60], [60, 60, 255]], labels=[u["name"], "", ""]), static=True)
        rr.log(base + "/camera", rr.Transform3D(translation=calib.t_lc, mat3x3=calib.R_lc), static=True)
        rr.log(base + "/camera/frustum", rr.Pinhole(image_from_camera=K, resolution=[W, H], camera_xyz=rr.ViewCoordinates.RDF,
                                                    image_plane_distance=0.3), static=True)
        rr.log(f"imu/{u['name']}/acc", rr.SeriesLines(names=["ax", "ay", "az"]), static=True)

    # ---- streamers + streams
    procs, streams = [], []
    for u in units:
        cmd = [args.exe, "--port", str(u["port"]), "--uuid", u["serial"], "--width", str(W), "--height", str(H), "--fps", str(args.fps)]
        if not args.no_usb200:
            cmd.append("--usb200")
        print("launching:", " ".join(cmd))
        procs.append(subprocess.Popen(cmd, cwd=str(ROOT)))
        time.sleep(1.0)
    for u in units:
        s = Stream(u["port"])
        if not s.connect(timeout_s=25.0):
            print(f"could not connect to streamer for {u['name']} on port {u['port']}", file=sys.stderr)
            for p in procs:
                p.terminate()
            return 2
        streams.append(s)
    print("both streams connected")

    t0 = time.time()
    st = [{"cloud_n": 0, "img_n": 0, "imu_n": 0, "seq": 0, "iseq": 0, "last_img_t": 0.0, "offsets": []} for _ in units]
    img_interval = 1.0 / args.image_fps if args.image_fps > 0 else 0
    last_report = -1
    try:
        while all(s.alive for s in streams):
            time.sleep(0.005)
            now = time.time()
            for i, (u, s, z) in enumerate(zip(units, streams, st)):
                base = f"world/{u['name']}"
                with s.lock:
                    pts, cloud_ts, cloud_n = s.cloud, s.cloud_ts, s.n_cloud
                    img, img_n = s.image, s.n_image
                    imu_n = s.n_imu
                    k = min(imu_n - z["imu_n"], s.imu.maxlen)
                    imu_new = list(s.imu)[-k:] if k > 0 else []
                for smp in imu_new:
                    ts, ax, ay, az_ = smp[:4]
                    rr.set_time("host", timestamp=now)
                    rr.set_time("device", timestamp=ts if ts > 1e9 else now)
                    rr.log(f"imu/{u['name']}/acc", rr.Scalars([ax, ay, az_]))
                z["imu_n"] = imu_n
                if pts is not None and cloud_n != z["cloud_n"]:
                    z["cloud_n"] = cloud_n
                    z["seq"] += 1
                    xyz = valid_xyz(pts)
                    rr.set_time("host", timestamp=now)
                    rr.set_time("device", timestamp=cloud_ts if cloud_ts > 1e9 else now)
                    rng = np.linalg.norm(xyz, axis=1) if xyz is not None and len(xyz) else np.zeros(0)
                    if len(rng):
                        cols = range_colors(rng, args.max_range)[:, ::-1]
                        if args.color == "rgb" and img is not None and img.shape[1] == W and img.shape[0] == H:
                            uv, inb, _ = calib.project(xyz, W, H, args.mode)
                            rgb = (cols * 0.45).astype(np.uint8)
                            uu = np.clip(uv[inb, 0].astype(int), 0, W - 1)
                            vv = np.clip(uv[inb, 1].astype(int), 0, H - 1)
                            rgb[inb] = img[vv, uu][:, ::-1]
                            cols = rgb
                        rr.log(base + "/lidar", rr.Points3D(positions=xyz, colors=cols, radii=rr.Radius.ui_points(2.0)))
                    if rec:
                        np.save(rec / u["name"] / f"cloud_{z['seq']:06d}.npy", pts)
                        cw.writerow([u["name"], z["seq"], f"{cloud_ts:.6f}", f"{now:.6f}", len(rng), f"{rng.max():.3f}" if len(rng) else ""])
                    if cloud_ts > 1e9:
                        z["offsets"].append(now - cloud_ts)
                if img is not None and img_n != z["img_n"] and now - z["last_img_t"] >= img_interval:
                    z["img_n"], z["last_img_t"] = img_n, now
                    z["iseq"] += 1
                    rr.set_time("host", timestamp=now)
                    rr.set_time("device", timestamp=now)
                    rr.log(f"cams/{u['name']}/image", rr.Image(img, color_model="BGR").compress(jpeg_quality=80))
                    if rec:
                        fn = f"img_{z['iseq']:06d}.jpg"
                        cv2.imwrite(str(rec / u["name"] / fn), img, [cv2.IMWRITE_JPEG_QUALITY, 90])
                        iw.writerow([u["name"], z["iseq"], f"{now:.6f}", fn])
            if rec and int(now - t0) % 10 == 0:
                clouds_csv.flush()
                images_csv.flush()
            if int(now - t0) // 5 != last_report:
                last_report = int(now - t0) // 5
                parts = []
                for u, z in zip(units, st):
                    off = np.median(z["offsets"][-50:]) if z["offsets"] else float("nan")
                    parts.append(f"{u['name']}: clouds={z['seq']} imgs={z['iseq']} host-device={off * 1000:+.0f}ms")
                print(f"t={int(now - t0)}s  " + " | ".join(parts))
            if args.seconds and now - t0 > args.seconds:
                break
        for s in streams:
            if s.error:
                print("stream ended:", s.error)
    except KeyboardInterrupt:
        pass
    finally:
        for s in streams:
            s.close()
        for p in procs:
            try:
                p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                p.terminate()
        if rec:
            clouds_csv.close()
            images_csv.close()
            print("recorded to", rec)
        # device-clock agreement between the two units (host arrival minus device stamp, per unit)
        offs = [np.median(z["offsets"]) if z["offsets"] else float("nan") for z in st]
        if len(offs) == 2 and all(np.isfinite(offs)):
            print(f"median host-minus-device offset: {units[0]['name']} {offs[0] * 1000:+.1f} ms, {units[1]['name']} {offs[1] * 1000:+.1f} ms, "
                  f"difference {abs(offs[0] - offs[1]) * 1000:.1f} ms")
        time.sleep(0.5)
        rr.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
