"""Scene-based extrinsic calibration between two AC1 units from a dual recording.

Takes N consecutive frames per unit ending at the given sequence numbers (near-simultaneous pair), denoises
each unit by per-cell median, levels both clouds on their floor planes (fixes roll, pitch, height), solves the
remaining yaw + x/y by global registration and a yaw sweep, refines with point-to-plane ICP, and reports the
transform that maps unit B points into unit A's frame together with agreement statistics.

Outputs <rec>/calib_A<seqA>_B<seqB>/: T_B_to_A.json, cloud_A.ply, cloud_B_in_A.ply, merged_union.ply,
merged_consensus.ply, disagreement_*.ply, preview.png, and a Rerun recording of the result.

Usage: calibrate_dual.py <rec_folder> --seq-a 134 --seq-b 123 [--nframes 2] [--no-rerun]
"""
import argparse
import datetime as dt
import json
import math
import sys
import time
from pathlib import Path

import matplotlib
import numpy as np
import open3d as o3d

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from live_view import ROOT  # noqa: E402

UNITS = ("unit_0060", "unit_0008")


def load_median(rec, unit, seq_end, n):
    """Per-cell median over n consecutive frames -> (27648,3) with NaN where no frame had a return."""
    frames = []
    for s in range(seq_end - n + 1, seq_end + 1):
        f = rec / unit / f"cloud_{s:06d}.npy"
        if f.exists():
            frames.append(np.load(f)[:, :3])
    stack = np.stack(frames)                       # (n, 27648, 3)
    with np.errstate(all="ignore"):
        med = np.nanmedian(stack, axis=0)
    ok = np.isfinite(med).all(axis=1) & (np.linalg.norm(med, axis=1) > 0.3)
    return med[ok], len(frames)


def to_pcd(xyz, color):
    p = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz.astype(np.float64)))
    p.paint_uniform_color(color)
    return p


def level_on_floor(xyz):
    """Rotation+translation that puts the dominant near-horizontal plane below the sensor at z=0 with normal +z."""
    below = xyz[xyz[:, 2] < -0.05]
    if len(below) < 500:
        return np.eye(4), None
    pcd = to_pcd(below, [0, 0, 0])
    plane, inl = pcd.segment_plane(distance_threshold=0.03, ransac_n=3, num_iterations=2000)
    a, b, c, d = plane
    n = np.array([a, b, c])
    n = n / np.linalg.norm(n)
    if n[2] < 0:
        n, d = -n, -d
    tilt = math.degrees(math.acos(min(1.0, abs(n[2]))))
    if tilt > 30 or len(inl) < 300:
        return np.eye(4), {"ok": False, "tilt_deg": tilt, "inliers": len(inl)}
    # rotation taking n -> +z (Rodrigues)
    z = np.array([0, 0, 1.0])
    v = np.cross(n, z)
    s, cth = np.linalg.norm(v), float(np.dot(n, z))
    if s < 1e-9:
        R = np.eye(3)
    else:
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        R = np.eye(3) + vx + vx @ vx * ((1 - cth) / (s * s))
    T = np.eye(4)
    T[:3, :3] = R
    # after rotation the plane is z = -d' ; shift so floor is z=0
    p0 = -d * n                      # a point on the plane
    T[2, 3] = -(R @ p0)[2]
    return T, {"ok": True, "tilt_deg": tilt, "inliers": len(inl), "floor_height_m": float(-(R @ p0)[2]) * -1}


def apply(T, xyz):
    return (xyz @ T[:3, :3].T) + T[:3, 3]


def yaw_T(deg, tx=0.0, ty=0.0):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    T = np.eye(4)
    T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    T[0, 3], T[1, 3] = tx, ty
    return T


def icp(src, dst, init, thresh):
    return o3d.pipelines.registration.registration_icp(
        src, dst, thresh, init, o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=80))


def nn_dist(src_xyz, dst_xyz):
    tree = o3d.geometry.KDTreeFlann(to_pcd(dst_xyz, [0, 0, 0]))
    out = np.empty(len(src_xyz))
    for i, p in enumerate(src_xyz):
        _, _, d2 = tree.search_knn_vector_3d(p, 1)
        out[i] = math.sqrt(d2[0])
    return out


def rot_to_rpy(R):
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    pitch = math.degrees(math.asin(-R[2, 0]))
    roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
    return roll, pitch, yaw


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rec")
    ap.add_argument("--seq-a", type=int, required=True)
    ap.add_argument("--seq-b", type=int, required=True)
    ap.add_argument("--nframes", type=int, default=2)
    ap.add_argument("--voxel", type=float, default=0.04)
    ap.add_argument("--max-range", type=float, default=30.0)
    ap.add_argument("--no-rerun", action="store_true")
    args = ap.parse_args()
    rec = Path(args.rec)
    out = rec / f"calib_A{args.seq_a}_B{args.seq_b}"
    out.mkdir(exist_ok=True)

    A, nA = load_median(rec, UNITS[0], args.seq_a, args.nframes)
    B, nB = load_median(rec, UNITS[1], args.seq_b, args.nframes)
    A = A[np.linalg.norm(A, axis=1) < args.max_range]
    B = B[np.linalg.norm(B, axis=1) < args.max_range]
    print(f"A: {len(A)} pts from {nA} frames (seq {args.seq_a - args.nframes + 1}..{args.seq_a}),  B: {len(B)} pts from {nB} frames")

    # 1) level each unit on its floor
    TlA, infoA = level_on_floor(A)
    TlB, infoB = level_on_floor(B)
    print("floor A:", infoA, " floor B:", infoB)
    Al, Bl = apply(TlA, A), apply(TlB, B)

    # 2) coarse yaw + xy: RANSAC on FPFH features and a brute yaw sweep; keep the best by ICP fitness/RMSE
    va, vb = args.voxel * 1.5, args.voxel * 1.5
    pa = to_pcd(Al, [0.2, 0.5, 1.0]).voxel_down_sample(va)
    pb = to_pcd(Bl, [1.0, 0.5, 0.2]).voxel_down_sample(vb)
    for p in (pa, pb):
        p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=va * 4, max_nn=40))
    fa = o3d.pipelines.registration.compute_fpfh_feature(pa, o3d.geometry.KDTreeSearchParamHybrid(radius=va * 8, max_nn=100))
    fb = o3d.pipelines.registration.compute_fpfh_feature(pb, o3d.geometry.KDTreeSearchParamHybrid(radius=va * 8, max_nn=100))
    candidates = []
    t0 = time.time()
    ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        pb, pa, fb, fa, True, va * 2.5, o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 3,
        [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
         o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(va * 2.5)],
        o3d.pipelines.registration.RANSACConvergenceCriteria(200000, 0.999))
    candidates.append(("ransac_fpfh", ransac.transformation))
    print(f"RANSAC: fitness {ransac.fitness:.3f} rmse {ransac.inlier_rmse:.3f} ({time.time() - t0:.1f}s)")
    for yaw in range(-180, 180, 10):           # yaw sweep with centroid alignment as translation guess
        T = yaw_T(yaw)
        cb = apply(T, np.asarray(pb.points)).mean(axis=0)
        ca = np.asarray(pa.points).mean(axis=0)
        T[0, 3], T[1, 3] = ca[0] - cb[0], ca[1] - cb[1]
        candidates.append((f"yaw{yaw:+d}", T))
    best = None
    for name, T in candidates:
        r1 = icp(pb, pa, T, va * 3)
        r2 = icp(pb, pa, r1.transformation, args.voxel * 1.5)
        score = (r2.fitness, -r2.inlier_rmse)
        if best is None or score > best[0]:
            best = (score, name, r2)
    (fit, nrmse), name, res = best
    print(f"best coarse start: {name} -> ICP fitness {fit:.3f} rmse {-nrmse * 100:.1f} cm")

    # 3) fine ICP on full-res leveled clouds
    pa_f = to_pcd(Al, [0, 0, 0]).voxel_down_sample(args.voxel / 2)
    pb_f = to_pcd(Bl, [0, 0, 0]).voxel_down_sample(args.voxel / 2)
    pa_f.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=args.voxel * 3, max_nn=40))
    fine = icp(pb_f, pa_f, res.transformation, args.voxel)
    fine2 = icp(pb_f, pa_f, fine.transformation, args.voxel / 2)
    T_lev = fine2.transformation                       # B_leveled -> A_leveled
    T_BA = np.linalg.inv(TlA) @ T_lev @ TlB            # B raw -> A raw
    print(f"fine ICP: fitness {fine2.fitness:.3f} rmse {fine2.inlier_rmse * 100:.2f} cm")

    # 4) agreement stats in A's frame
    B_in_A = apply(T_BA, B)
    dBA = nn_dist(B_in_A, A)
    dAB = nn_dist(A, B_in_A)
    tol = 0.05
    stats = {
        "overlap_B_in_A_frac": float((dBA < tol).mean()), "overlap_A_in_B_frac": float((dAB < tol).mean()),
        "rmse_cm_B_to_A_within_5cm": float(np.sqrt(np.mean(dBA[dBA < tol] ** 2)) * 100) if (dBA < tol).any() else None,
        "median_nn_cm_B_to_A": float(np.median(dBA) * 100),
        "before_overlap_frac_identity": float((nn_dist(B, A) < tol).mean()),
    }
    roll, pitch, yaw = rot_to_rpy(T_BA[:3, :3])
    result = {
        "rec": str(rec), "seq_a": args.seq_a, "seq_b": args.seq_b, "nframes": args.nframes,
        "T_B_to_A": T_BA.tolist(), "translation_m": T_BA[:3, 3].tolist(), "roll_pitch_yaw_deg": [roll, pitch, yaw],
        "floor_A": infoA, "floor_B": infoB, "icp_fitness": fine2.fitness, "icp_inlier_rmse_m": fine2.inlier_rmse,
        "coarse_start": name, "agreement": stats, "created": dt.datetime.now().isoformat(),
        "note": "T_B_to_A maps unit_0008 points into unit_0060's frame (x fwd, y left, z up). Scene-based; re-run after any remount.",
    }
    json.dump(result, open(out / "T_B_to_A.json", "w"), indent=2)
    print(f"T_B_to_A: translation ({T_BA[0,3]:+.3f}, {T_BA[1,3]:+.3f}, {T_BA[2,3]:+.3f}) m, roll {roll:+.2f} pitch {pitch:+.2f} yaw {yaw:+.2f} deg")
    print(f"agreement: B->A overlap {stats['overlap_B_in_A_frac']*100:.0f}% within 5 cm (was {stats['before_overlap_frac_identity']*100:.0f}% before), "
          f"A->B {stats['overlap_A_in_B_frac']*100:.0f}%, rmse {stats['rmse_cm_B_to_A_within_5cm']:.2f} cm, median nn {stats['median_nn_cm_B_to_A']:.1f} cm")

    # 5) outputs: clouds
    pA = to_pcd(A, [0.2, 0.5, 1.0]); pB = to_pcd(B_in_A, [1.0, 0.5, 0.2])
    o3d.io.write_point_cloud(str(out / "cloud_A.ply"), pA)
    o3d.io.write_point_cloud(str(out / "cloud_B_in_A.ply"), pB)
    o3d.io.write_point_cloud(str(out / "merged_union.ply"), (pA + pB).voxel_down_sample(0.02))
    cons = np.vstack([A[dAB < tol], B_in_A[dBA < tol]])
    o3d.io.write_point_cloud(str(out / "merged_consensus.ply"), to_pcd(cons, [0.1, 0.8, 0.1]).voxel_down_sample(0.02))
    o3d.io.write_point_cloud(str(out / "disagreement_A_only.ply"), to_pcd(A[dAB >= tol], [0.2, 0.5, 1.0]))
    o3d.io.write_point_cloud(str(out / "disagreement_B_only.ply"), to_pcd(B_in_A[dBA >= tol], [1.0, 0.5, 0.2]))
    print(f"consensus {len(cons)} pts; A-only {(dAB >= tol).sum()}, B-only {(dBA >= tol).sum()}")

    # 6) preview PNG: top-down before / after, and consensus
    fig, axes = plt.subplots(1, 3, figsize=(20, 7), dpi=110)
    for ax, (title, pa_, pb_) in zip(axes, [("before (identity)", A, B), ("after calibration", A, B_in_A), ("consensus (both agree <5 cm)", cons, None)]):
        ax.scatter(-pa_[:, 1], pa_[:, 0], s=0.4, c="tab:blue" if pb_ is not None else "tab:green", linewidths=0, label="unit_0060" if pb_ is not None else "consensus")
        if pb_ is not None:
            ax.scatter(-pb_[:, 1], pb_[:, 0], s=0.4, c="tab:orange", linewidths=0, label="unit_0008")
        ax.set_title(title); ax.set_xlabel("-y [m]"); ax.set_ylabel("x fwd [m]"); ax.set_aspect("equal", adjustable="datalim"); ax.grid(alpha=0.3); ax.legend(markerscale=12, loc="upper right")
    fig.suptitle(f"{rec.name}: A frames ..{args.seq_a}, B frames ..{args.seq_b} | T_B_to_A yaw {yaw:+.1f} deg, t=({T_BA[0,3]:+.2f},{T_BA[1,3]:+.2f},{T_BA[2,3]:+.2f}) m | "
                 f"ICP rmse {fine2.inlier_rmse*100:.1f} cm, overlap {stats['overlap_B_in_A_frac']*100:.0f}%")
    fig.tight_layout(); fig.savefig(out / "preview.png"); print("wrote", out / "preview.png")

    if args.no_rerun:
        return 0
    import rerun as rr, rerun.blueprint as rrb
    sdk_root = Path(rr.__file__).resolve().parent.parent
    viewer = next((p for p in (sdk_root / "rerun_cli" / "rerun.exe",) if p.exists()), None)
    bp = rrb.Blueprint(rrb.Horizontal(
        rrb.Spatial3DView(origin="world", name="calibrated: blue = unit_0060, orange = unit_0008 in A frame", background=rrb.Background(color=[228, 228, 228])),
        rrb.Spatial3DView(origin="world", name="consensus only", background=rrb.Background(color=[228, 228, 228]),
                          contents=["+ $origin/consensus/**", "+ $origin/axes/**"]), ), collapse_panels=True)
    rr.init("ac1_calib_" + out.name, default_blueprint=bp)
    rr.spawn(executable_path=str(viewer) if viewer else None, default_blueprint=bp, memory_limit="1GiB", server_memory_limit="256MiB", connect=False)
    rr.set_sinks(rr.GrpcSink(), rr.FileSink(str(out / "calib.rrd")), default_blueprint=bp)
    rr.send_blueprint(bp, make_active=True, make_default=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/axes", rr.Arrows3D(origins=np.zeros((3, 3)), vectors=np.eye(3) * 0.3, colors=[[255, 60, 60], [60, 255, 60], [60, 60, 255]]), static=True)
    rr.log("world/unit_0060", rr.Points3D(positions=A, colors=[60, 120, 255], radii=rr.Radius.ui_points(2.0)), static=True)
    rr.log("world/unit_0008_in_A", rr.Points3D(positions=B_in_A, colors=[255, 140, 40], radii=rr.Radius.ui_points(2.0)), static=True)
    rr.log("world/unit_0008_origin", rr.Transform3D(translation=T_BA[:3, 3], mat3x3=T_BA[:3, :3]), static=True)
    rr.log("world/unit_0008_origin/axes", rr.Arrows3D(origins=np.zeros((3, 3)), vectors=np.eye(3) * 0.3, colors=[[255, 60, 60], [60, 255, 60], [60, 60, 255]], labels=["unit_0008", "", ""]), static=True)
    rr.log("world/consensus", rr.Points3D(positions=cons, colors=[30, 170, 60], radii=rr.Radius.ui_points(2.0)), static=True)
    rr.log("info", rr.TextLog(json.dumps({k: result[k] for k in ("translation_m", "roll_pitch_yaw_deg", "icp_fitness", "agreement")})), static=True)
    time.sleep(1.0)
    rr.disconnect()
    print("opened in Rerun as", "ac1_calib_" + out.name)
    return 0


if __name__ == "__main__":
    sys.exit(main())
