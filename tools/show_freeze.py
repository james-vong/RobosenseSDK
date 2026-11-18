"""Show EVERY returned point of a saved freeze frame, three ways, on a light background in Rerun, plus a PNG.

Panels: camera colors with brightness boosted (dark pixels lifted so they stay visible), log-range colors,
and LiDAR intensity. Points outside the camera's view get full-brightness range colors instead of dimmed ones.
Also writes <folder>/all_points.png (matplotlib, top-down + front view x 3 colorings) and prints how many grid
cells had no return.

Usage: show_freeze.py [freeze_folder]   (default: newest captures/freeze_*)
       show_freeze.py ... --no-rerun     (PNG + stats only)
"""
import json
import sys
from pathlib import Path

import cv2
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ac1_calib import Ac1Calib  # noqa: E402
from live_view import ROOT, range_colors  # noqa: E402


def boost(rgb, floor=70, gamma=0.6):
    """Lift dark camera colors so they are visible on a light/dark background without losing hue."""
    c = (255.0 * (rgb / 255.0) ** gamma)
    lum = c.max(axis=1, keepdims=True)
    lift = np.clip((floor - lum) / max(floor, 1), 0, 1)          # 0 for bright points, up to 1 for black ones
    c = c * (1 - lift) + np.array([150, 150, 150]) * lift         # blend near-black toward mid gray
    return np.clip(c, 0, 255).astype(np.uint8)


def main(argv):
    no_rerun = "--no-rerun" in argv
    args = [a for a in argv if not a.startswith("--")]
    folder = Path(args[0]) if args else sorted((ROOT / "captures").glob("freeze_*"))[-1]
    pts = np.load(folder / "cloud_grid.npy")
    img = cv2.imread(str(folder / "image.png"))
    meta = json.load(open(folder / "meta.json"))
    H, W = img.shape[:2]
    calib = Ac1Calib(meta.get("calib"))
    mode = meta.get("intrinsics_mode", "squash")

    xyz_all = pts[:, :3]
    ok = np.isfinite(xyz_all).all(axis=1) & (np.abs(xyz_all).sum(axis=1) > 0)
    xyz, inten = xyz_all[ok], pts[ok, 3]
    rng = np.linalg.norm(xyz, axis=1)
    uv, inb, _ = calib.project(xyz, W, H, mode)
    rgb_raw = np.zeros((len(xyz), 3), np.uint8)
    u = np.clip(uv[inb, 0].astype(int), 0, W - 1)
    v = np.clip(uv[inb, 1].astype(int), 0, H - 1)
    rgb_raw[inb] = img[v, u][:, ::-1]
    col_range = range_colors(rng, 50.0)[:, ::-1]
    col_rgb = boost(rgb_raw.astype(float))
    col_rgb[~inb] = col_range[~inb]                                 # outside camera: full-brightness range color
    ival = np.clip(inten / max(np.percentile(inten, 99), 1), 0, 1)
    col_int = (np.stack([ival, ival, ival], axis=1) * 215 + 40).astype(np.uint8)
    dark = (rgb_raw[inb].max(axis=1) < 30).sum()
    g = ok.reshape(288, 96)
    print(f"{folder.name}: grid {len(pts)} cells, returned {ok.sum()} ({100 * ok.mean():.1f}%), no return {(~ok).sum()}")
    print(f"  in camera view {inb.sum()}, outside {(~inb).sum()}, painted near-black {dark} -> all shown with boosted/range colors")
    print(f"  no-return by scan-line band bottom->top: " + ", ".join(f"{100 * (1 - g[a:a + 48].mean()):.0f}%" for a in range(0, 288, 48)))
    print(f"  range p50 {np.percentile(rng, 50):.2f} m, max {rng.max():.2f} m")

    # ---- return map: 144 elevation lines x 192 azimuth columns (even grid rows = left 60 deg, odd = right 60 deg)
    rng_grid = np.full(27648, np.nan)
    rng_grid[ok] = rng
    rg = rng_grid.reshape(288, 96)
    rmap = np.full((144, 192), np.nan)
    rmap[:, :96] = rg[0::2]            # even rows: azimuth +60 -> 0 (left half), columns left -> center
    rmap[:, 96:] = rg[1::2]            # odd rows:  azimuth 0 -> -60 (right half), center -> right
    rmap = rmap[::-1]                  # row 0 = bottom scan line -> put top of the field at the top
    empty = ~np.isfinite(rmap)
    print(f"  return map 144x192: {int((~empty).sum())} returns, {int(empty.sum())} empty cells; "
          f"empty by quadrant (TL,TR,BL,BR): " + ", ".join(f"{100 * empty[a:a + 72, b:b + 96].mean():.0f}%" for a in (0, 72) for b in (0, 96)))

    # ---- PNG: return map on top, then top-down + front view, three colorings
    fig = plt.figure(figsize=(18, 14), dpi=110)
    gs = fig.add_gridspec(3, 3, height_ratios=[1.3, 1, 1])
    axm = fig.add_subplot(gs[0, :])
    shown = np.log(np.clip(np.nan_to_num(rmap, nan=0.2), 0.2, 50) / 0.2) / np.log(250)
    im = axm.imshow(np.ma.masked_where(empty, shown), cmap="turbo_r", aspect="auto", vmin=0, vmax=1, interpolation="nearest",
                    extent=[60, -60, -45, 45])
    axm.imshow(np.ma.masked_where(~empty, np.ones_like(shown)), cmap=matplotlib.colors.ListedColormap(["magenta"]), aspect="auto",
               interpolation="nearest", extent=[60, -60, -45, 45])
    axm.set_title(f"return map: every grid cell (144 scan lines x 192 columns = {rmap.size}); magenta = no return ({int(empty.sum())} cells), "
                  f"color = range (red near, blue far)")
    axm.set_xlabel("azimuth [deg]  (+ = left)")
    axm.set_ylabel("elevation [deg]")
    cb = fig.colorbar(im, ax=axm, fraction=0.02, pad=0.01)
    cb.set_ticks([0, np.log(5) / np.log(250), np.log(50) / np.log(250), np.log(250) / np.log(250)])
    cb.set_ticklabels(["0.2 m", "1 m", "10 m", "50 m"])
    axes = np.array([[fig.add_subplot(gs[1, j]) for j in range(3)], [fig.add_subplot(gs[2, j]) for j in range(3)]])
    sets = [("camera colors (boosted; outside-camera = range)", col_rgb), ("range colors (log, 0.2-50 m)", col_range), ("LiDAR intensity", col_int)]
    for j, (title, col) in enumerate(sets):
        c = col / 255.0
        ax = axes[0, j]
        ax.scatter(-xyz[:, 1], xyz[:, 0], c=c, s=0.6, linewidths=0)
        ax.set_title(f"top-down: {title}")
        ax.set_xlabel("-y  (left <- -> right) [m]")
        ax.set_ylabel("x forward [m]")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, alpha=0.3)
        ax = axes[1, j]
        ax.scatter(-xyz[:, 1], xyz[:, 2], c=c, s=0.6, linewidths=0)
        ax.set_title("front view (from sensor)")
        ax.set_xlabel("-y [m]")
        ax.set_ylabel("z up [m]")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, alpha=0.3)
    fig.suptitle(f"{folder.name}: all {ok.sum()} returned points of {len(pts)} grid cells "
                 f"({(~ok).sum()} no return, {int((~inb).sum())} outside camera view)", fontsize=12)
    fig.tight_layout()
    png = folder / "all_points.png"
    fig.savefig(png)
    print("wrote", png)

    if no_rerun:
        return 0
    import rerun as rr
    import rerun.blueprint as rrb
    sdk_root = Path(rr.__file__).resolve().parent.parent
    viewer = next((p for p in (sdk_root / "rerun_cli" / "rerun.exe", sdk_root / "rerun_cli" / "rerun") if p.exists()), None)
    bg = rrb.Background(color=[228, 228, 228])

    def view(name, ent):
        return rrb.Spatial3DView(origin="world", name=name, background=bg,
                                 contents=["+ $origin/" + ent + "/**", "+ $origin/camera/**", "+ $origin/sensor_axes/**"])

    bp = rrb.Blueprint(
        rrb.Vertical(
            rrb.Horizontal(view("all points: camera colors (boosted)", "all_rgb"),
                           view("all points: range colors", "all_range"),
                           view("all points: LiDAR intensity", "all_intensity")),
            rrb.Horizontal(rrb.Spatial2DView(origin="camera/image", name="camera"),
                           rrb.Spatial2DView(origin="grid/return_map", name="return map (magenta = no return)")),
            row_shares=[3, 1.2],
        ),
        collapse_panels=True,
    )
    rr.init("ac1_all_" + folder.name, default_blueprint=bp)
    rr.spawn(executable_path=str(viewer) if viewer else None, default_blueprint=bp, memory_limit="1GiB",
             server_memory_limit="256MiB", connect=False)
    rr.set_sinks(rr.GrpcSink(), rr.FileSink(str(folder / "all_points.rrd")), default_blueprint=bp)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/sensor_axes", rr.Arrows3D(origins=np.zeros((3, 3)), vectors=np.eye(3) * 0.3,
                                            colors=[[255, 60, 60], [60, 255, 60], [60, 60, 255]], labels=["x fwd", "y left", "z up"]), static=True)
    rad = rr.Radius.ui_points(3.0)
    rr.log("world/all_rgb", rr.Points3D(positions=xyz, colors=col_rgb, radii=rad), static=True)
    rr.log("world/all_range", rr.Points3D(positions=xyz, colors=col_range, radii=rad), static=True)
    rr.log("world/all_intensity", rr.Points3D(positions=xyz, colors=col_int, radii=rad), static=True)
    rr.log("world/camera", rr.Transform3D(translation=calib.t_lc, mat3x3=calib.R_lc), static=True)
    rr.log("world/camera/frustum", rr.Pinhole(image_from_camera=calib.K_for(W, H, mode), resolution=[W, H],
                                              camera_xyz=rr.ViewCoordinates.RDF, image_plane_distance=0.5), static=True)
    rr.log("camera/image", rr.Image(img, color_model="BGR").compress(jpeg_quality=90), static=True)
    # return map as an image entity (magenta = empty cell), upscaled 4x for legibility
    rm_rgb = np.zeros((144, 192, 3), np.uint8)
    rm_rgb[~empty] = range_colors(rmap[~empty], 50.0)[:, ::-1]
    rm_rgb[empty] = [255, 0, 255]
    rr.log("grid/return_map", rr.Image(cv2.resize(rm_rgb, (192 * 4, 144 * 4), interpolation=cv2.INTER_NEAREST), color_model="RGB"), static=True)
    rr.log("info", rr.TextLog(f"{folder.name}: {ok.sum()} returned points shown in every panel; {(~ok).sum()} cells had no return "
                              f"(magenta in grid/return_map)"), static=True)
    import time
    time.sleep(1.0)
    rr.disconnect()
    print("opened in Rerun as recording", "ac1_all_" + folder.name)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
