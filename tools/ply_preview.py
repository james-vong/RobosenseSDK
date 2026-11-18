"""Render a quick PNG preview of an AC1 point cloud PLY written by demo_ac1_dump.

Usage: ply_preview.py <cloud.ply> [out.png]
Produces a 3-panel figure: top-down (x,y), front (y,z), and side (x,z), colored by range.
"""
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def read_ascii_ply(path: Path):
    with open(path, "r") as f:
        header = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError("no end_header")
            header.append(line.strip())
            if line.strip() == "end_header":
                break
        n = next(int(h.split()[-1]) for h in header if h.startswith("element vertex"))
        props = [h.split()[2] for h in header if h.startswith("property")]
        data = np.loadtxt(f, max_rows=n) if n > 0 else np.zeros((0, len(props)))
    data = data.reshape(-1, len(props))
    return {p: data[:, i] for i, p in enumerate(props)}, header


def main(argv):
    if not argv:
        print(__doc__)
        return 1
    src = Path(argv[0])
    dst = Path(argv[1]) if len(argv) > 1 else src.with_suffix(".png")
    cols, header = read_ascii_ply(src)
    x, y, z = cols["x"], cols["y"], cols["z"]
    r = np.sqrt(x * x + y * y + z * z)
    comment = next((h[len("comment "):] for h in header if h.startswith("comment")), "")
    print(f"{src.name}: {len(x)} points, range {r.min():.2f}..{r.max():.2f} m, "
          f"x {x.min():.2f}..{x.max():.2f}, y {y.min():.2f}..{y.max():.2f}, z {z.min():.2f}..{z.max():.2f}")

    fig, axes = plt.subplots(1, 3, figsize=(18, 6), dpi=110)
    views = [("top-down: x (fwd) vs y (left)", x, y, "x [m]", "y [m]"),
             ("front: y (left) vs z (up)", y, z, "y [m]", "z [m]"),
             ("side: x (fwd) vs z (up)", x, z, "x [m]", "z [m]")]
    for ax, (title, a, b, la, lb) in zip(axes, views):
        sc = ax.scatter(a, b, c=r, s=1.0, cmap="viridis", vmin=0, vmax=max(1.0, np.percentile(r, 99)), linewidths=0)
        ax.set_title(title)
        ax.set_xlabel(la)
        ax.set_ylabel(lb)
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, alpha=0.3)
    fig.colorbar(sc, ax=axes, label="range [m]", shrink=0.8)
    fig.suptitle(f"RoboSense AC1 point cloud  |  {len(x)} valid points  |  {comment}", fontsize=11)
    fig.savefig(dst, bbox_inches="tight")
    print(f"wrote {dst}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
