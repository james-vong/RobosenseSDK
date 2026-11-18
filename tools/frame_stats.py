"""Robust geometry stats of a full-grid AC1 cloud (.npy from grab_frame.py) to pin down the sensor frame."""
import sys, warnings
import numpy as np
p = sys.argv[1]
pts = np.load(p)
gh, gw = 288, 96
v = pts[np.isfinite(pts[:, :3]).all(axis=1) & (np.abs(pts[:, :3]).sum(axis=1) > 0), :3]
x, y, z = v.T
r = np.linalg.norm(v, axis=1)
far = r > 0.8
print(f"{p}: valid {len(v)}  (range>0.8m: {far.sum()})")
for name, a in (("x", x), ("y", y), ("z", z), ("range", r)):
    q = np.percentile(a, [1, 5, 50, 95, 99])
    print(f"  {name:5s} p1={q[0]:6.2f} p5={q[1]:6.2f} p50={q[2]:6.2f} p95={q[3]:6.2f} p99={q[4]:6.2f}")
# angular extents (range>0.8m) under 'x forward'
ang_y = np.degrees(np.arctan2(y, x))                 # angle in x-y plane
ang_z = np.degrees(np.arctan2(z, np.hypot(x, y)))    # angle toward z
for name, a in (("angle in x-y plane (toward +y)", ang_y[far]), ("angle toward +z", ang_z[far])):
    q = np.percentile(a, [1, 5, 95, 99])
    print(f"  {name:32s} p1={q[0]:6.1f} p5={q[1]:6.1f} p95={q[2]:6.1f} p99={q[3]:6.1f}  width(p1-p99)={q[3]-q[0]:.1f} deg")
# the strong plane near z=0: where is it?
m = np.abs(z) < 0.07
print(f"  plane |z|<7cm: {m.sum()} pts ({100*m.mean():.1f}%)  x p5..p95 = {np.percentile(x[m],5):.2f}..{np.percentile(x[m],95):.2f}  y p5..p95 = {np.percentile(y[m],5):.2f}..{np.percentile(y[m],95):.2f}")
for lo, hi in ((0.1, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 4.0), (4.0, 12.0)):
    mm = (x >= lo) & (x < hi)
    if mm.any():
        print(f"    x in [{lo},{hi}): {mm.sum():5d} pts, fraction with |z|<7cm = {100*np.mean(np.abs(z[mm])<0.07):5.1f}%,  y p5..p95 = {np.percentile(y[mm],5):.2f}..{np.percentile(y[mm],95):.2f},  z p5..p95 = {np.percentile(z[mm],5):.2f}..{np.percentile(z[mm],95):.2f}")
# per-row / per-col sweep using beam directions (unit vectors), robust to scene content
g = pts[:, :3].reshape(gh, gw, 3)
n = g / np.linalg.norm(g, axis=2, keepdims=True)
with warnings.catch_warnings():
    warnings.simplefilter("ignore", RuntimeWarning)
    row_dir = np.nanmean(n, axis=1)   # (288,3) mean beam direction per row
    col_dir = np.nanmean(n, axis=0)   # (96,3)  mean beam direction per col
def show(label, d):
    ok = np.isfinite(d).all(axis=1)
    d = d[ok]
    ay = np.degrees(np.arctan2(d[:, 1], d[:, 0])); az = np.degrees(np.arctan2(d[:, 2], np.hypot(d[:, 0], d[:, 1])))
    print(f"  {label}: n={len(d)}  angle-toward-y first/last = {ay[0]:6.1f}/{ay[-1]:6.1f} (span {ay[-1]-ay[0]:6.1f})   angle-toward-z first/last = {az[0]:6.1f}/{az[-1]:6.1f} (span {az[-1]-az[0]:6.1f})")
show("rows 0..287", row_dir)
show("cols 0..95 ", col_dir)
