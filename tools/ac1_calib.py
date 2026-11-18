"""RoboSense AC1 calibration: camera intrinsics (OpenCV rational distortion), camera<->LiDAR extrinsics,
and projection of LiDAR points into the camera image at any stream resolution.

Calibration file format = RoboSense `calibration.yaml` (as shipped in AcViewer's StreamingAssets/Config and as
read from the device by the ROS driver). Search order: calib/calibration_<device>.yaml, any calib/calibration*.yaml,
then AcViewer's shipped sample. Drop this unit's own file into calib/ when you get it; the sample is for unit
1111BEAC0003 and this laptop's unit is 1111BFAF0060 (small unit-to-unit differences expected).

Frames: LiDAR/sensor = x forward, y left, z up.  Camera = OpenCV (x right, y down, z forward).
"""
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
CALIB_DIR = ROOT / "calib"
ACVIEWER_SAMPLE = Path(r"C:\Program Files (x86)\st\AcViewer\AcViewer_Data\StreamingAssets\Config\calibration.yaml")
THIS_UNIT = "1111BFAF0060"


def quat_to_R(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def find_default():
    for cand in (CALIB_DIR / f"calibration_{THIS_UNIT}.yaml", CALIB_DIR / f"calibration_{THIS_UNIT.lower()}.yaml"):
        if cand.exists():
            return cand
    found = sorted(CALIB_DIR.glob("calibration*.yaml")) if CALIB_DIR.exists() else []
    if found:
        return found[0]
    if ACVIEWER_SAMPLE.exists():
        return ACVIEWER_SAMPLE
    raise FileNotFoundError("no AC1 calibration.yaml found (looked in calib/ and the AcViewer install)")


class Ac1Calib:
    def __init__(self, path=None):
        self.path = Path(path) if path else find_default()
        with open(self.path, "r", encoding="utf-8") as f:
            d = yaml.safe_load(f)
        self.device_id = str(d.get("DEVICE_ID", "?"))
        cam = d["Sensor"]["Camera"]
        intr = cam["intrinsic"]
        self.K_native = np.array(intr["int_matrix"], float).reshape(3, 3)
        dist = np.array(intr.get("dist_coeff", []), float)
        self.dist = dist[:8] if len(dist) >= 8 else np.pad(dist, (0, 8 - len(dist)))  # k1 k2 p1 p2 k3 k4 k5 k6
        self.native_size = tuple(int(v) for v in intr.get("image_size", (1920, 1080)))
        t, q = cam["extrinsic"]["translation"], cam["extrinsic"]["quaternion"]
        self.t_lc = np.array([t["x"], t["y"], t["z"]], float)        # camera origin, in LiDAR frame
        self.R_lc = quat_to_R(q["x"], q["y"], q["z"], q["w"])        # camera axes -> LiDAR frame
        lid = d["Sensor"].get("Lidar", {}).get("extrinsic", {})
        lt = lid.get("translation", {"x": 0, "y": 0, "z": 0})
        lq = lid.get("quaternion", {"x": 0, "y": 0, "z": 0, "w": 1})
        self.lidar_extrinsic_is_identity = (
            abs(lt["x"]) + abs(lt["y"]) + abs(lt["z"]) < 1e-9 and abs(lq["w"] - 1) < 1e-9)
        self.R_cl = self.R_lc.T
        self.t_cl = -self.R_cl @ self.t_lc                             # p_cam = R_cl @ p_lidar + t_cl
        imu = d["Sensor"].get("IMU", {}).get("extrinsic", {})
        it = imu.get("translation", {"x": 0, "y": 0, "z": 0})
        self.t_li = np.array([it["x"], it["y"], it["z"]], float)

    def K_for(self, w, h, mode="squash"):
        """Intrinsics for a w x h stream. 'squash': native frame scaled anisotropically to w x h.
        'crop': native frame center-cropped to the target aspect ratio, then scaled."""
        W, H = self.native_size
        K = self.K_native.copy()
        if mode == "crop":
            target, native = w / h, W / H
            if native > target:
                cw, ch, ox, oy = H * target, H, (W - H * target) / 2, 0.0
            else:
                cw, ch, ox, oy = W, W / target, 0.0, (H - W / target) / 2
            s = w / cw
            K[0, 0] *= s
            K[1, 1] *= s
            K[0, 2] = (K[0, 2] - ox) * s
            K[1, 2] = (K[1, 2] - oy) * s
        else:
            sx, sy = w / W, h / H
            K[0, 0] *= sx
            K[0, 2] *= sx
            K[1, 1] *= sy
            K[1, 2] *= sy
        return K

    def to_camera(self, xyz):
        return xyz @ self.R_cl.T + self.t_cl

    def project(self, xyz, w, h, mode="squash", max_angle_deg=68.0):
        """Project LiDAR-frame points (n,3) into a w x h image.
        Returns uv (n,2) float (NaN where not projectable), in_image mask (n,), depth along the optical axis (n,)."""
        xyz = np.asarray(xyz, float)
        p = self.to_camera(xyz)
        z = p[:, 2]
        ang = np.degrees(np.arctan2(np.hypot(p[:, 0], p[:, 1]), z))
        ok = np.isfinite(p).all(axis=1) & (z > 0.05) & (ang < max_angle_deg)
        uv = np.full((len(xyz), 2), np.nan)
        if ok.any():
            pts, _ = cv2.projectPoints(p[ok].reshape(-1, 1, 3), np.zeros(3), np.zeros(3), self.K_for(w, h, mode), self.dist)
            uv[ok] = pts.reshape(-1, 2)
        inb = ok & np.isfinite(uv).all(axis=1) & (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
        return uv, inb, z

    def describe(self):
        K = self.K_native
        return (f"calib {self.path.name} (device {self.device_id}{'' if self.device_id.upper() == THIS_UNIT else ', NOT this unit'}): "
                f"native {self.native_size[0]}x{self.native_size[1]} fx={K[0,0]:.1f} fy={K[1,1]:.1f} cx={K[0,2]:.1f} cy={K[1,2]:.1f}, "
                f"cam pos in lidar frame = ({self.t_lc[0]:+.4f}, {self.t_lc[1]:+.4f}, {self.t_lc[2]:+.4f}) m")


if __name__ == "__main__":
    c = Ac1Calib()
    print(c.describe())
    print("R_lc (camera axes in lidar frame, columns = cam x,y,z):\n", np.round(c.R_lc, 4))
    with open(c.path, "r", encoding="utf-8") as f:
        add = yaml.safe_load(f).get("ADDTION_INFO", {})
    if "Rcl" in add:
        Rcl = np.array(add["Rcl"], float).reshape(3, 3)
        print("max |R_cl - file Rcl| =", np.abs(c.R_cl - Rcl).max())
        print("t_cl =", np.round(c.t_cl, 4), " file Pcl =", add.get("Pcl"))
    for mode in ("squash", "crop"):
        print(f"K for 640x480 ({mode}):\n", np.round(c.K_for(640, 480, mode), 2))
    test = np.array([[2.0, 0.0, 0.0], [2.0, 1.0, 0.0], [2.0, -1.0, 0.0], [2.0, 0.0, 0.5], [2.0, 0.0, -0.5]])
    uv, inb, z = c.project(test, 640, 480)
    for p_, u_, i_ in zip(test, uv, inb):
        print(f"  lidar {p_} -> pixel ({u_[0]:7.1f}, {u_[1]:7.1f}) in_image={i_}")
