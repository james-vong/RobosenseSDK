# RoboSense AC1 SDK: patched rs_driver + tools (Windows and Linux)

A working Windows build of RoboSense's `rs_driver` (branch `dev_opt_AC1`) for the AC1 Active Camera, with the
fixes we needed to make it run on a laptop, plus the capture / viewer tools built on it. Everything here was
verified on Windows 11 with two AC1 units (serials 1111BFAF0060 and 1111BFA90008) over USB 2.0 and USB-C.

If the stock driver "refuses" on your laptop, it is almost always one of the four things in **What is different**.

## Quick start

1. **Driver binding (one time).** rs_driver talks to the AC1 through libusb, so all three AC1 USB interfaces must be
   bound to **WinUSB**, not Windows' own camera/HID drivers. Install RoboSense AcViewer and run
   `C:\Program Files (x86)\st\AcViewer\Driver\AC_driver_install.bat` as administrator (it uses libwdi), or use
   Zadig to bind WinUSB to `USB\VID_3840&PID_1010&MI_00`, `MI_02` and `MI_04`. Check with
   `pnputil /enum-devices /instanceid "<id>" /stack`: the stack must say `WINUSB`.
2. **Power.** The AC1 needs its own 12 V supply (12.6 W) in addition to USB. Without it the unit never enumerates.
3. **Cable.** USB-C does not imply USB 3. On a USB 2.0 link (480 Mb/s) the LiDAR and IMU run at full rate; the
   camera works at 1080p 10 fps or 640x480 30 fps, but 1080p 30 fps tears. Check with `tools\probe_ac1.py`.
4. **Run a prebuilt demo** (no build needed):
   ```
   prebuilt\demo_ac1_dump.exe --usb200 --seconds 10 --out captures\test
   ```
   `--usb200` lets the driver start on a USB 2.0 link (it refuses otherwise). Omit on USB 3.


## Linux (Debian 13 arm64 verified on the Amlogic MB41, 2026-11-18)

No driver binding step: libusb opens the device directly. Needs `gcc g++ cmake make libudev-dev`.
```
sudo cp 99-robosense-ac1.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules   # open AC1 without sudo
./build_linux.sh                                   # -> rs_driver/build/demo/demo_usb, demo_ac1_dump, demo_ac1_stream
rs_driver/build/demo/demo_ac1_dump --usb200 --seconds 10 --out captures/test
```
`demo_ac1_stream` serves frames over TCP (`--port`, `--uuid`), so a Python viewer on another machine can attach;
bind is 127.0.0.1 by default. Check enumeration with `lsusb` (look for `3840:1010`) and `sudo dmesg | grep usb`.
"Cannot enable. Maybe the USB cable is bad?" from the kernel means the port never completed link training: try
the USB 3 port, a different cable, and confirm the AC1 has its 12 V supply.

## What is different from upstream (RoboSense-LiDAR/rs_driver @ dev_opt_AC1, see UPSTREAM_COMMIT.txt)

Full diff in `CHANGES_FROM_UPSTREAM.patch` (64 lines). In words:

1. **`src/rs_driver/driver/input/input_usb.cpp`, `findDevices()`** — with two AC1s connected, upstream returns
   "Device is not found" as soon as the enumeration meets a unit whose serial differs from the requested
   `device_uuid`, even after it already matched the right one. Patched to skip non-matching units and stop at the
   first match. Single-unit behaviour is unchanged.
2. **`demo/CMakeLists.txt`** — registers two extra demos built from `tools/`: `demo_ac1_dump` (saves clouds as PLY,
   images as raw NV12, IMU as CSV) and `demo_ac1_stream` (serves frames to any number of viewers over localhost TCP,
   selects a unit by `--uuid`, used by the Python viewers).
3. **No change to the driver's USB 2.0 refusal**, but every tool passes `enable_usb200` (`--usb200`) because the
   check in `input_usb.cpp` otherwise aborts init on a USB 2.0 link.
4. **Not a code change, but required:** WinUSB binding (step 1 above). The stock driver gives
   `LIBUSB_ERROR_NOT_SUPPORTED` / "Device is not found" when Windows' usbvideo driver holds the interfaces.

Everything else in `rs_driver/` is upstream as cloned (vendored libusb + libuvc fork included).

## Building from source

Visual Studio 2022 Build Tools (MSVC 14.4x, with the C++ CMake tools component) is all that is needed:
```
build_rs_driver.cmd     :: configure (ENABLE_USB=ON, COMPILE_DEMOS=ON, DISABLE_PCAP_PARSE=ON) + build with Ninja
rebuild.cmd             :: incremental
```
Outputs land in `rs_driver\build\demo\`. The scripts assume the Build Tools default install path; edit the two
paths at the top if yours differs.

## Tools (Python 3.11+; `pip install pyusb libusb-package numpy opencv-python pyyaml matplotlib rerun-sdk open3d`)

| Tool | What |
|---|---|
| `prebuilt-linux-arm64/` | Same three demos built on the MB41 (Debian 13 arm64); `chmod +x` after checkout. |
| `tools/probe_ac1.py` | Enumerate AC1s via libusb: serial, negotiated USB speed, per-interface claim test. First thing to run. |
| `tools/live_view.py` | OpenCV live window: camera, LiDAR front view, top-down map, IMU. |
| `tools/rerun_view.py` | Live Rerun viewer, points colored from the camera; `--uuid` to pick a unit. |
| `tools/dual_view.py` | Two units in one Rerun recording with on-disk recording (npy clouds, jpgs, timestamp CSV). |
| `tools/freeze_frame.py`, `show_freeze.py` | One synchronized cloud + image, colored, saved and shown in Rerun. |
| `tools/calibrate_dual.py` | Scene-based extrinsic calibration between two units (Open3D). |
| `tools/ac1_calib.py` | Reads RoboSense `calibration.yaml`, projects points into the camera. |
| `tools/overlay_view.py`, `raw_to_png.py`, `nv12_stats.py`, `ply_preview.py`, `uvc_formats.py`, `grab_frame.py`, `frame_stats.py` | Utilities. |

`calib/calibration_sample_1111BEAC0003.yaml` is the calibration file AcViewer ships (another unit); use your unit's
own file from the device when you have it.

## Facts worth knowing

- Sensor frame: x forward, y left, z up. The 288x96 point grid is 144 elevation lines x 2 azimuth windows
  (even rows = left 60°, odd rows = right 60°); do not reshape it as an image, project by angle.
- The first 1 to 2 frames after start are a stale buffer (2015 timestamp, junk ranges); the tools drop them.
- Requesting a camera fps the device does not advertise (anything but 30 / 15 / 10) crashes the driver.
- Measured: 0.3 cm range jitter below 8 m, 24.35 m plane fit vs 24.5 m laser meter, returns to 70 m on a bright
  building outdoors, nothing usable through a low-e window.

Licence: rs_driver is BSD-3 (RoboSense); the tools in `tools/` are provided as-is.
