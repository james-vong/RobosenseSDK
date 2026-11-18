"""Probe the RoboSense AC1 over libusb (same access path rs_driver dev_opt_AC1 uses).

Reports descriptors, negotiated USB speed, interfaces/endpoints, and whether each
interface can be opened + claimed from user space via WinUSB/libusb.
"""
import sys
import usb.core
import usb.util
import libusb_package

VID, PID = 0x3840, 0x1010
SPEED = {0: "unknown", 1: "LOW (1.5 Mb/s)", 2: "FULL (12 Mb/s)", 3: "HIGH (480 Mb/s)",
         4: "SUPER (5 Gb/s)", 5: "SUPER+ (10 Gb/s)"}
CLS = {0x03: "HID", 0x0E: "Video (UVC)", 0xFF: "Vendor"}
SUB_VIDEO = {0x01: "VideoControl", 0x02: "VideoStreaming", 0x03: "InterfaceCollection"}
EPTYPE = {0: "CTRL", 1: "ISOC", 2: "BULK", 3: "INTR"}

backend = libusb_package.get_libusb1_backend()
print(f"libusb backend: {backend}")
devs = list(usb.core.find(find_all=True, idVendor=VID, backend=backend))
print(f"devices with VID 0x{VID:04x}: {len(devs)}")
if not devs:
    print("RESULT: AC1 NOT visible to libusb (no interface bound to WinUSB/libusb-compatible driver)")
    sys.exit(2)

rc = 0
for d in devs:
    print("=" * 70)
    print(f"VID:PID 0x{d.idVendor:04x}:0x{d.idProduct:04x}  bcdDevice 0x{d.bcdDevice:04x}  bcdUSB 0x{d.bcdUSB:04x}")
    print(f"bus {d.bus} addr {d.address} ports {d.port_numbers}  speed: {SPEED.get(d.speed, d.speed)}")
    print(f"bDeviceClass 0x{d.bDeviceClass:02x}  bNumConfigurations {d.bNumConfigurations}  bMaxPacketSize0 {d.bMaxPacketSize0}")
    for name in ("manufacturer", "product", "serial_number"):
        try:
            print(f"  {name}: {getattr(d, name)}")
        except Exception as e:  # string reads need an open handle; report but continue
            print(f"  {name}: <read failed: {e}>")
    for cfg in d:
        print(f"config {cfg.bConfigurationValue}: {cfg.bNumInterfaces} interfaces, maxpower {cfg.bMaxPower*2} mA")
        for intf in cfg:
            cls = CLS.get(intf.bInterfaceClass, f"0x{intf.bInterfaceClass:02x}")
            sub = SUB_VIDEO.get(intf.bInterfaceSubClass, f"0x{intf.bInterfaceSubClass:02x}") if intf.bInterfaceClass == 0x0E else f"0x{intf.bInterfaceSubClass:02x}"
            print(f"  intf {intf.bInterfaceNumber} alt {intf.bAlternateSetting}: {cls}/{sub} proto 0x{intf.bInterfaceProtocol:02x}  eps={intf.bNumEndpoints}")
            for ep in intf:
                t = EPTYPE[usb.util.endpoint_type(ep.bmAttributes)]
                dirn = "IN " if usb.util.endpoint_direction(ep.bEndpointAddress) == usb.util.ENDPOINT_IN else "OUT"
                print(f"      ep 0x{ep.bEndpointAddress:02x} {dirn} {t:4s} wMaxPacketSize {ep.wMaxPacketSize}")
    # Try to claim every interface (alt 0) -- this is what rs_driver/libuvc do.
    print("-- claim test (exclusive access; fails if AcViewer or another process holds it)")
    cfg = d.get_active_configuration()
    seen = set()
    for intf in cfg:
        n = intf.bInterfaceNumber
        if n in seen:
            continue
        seen.add(n)
        try:
            usb.util.claim_interface(d, n)
            print(f"  intf {n}: CLAIM OK")
            usb.util.release_interface(d, n)
        except Exception as e:
            print(f"  intf {n}: CLAIM FAILED -> {e}")
            rc = 1
    usb.util.dispose_resources(d)
print("RESULT:", "all interfaces claimable" if rc == 0 else "some interfaces not claimable (see above)")
sys.exit(rc)
