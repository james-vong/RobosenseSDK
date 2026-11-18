"""Dump the UVC VideoStreaming format/frame descriptors the AC1 advertises (no interface claim)."""
import struct
import usb.core
import libusb_package

VID, PID = 0x3840, 0x1010
FMT = {0x04: "UNCOMPRESSED", 0x06: "MJPEG", 0x10: "FRAME_BASED", 0x0A: "STREAM_BASED"}
FRM = {0x05: "FRAME_UNCOMPRESSED", 0x07: "FRAME_MJPEG", 0x11: "FRAME_FRAME_BASED"}

d = usb.core.find(idVendor=VID, idProduct=PID, backend=libusb_package.get_libusb1_backend())
assert d is not None, "AC1 not found"
cfg = d.get_active_configuration()
for intf in cfg:
    if intf.bInterfaceClass != 0x0E or intf.bInterfaceSubClass != 0x02:
        continue
    print(f"=== VideoStreaming interface {intf.bInterfaceNumber} alt {intf.bAlternateSetting}")
    extra = bytes(intf.extra_descriptors)
    i = 0
    while i + 2 < len(extra):
        ln, typ, sub = extra[i], extra[i + 1], extra[i + 2]
        if ln == 0:
            break
        blk = extra[i:i + ln]
        if typ == 0x24:  # CS_INTERFACE
            if sub in FMT:
                if sub == 0x04 or sub == 0x10:
                    guid = blk[5:21]
                    fourcc = guid[:4].decode("ascii", "replace")
                    print(f"  FORMAT {FMT[sub]} idx={blk[3]} frames={blk[4]} fourcc={fourcc!r} bpp={blk[21]}")
                else:
                    print(f"  FORMAT {FMT[sub]} idx={blk[3]} frames={blk[4]}")
            elif sub in FRM:
                w, h = struct.unpack_from("<HH", blk, 5)
                if sub == 0x11:
                    (dfi,) = struct.unpack_from("<I", blk, 17)
                    n = blk[21]
                    ivals = [struct.unpack_from("<I", blk, 26 + 4 * k)[0] for k in range(n)] if n else []
                else:
                    (dfi,) = struct.unpack_from("<I", blk, 21)
                    n = blk[25]
                    ivals = [struct.unpack_from("<I", blk, 26 + 4 * k)[0] for k in range(n)] if n else []
                fps = [round(1e7 / v, 1) for v in ivals if v]
                print(f"    FRAME {FRM[sub]} idx={blk[3]} {w}x{h} default_fps={round(1e7/dfi,1) if dfi else '?'} fps_list={fps}")
            elif sub == 0x01:
                print(f"  VS_INPUT_HEADER formats={blk[3]} ep=0x{blk[6]:02x}")
        i += ln
