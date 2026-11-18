# MB41 (Amlogic SC2, kernel 6.19.6-rwsdev): USB 2.0 high-speed workaround

On this image, high-speed USB devices fail to enumerate (`usb usb1-port1: Cannot enable. Maybe the USB cable is bad?`):
the device connects at full speed, and at port reset the xHCI port reports a disconnect. Seen with the RoboSense AC1.

Found empirically on 2026-11-18: replaying a series of runtime USB2-PHY / dwc3 register writes and then restoring
every register to its boot value leaves the first host port (`usb1-port1`) working at 480 Mb/s until the next reboot.
Nothing is written to storage.

- `validated_original/replay_all.sh` (+ `phyreg.py`, `phypll.py`, `mempoke.py` in the same folder, expected in `~`):
  **the files that were actually run.** Verified twice: clean reboot -> AC1 fails -> run -> AC1 enumerates at high speed,
  no physical change. Takes about 40 s. Needs passwordless sudo. `p1try.sh` forces a fresh enumeration on port 1.
- `mb41_usb_hs_workaround.sh`: the same sequence as one self-contained file (run with sudo). **Not yet run.**

What is known: registers read identical before and after (only PLL lock status bits R16[30]/R19[30] change);
a PLL re-lock alone does not fix it; each step alone does not fix it; the working state survives replug and port
resets and is lost on reboot; the second port (`usb1-port2`) still fails. The minimal step is not yet isolated,
and the real fix belongs in the kernel's SC2 USB2 PHY init (vendor reference: U-Boot `arch/arm/mach-meson/sc2/usb.c`).
