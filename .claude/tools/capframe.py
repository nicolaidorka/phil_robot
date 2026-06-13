#!/usr/bin/env python3
"""Grab a single still from a USB camera for the camera-guided board-inspection
loop (e.g. reading TMC driver chip labels during the reflash project).

Usage:
    python3 capframe.py [device_index] [out.jpg] [exposure]

  device_index : V4L2 index. Logitech Brio = 2 (built-in laptop cam = 0).
  out.jpg      : output path (default /tmp/frame_<idx>.jpg)
  exposure     : manual exposure value (lower = darker, kills glare/reflection
                 on shiny PCBs). Omit or 'auto' for auto-exposure.

Notes:
  - cv2 (opencv 4.11) + the Logitech Brio 101 are already installed/connected.
  - Brio reports 1920x1080 here even when 4K is requested.
  - For shiny chips, glare blows out the silkscreen; pass an exposure like
    50-150 AND/OR have the user tilt the board so the reflection moves off the chip.
"""
import sys, cv2, time

idx = int(sys.argv[1]) if len(sys.argv) > 1 else 2
out = sys.argv[2] if len(sys.argv) > 2 else f"/tmp/frame_{idx}.jpg"
exp = sys.argv[3] if len(sys.argv) > 3 else "auto"

cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
if not cap.isOpened():
    cap = cv2.VideoCapture(idx)
if not cap.isOpened():
    print(f"FAIL: cannot open video{idx}"); sys.exit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)

if exp != "auto":
    # V4L2/UVC: AUTO_EXPOSURE=1 => manual mode, 3 => aperture-priority auto
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    cap.set(cv2.CAP_PROP_EXPOSURE, float(exp))

ok = False
for _ in range(40):                 # warm up: let AF / AE settle
    ok, frame = cap.read()
    time.sleep(0.05)
w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)); h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
cap.release()
if not ok or frame is None:
    print(f"FAIL: opened video{idx} but no frame"); sys.exit(2)
cv2.imwrite(out, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
print(f"OK video{idx} -> {out}  ({w}x{h})  exposure={exp}")
