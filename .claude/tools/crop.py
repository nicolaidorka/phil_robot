#!/usr/bin/env python3
"""Crop + upscale + sharpen regions of a captured frame to read small text
(chip part numbers, PCB silkscreen). Edit REGIONS for the frame at hand.

Usage: python3 crop.py [frame.jpg]
Writes /tmp/crop_<name>.jpg for each region.
"""
import sys, cv2

src = sys.argv[1] if len(sys.argv) > 1 else "/tmp/frame_2.jpg"
img = cv2.imread(src)
H, W = img.shape[:2]
print("full res", W, H)

# regions as fractions (x0,y0,x1,y1)
REGIONS = {
    "a": (0.00, 0.00, 0.40, 0.30),
    "b": (0.30, 0.00, 0.70, 0.30),
    "c": (0.28, 0.30, 0.70, 0.65),
}
for name, (x0, y0, x1, y1) in REGIONS.items():
    c = img[int(y0*H):int(y1*H), int(x0*W):int(x1*W)]
    c = cv2.resize(c, None, fx=3.0, fy=3.0, interpolation=cv2.INTER_CUBIC)
    blur = cv2.GaussianBlur(c, (0, 0), 3)
    c = cv2.addWeighted(c, 1.6, blur, -0.6, 0)
    cv2.imwrite(f"/tmp/crop_{name}.jpg", c, [cv2.IMWRITE_JPEG_QUALITY, 95])
    print("wrote", name, c.shape[1], c.shape[0])
