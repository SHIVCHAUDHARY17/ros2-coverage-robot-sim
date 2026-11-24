"""Generate the synthetic test image used by coverage_perception (no camera needed).

A concrete surface (grey, noisy, uneven lighting, a dark wet stain) with an
irregular area marked in red spray paint - the usual way removal areas are
marked on site - plus a few stray paint specks that the perception must ignore.

    py scripts/make_sample_marking.py src/coverage_perception/data/sample_marking.jpg
"""
import sys

import cv2
import numpy as np

W, H = 800, 600  # 4:3, same aspect as the 2.0 m x 1.5 m workspace
rng = np.random.default_rng(3)

# concrete: grey base + fine grain + coarse blotches + left-to-right lighting gradient
img = np.full((H, W), 150.0)
img += rng.normal(0, 12, (H, W))
blotch = cv2.GaussianBlur(rng.normal(0, 1, (H // 8, W // 8)), (0, 0), 2)
img += 25 * cv2.resize(blotch, (W, H)) / (np.abs(blotch).max() + 1e-9)
img *= np.linspace(0.8, 1.1, W)[None, :]
img = np.clip(img, 0, 255).astype(np.uint8)
bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

# a dark wet stain (NOT paint: low saturation, must be ignored)
cv2.ellipse(bgr, (620, 150), (90, 50), 20, 0, 360, (85, 85, 90), -1)

# the marked area: irregular polygon filled with spray-paint texture
poly = np.array([[90, 120], [430, 80], [560, 210], [520, 420], [300, 520], [120, 450], [60, 280]],
                np.int32)
paint = np.zeros((H, W), np.uint8)
cv2.fillPoly(paint, [poly], 255)
cv2.polylines(paint, [poly], True, 255, 14)  # thicker outline, like a sprayed border
holes = rng.random((H, W)) < 0.12             # spray gaps: concrete shows through
paint[holes] = 0
red = np.zeros_like(bgr)
red[:] = (40, 35, 200)                         # BGR spray red
shade = rng.normal(1.0, 0.12, (H, W))[..., None]
painted = np.clip(red * shade, 0, 255).astype(np.uint8)
m = paint > 0
bgr[m] = (0.85 * painted[m] + 0.15 * bgr[m]).astype(np.uint8)

# stray overspray specks outside the area (small: perception must drop them)
for _ in range(12):
    c = (int(rng.integers(600, 780)), int(rng.integers(300, 580)))
    cv2.circle(bgr, c, int(rng.integers(2, 6)), (40, 35, 200), -1)

out = sys.argv[1] if len(sys.argv) > 1 else "sample_marking.jpg"
cv2.imwrite(out, bgr, [cv2.IMWRITE_JPEG_QUALITY, 88])
print("wrote", out, bgr.shape)
