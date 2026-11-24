"""Classical CV: find the spray-painted removal area in an image and turn it into a grid.

On construction sites the area to treat is usually marked with spray paint. The
pipeline is deliberately classical (no training data exists for this setup) and
every step is explainable:

  1. colour threshold in HSV  - red paint is strongly saturated, concrete and wet
                                stains are grey (low saturation), so saturation
                                separates them better than brightness
  2. morphological close/open - close fills the spray gaps, open removes speckle
  3. drop small blobs         - stray overspray must not become a target
  4. fill each blob's outline - the marked area includes its interior
  5. image -> grid            - a cell is a target if enough of it is painted

Pure functions (numpy/OpenCV only, no ROS) so they are unit-tested with pytest.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import cv2
import numpy as np


@dataclass
class MaskConfig:
    # red wraps around hue 0 in OpenCV's 0..179 hue range, so two bands
    hue_low_max: int = 10
    hue_high_min: int = 170
    sat_min: int = 100            # paint is saturated, concrete is not
    val_min: int = 60             # ignore near-black pixels
    close_px: int = 9             # kernel size to bridge spray gaps
    open_px: int = 5              # kernel size to remove speckle
    min_area_frac: float = 0.005  # blobs smaller than this share of the image are dropped
    cell_min_fraction: float = 0.3  # a cell is target if >= 30 % of it is painted


def paint_mask(bgr: np.ndarray, cfg: MaskConfig) -> np.ndarray:
    """Steps 1-4: binary mask (uint8, 0/255) of the marked area."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    low = cv2.inRange(hsv, (0, cfg.sat_min, cfg.val_min), (cfg.hue_low_max, 255, 255))
    high = cv2.inRange(hsv, (cfg.hue_high_min, cfg.sat_min, cfg.val_min), (179, 255, 255))
    mask = cv2.bitwise_or(low, high)

    if cfg.close_px > 1:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cfg.close_px, cfg.close_px))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    if cfg.open_px > 1:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cfg.open_px, cfg.open_px))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = cfg.min_area_frac * mask.shape[0] * mask.shape[1]
    keep = [c for c in contours if cv2.contourArea(c) >= min_area]
    filled = np.zeros_like(mask)
    cv2.drawContours(filled, keep, -1, 255, thickness=cv2.FILLED)
    return filled


def mask_to_grid(mask: np.ndarray, cols: int, rows: int, cell_min_fraction: float) -> List[bool]:
    """Step 5: resample the mask onto the robot's grid.

    INTER_AREA averages all pixels of a cell = painted fraction of that cell.
    Image row 0 is the TOP, grid row 0 is the BOTTOM (origin bottom-left), so flip.
    Returns a row-major list, same layout as coverage::Grid / OccupancyGrid.
    """
    frac = cv2.resize(mask.astype(np.float32) / 255.0, (cols, rows), interpolation=cv2.INTER_AREA)
    frac = np.flipud(frac)
    return [bool(v >= cell_min_fraction) for v in frac.reshape(-1)]


def overlay(bgr: np.ndarray, mask: np.ndarray, grid: List[bool], cols: int, rows: int) -> np.ndarray:
    """Debug image: detected area tinted, target cells outlined."""
    out = bgr.copy()
    tint = out.copy()
    tint[mask > 0] = (255, 160, 0)
    out = cv2.addWeighted(tint, 0.35, out, 0.65, 0)
    h, w = out.shape[:2]
    cw, ch = w / cols, h / rows
    for i, on in enumerate(grid):
        if on:
            r, c = divmod(i, cols)
            y0 = int(h - (r + 1) * ch)  # grid row 0 is at the bottom of the image
            cv2.rectangle(out, (int(c * cw), y0), (int((c + 1) * cw), int(y0 + ch)), (255, 255, 255), 1)
    return out


def summary(grid: List[bool], cell_size: float) -> Tuple[int, float]:
    """(number of target cells, target area in m^2)."""
    n = sum(grid)
    return n, n * cell_size * cell_size
