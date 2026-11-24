"""Unit tests for the paint-marking perception. Pure OpenCV/numpy, no ROS needed."""

import os

import cv2
import numpy as np
import pytest

from coverage_perception.mask import MaskConfig, mask_to_grid, paint_mask, summary

RED = (40, 35, 200)    # BGR spray red
GREY = (150, 150, 150)


def concrete(h=300, w=400, seed=0):
    rng = np.random.default_rng(seed)
    g = np.clip(150 + rng.normal(0, 12, (h, w)), 0, 255).astype(np.uint8)
    return cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)


def test_painted_rectangle_is_found_and_grey_is_not():
    img = concrete()
    cv2.rectangle(img, (100, 75), (299, 224), RED, -1)  # exactly 200 x 150 px
    m = paint_mask(img, MaskConfig())
    assert m[150, 200] == 255          # inside the paint
    assert m[10, 10] == 0              # plain concrete
    assert abs((m > 0).sum() - 200 * 150) < 0.02 * 200 * 150


def test_grey_stain_is_not_paint():
    img = concrete()
    cv2.ellipse(img, (200, 150), (80, 50), 0, 0, 360, (85, 85, 90), -1)  # dark, unsaturated
    assert paint_mask(img, MaskConfig()).sum() == 0


def test_spray_gaps_are_closed():
    img = concrete()
    rng = np.random.default_rng(1)
    block = np.zeros(img.shape[:2], bool)
    block[75:225, 100:300] = True
    block &= rng.random(block.shape) > 0.15  # 15 % of the paint missing (spray texture)
    img[block] = RED
    m = paint_mask(img, MaskConfig())
    inside = m[85:215, 110:290]
    assert (inside > 0).mean() > 0.99      # holes inside the area are filled


def test_small_overspray_specks_are_ignored():
    img = concrete()
    for c in [(30, 30), (350, 40), (60, 250), (370, 270)]:
        cv2.circle(img, c, 4, RED, -1)
    assert paint_mask(img, MaskConfig()).sum() == 0


def test_mask_to_grid_flips_rows_and_applies_threshold():
    mask = np.zeros((300, 400), np.uint8)
    mask[0:150, 0:200] = 255  # top-left quarter of the IMAGE
    grid = mask_to_grid(mask, cols=4, rows=2, cell_min_fraction=0.3)
    # grid row 0 is the bottom, so the painted area must appear in grid row 1, cols 0-1
    assert grid == [False, False, False, False,
                    True, True, False, False]


def test_cell_threshold():
    mask = np.zeros((100, 100), np.uint8)
    mask[:, 0:20] = 255  # 20 % of the single cell
    assert mask_to_grid(mask, 1, 1, 0.3) == [False]
    assert mask_to_grid(mask, 1, 1, 0.1) == [True]


def test_summary_area():
    assert summary([True, True, False, True], 0.05) == (3, pytest.approx(3 * 0.0025))


def test_sample_image_ignores_stain_and_specks():
    path = os.path.join(os.path.dirname(__file__), "..", "data", "sample_marking.jpg")
    img = cv2.imread(path)
    assert img is not None
    m = paint_mask(img, MaskConfig())
    assert m[300, 300] == 255   # inside the marked polygon
    assert m[150, 620] == 0     # centre of the dark wet stain
    assert m[340, 658] == 0     # one of the overspray specks
    cells, _ = summary(mask_to_grid(m, 40, 30, 0.3), 0.05)
    assert 400 < cells < 520    # ~460 cells for this marking
