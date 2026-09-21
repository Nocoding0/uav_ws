"""HSV color target measurements; no ROS or flight-control dependencies."""
from dataclasses import dataclass
from typing import Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class ColorTarget:
    center: Tuple[float, float]  # normalized image coordinates, right/down positive
    bbox: Tuple[int, int, int, int]
    area: float
    fill_ratio: float


def detect_color_target(image, lower=(95, 100, 60), upper=(130, 255, 255),
                        minimum_area=300.0, maximum_area_fraction=0.5,
                        minimum_fill_ratio=0.5, ambiguity_ratio=0.65):
    """Return (measurement, mask, reason). Reject similarly sized candidates.

    Hue is OpenCV's [0,179]; a lower hue above upper hue selects a range
    crossing zero (e.g. red). Size and fill checks are for a solid target.
    """
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError('image must be an 8-bit BGR image')
    for bounds in (lower, upper):
        if len(bounds) != 3 or any(v < 0 or v > limit for v, limit in zip(bounds, (179, 255, 255))):
            raise ValueError('HSV bounds outside OpenCV range')
    if lower[1] > upper[1] or lower[2] > upper[2]:
        raise ValueError('S/V lower bounds must not exceed upper bounds')
    if not (minimum_area > 0 and 0 < maximum_area_fraction <= 1
            and 0 < minimum_fill_ratio <= 1 and 0 < ambiguity_ratio <= 1):
        raise ValueError('invalid shape/ambiguity thresholds')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    if lower[0] <= upper[0]:
        mask = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
    else:
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, np.array((0, lower[1], lower[2]), np.uint8), np.array(upper, np.uint8)),
            cv2.inRange(hsv, np.array(lower, np.uint8), np.array((179, upper[1], upper[2]), np.uint8)))
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    height, width = image.shape[:2]
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if not minimum_area <= area <= maximum_area_fraction * width * height:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if x <= 0 or y <= 0 or x + w >= width or y + h >= height:
            continue  # truncated shape: center and size are unreliable
        fill = area / (w * h)
        if fill < minimum_fill_ratio:
            continue
        moments = cv2.moments(contour)
        center = (moments['m10'] / moments['m00'] / width,
                  moments['m01'] / moments['m00'] / height)
        candidates.append(ColorTarget(center, (x, y, w, h), area, fill))
    candidates.sort(key=lambda item: item.area, reverse=True)
    if not candidates:
        return None, mask, 'no_target'
    if len(candidates) > 1 and candidates[1].area >= candidates[0].area * ambiguity_ratio:
        return None, mask, 'ambiguous_targets'
    return candidates[0], mask, 'detected'
