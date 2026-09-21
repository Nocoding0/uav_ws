"""Tolerant geometric detector for the D-problem cross-and-rings marker.

The printed target is viewed from a moving aircraft, so blur, perspective,
uneven illumination and short partial occlusions are normal.  A complete
double-ring-and-cross pattern gives the strongest match, while an independently
recognisable centre cross or ring is sufficient for reacquisition.  Tiny line
fragments are deliberately rejected because their centre is ambiguous.
"""

from __future__ import annotations

import math
from typing import Iterable, Optional, Sequence, Tuple

import cv2
import numpy as np


Ellipse = Tuple[Tuple[float, float], Tuple[float, float], float]
Detection = Tuple[
    float,
    float,
    float,
    Tuple[Ellipse, ...],
    Tuple[np.ndarray, ...],
]


def _threshold_marker(
    image: np.ndarray,
    color_mode: str,
    blue_hsv_lower: Sequence[int],
    blue_hsv_upper: Sequence[int],
) -> np.ndarray:
    if color_mode == "blue":
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        threshold = cv2.inRange(
            hsv,
            np.asarray(blue_hsv_lower, dtype=np.uint8),
            np.asarray(blue_hsv_upper, dtype=np.uint8),
        )
        return cv2.morphologyEx(
            threshold,
            cv2.MORPH_CLOSE,
            np.ones((5, 5), dtype=np.uint8),
            iterations=1,
        )

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    return cv2.adaptiveThreshold(
        gray,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        41,
        7,
    )


def _ellipse_point(
    ellipse: Ellipse, normalized_radius: float, theta: float
) -> Tuple[int, int]:
    (cx, cy), (axis_a, axis_b), angle_deg = ellipse
    angle = math.radians(angle_deg)
    local_x = 0.5 * axis_a * normalized_radius * math.cos(theta)
    local_y = 0.5 * axis_b * normalized_radius * math.sin(theta)
    return (
        int(round(cx + local_x * math.cos(angle) - local_y * math.sin(angle))),
        int(round(cy + local_x * math.sin(angle) + local_y * math.cos(angle))),
    )


def _angular_band_coverage(
    mask: np.ndarray,
    ellipse: Ellipse,
    radii: Iterable[float],
    *,
    expect_dark: bool,
    sample_count: int = 96,
) -> Optional[float]:
    """Return the fraction of angles whose radial band has the expected tone."""
    height, width = mask.shape[:2]
    radii = tuple(radii)
    matching_angles = 0
    valid_angles = 0
    for index in range(sample_count):
        theta = 2.0 * math.pi * index / sample_count
        values = []
        for radius in radii:
            x, y = _ellipse_point(ellipse, radius, theta)
            if not (0 <= x < width and 0 <= y < height):
                values = []
                break
            values.append(mask[y, x] > 0)
        if not values:
            continue
        valid_angles += 1
        dark_fraction = sum(values) / float(len(values))
        if (expect_dark and dark_fraction >= 0.5) or (
            not expect_dark and dark_fraction <= 0.25
        ):
            matching_angles += 1
    # A marker near the edge of the frame remains useful for steering.  Avoid
    # accepting a tiny visible fragment, while allowing moderate clipping.
    if valid_angles < int(round(0.60 * sample_count)):
        return None
    return matching_angles / float(valid_angles)


def _ellipse_fit_error(contour: np.ndarray, ellipse: Ellipse) -> float:
    """Measure the 90th-percentile radial error of a contour ellipse fit."""
    (cx, cy), (axis_a, axis_b), angle_deg = ellipse
    if axis_a <= 0.0 or axis_b <= 0.0:
        return math.inf
    angle = math.radians(angle_deg)
    cos_angle = math.cos(angle)
    sin_angle = math.sin(angle)
    points = contour.reshape(-1, 2).astype(np.float64)
    dx = points[:, 0] - cx
    dy = points[:, 1] - cy
    local_x = dx * cos_angle + dy * sin_angle
    local_y = -dx * sin_angle + dy * cos_angle
    radii = np.sqrt(
        np.square(local_x / (0.5 * axis_a))
        + np.square(local_y / (0.5 * axis_b))
    )
    return float(np.percentile(np.abs(radii - 1.0), 90))


def _line_intersection(
    first: np.ndarray, second: np.ndarray
) -> Optional[Tuple[float, float]]:
    x1, y1, x2, y2 = (float(value) for value in first)
    x3, y3, x4, y4 = (float(value) for value in second)
    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denominator) < 1e-6:
        return None
    determinant_first = x1 * y2 - y1 * x2
    determinant_second = x3 * y4 - y3 * x4
    return (
        (
            determinant_first * (x3 - x4)
            - (x1 - x2) * determinant_second
        )
        / denominator,
        (
            determinant_first * (y3 - y4)
            - (y1 - y2) * determinant_second
        )
        / denominator,
    )


def _find_center_cross(
    mask: np.ndarray, ellipse: Ellipse, major: float
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    edges = cv2.Canny(mask, 50, 150)
    lines = cv2.HoughLinesP(
        edges,
        1,
        np.pi / 180.0,
        threshold=max(14, int(round(0.07 * major))),
        minLineLength=max(10, int(round(0.11 * major))),
        maxLineGap=max(10, int(round(0.12 * major))),
    )
    if lines is None:
        return None

    cx, cy = ellipse[0]
    candidates = []
    for raw_line in lines[:, 0, :]:
        x1, y1, x2, y2 = (float(value) for value in raw_line)
        dx = x2 - x1
        dy = y2 - y1
        length = math.hypot(dx, dy)
        if not 0.11 * major <= length <= 0.92 * major:
            continue
        distance = abs(
            dy * cx - dx * cy + x2 * y1 - y2 * x1
        ) / max(length, 1.0)
        midpoint_distance = math.hypot(
            0.5 * (x1 + x2) - cx,
            0.5 * (y1 + y2) - cy,
        )
        maximum_endpoint_distance = max(
            math.hypot(x1 - cx, y1 - cy),
            math.hypot(x2 - cx, y2 - cy),
        )
        if (
            distance <= 0.11 * major
            and midpoint_distance <= 0.34 * major
            and maximum_endpoint_distance <= 0.54 * major
        ):
            candidates.append(
                (raw_line, math.atan2(dy, dx) % math.pi, length)
            )

    best_pair = None
    best_score = -math.inf
    for first_index, first in enumerate(candidates):
        for second in candidates[first_index + 1 :]:
            difference = abs(first[1] - second[1])
            difference = min(difference, math.pi - difference)
            if not math.radians(58.0) <= difference <= math.radians(122.0):
                continue
            intersection = _line_intersection(first[0], second[0])
            if intersection is None:
                continue
            intersection_error = math.hypot(
                intersection[0] - cx, intersection[1] - cy
            )
            if intersection_error > 0.14 * major:
                continue
            length_ratio = min(first[2], second[2]) / max(first[2], second[2])
            if length_ratio < 0.28:
                continue
            orthogonality_error = abs(difference - 0.5 * math.pi)
            score = (
                first[2]
                + second[2]
                - 2.0 * intersection_error
                - major * orthogonality_error
            )
            if score > best_score:
                best_score = score
                best_pair = (first[0], second[0])
    return best_pair


def _cross_contour_score(contour: np.ndarray, major: float) -> Optional[float]:
    """Score a connected four-arm cross and reject boxes, circles and Ls."""
    area = abs(cv2.contourArea(contour))
    hull = cv2.convexHull(contour)
    hull_area = abs(cv2.contourArea(hull))
    if hull_area <= 0.0:
        return None
    solidity = area / hull_area
    if not 0.18 <= solidity <= 0.78:
        return None

    hull_indices = cv2.convexHull(contour, returnPoints=False)
    if hull_indices is None or len(hull_indices) < 4:
        return None
    try:
        defects = cv2.convexityDefects(contour, hull_indices)
    except cv2.error:
        return None
    if defects is None:
        return None
    deep_defects = sum(
        1
        for defect in defects[:, 0, :]
        if defect[3] / 256.0 >= 0.035 * major
    )
    if deep_defects < 3:
        return None
    return float(deep_defects) + (0.78 - solidity)


def _central_line_direction_count(
    mask: np.ndarray, ellipse: Ellipse, major: float
) -> int:
    """Count long line directions through an ellipse centre.

    One or two directions are compatible with a plain ring or centre cross.
    Three or more reject wheels, grilles and other spoke-like circles.
    """
    edges = cv2.Canny(mask, 50, 150)
    lines = cv2.HoughLinesP(
        edges,
        1,
        np.pi / 180.0,
        threshold=max(14, int(round(0.07 * major))),
        minLineLength=max(10, int(round(0.22 * major))),
        maxLineGap=max(8, int(round(0.08 * major))),
    )
    if lines is None:
        return 0
    cx, cy = ellipse[0]
    angles = []
    for raw_line in lines[:, 0, :]:
        x1, y1, x2, y2 = (float(value) for value in raw_line)
        dx = x2 - x1
        dy = y2 - y1
        length = math.hypot(dx, dy)
        if length < 0.22 * major:
            continue
        distance = abs(
            dy * cx - dx * cy + x2 * y1 - y2 * x1
        ) / max(length, 1.0)
        midpoint_distance = math.hypot(
            0.5 * (x1 + x2) - cx,
            0.5 * (y1 + y2) - cy,
        )
        if distance <= 0.10 * major and midpoint_distance <= 0.30 * major:
            angles.append(math.atan2(dy, dx) % math.pi)

    clusters = []
    tolerance = math.radians(14.0)
    for angle in sorted(angles):
        if not any(
            min(abs(angle - existing), math.pi - abs(angle - existing))
            <= tolerance
            for existing in clusters
        ):
            clusters.append(angle)
    return len(clusters)


def detect_concentric_marker(
    image: np.ndarray,
    minimum_area: float,
    color_mode: str = "black",
    blue_hsv_lower: Sequence[int] = (90, 70, 40),
    blue_hsv_upper: Sequence[int] = (140, 255, 255),
    minimum_normalized_size: float = 0.045,
    maximum_normalized_size: float = 0.70,
) -> Optional[Detection]:
    """Detect the complete target or any independently identifiable part.

    The complete double ring plus cross receives the highest score.  A centre
    cross or one sufficiently complete ring can also provide a centre for
    image-based steering.  Size limits remain caller-configurable because they
    are an effective protection against distant texture and nearby clutter.
    """
    if image is None or image.ndim != 3:
        return None
    height, width = image.shape[:2]
    if height < 2 or width < 2:
        return None

    mask = _threshold_marker(
        image, color_mode, blue_hsv_lower, blue_hsv_upper
    )
    contours, _ = cv2.findContours(
        mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE
    )
    candidates = []
    for contour in contours:
        area = abs(cv2.contourArea(contour))
        if area < minimum_area or len(contour) < 20:
            continue
        ellipse = cv2.fitEllipse(contour)
        _, (axis_a, axis_b), _ = ellipse
        major = max(axis_a, axis_b)
        minor = min(axis_a, axis_b)
        normalized_size = major / float(width)
        if not minimum_normalized_size <= normalized_size <= maximum_normalized_size:
            continue
        if major <= 0.0 or minor / major < 0.32:
            continue

        fit_error = _ellipse_fit_error(contour, ellipse)
        if fit_error > 0.22:
            continue

        # From outside inward the marker must contain an outer ring, a white
        # annulus, an inner ring, and a white centre crossed by two bars.
        # Multiple radial bands still reject grilles, text and cables.  The
        # coverage values are intentionally tolerant of glare, blur and short
        # occlusions; the independently detected cross supplies the remaining
        # discrimination.
        band_requirements = (
            ((0.88, 0.93, 0.98, 1.02), True, 0.58),
            ((0.72, 0.77, 0.82), False, 0.48),
            ((0.53, 0.58, 0.63, 0.68), True, 0.46),
            ((0.32, 0.38, 0.44, 0.48), False, 0.42),
            ((1.06, 1.11, 1.16), False, 0.52),
        )
        coverages = []
        valid_bands = True
        for radii, expect_dark, minimum_coverage in band_requirements:
            coverage = _angular_band_coverage(
                mask, ellipse, radii, expect_dark=expect_dark
            )
            if coverage is None or coverage < minimum_coverage:
                valid_bands = False
                break
            coverages.append(coverage)
        if not valid_bands:
            continue

        cross_pair = _find_center_cross(mask, ellipse, major)
        if cross_pair is None:
            continue

        predicted_inner = (
            ellipse[0],
            (axis_a * 0.60, axis_b * 0.60),
            ellipse[2],
        )
        score = sum(coverages) - 2.0 * fit_error
        candidates.append(
            (
                score,
                ellipse[0][0] / width,
                ellipse[0][1] / height,
                normalized_size,
                (ellipse, predicted_inner),
                cross_pair,
            )
        )

    # Fallback 1: the centre cross by itself.  Its connected four-arm contour
    # plus two perpendicular Hough lines is much safer than accepting an
    # arbitrary pair of background edges.
    for contour in contours:
        area = abs(cv2.contourArea(contour))
        if area < minimum_area or len(contour) < 12:
            continue
        x, y, box_width, box_height = cv2.boundingRect(contour)
        major = float(max(box_width, box_height))
        minor = float(min(box_width, box_height))
        normalized_size = major / float(width)
        if not minimum_normalized_size <= normalized_size <= maximum_normalized_size:
            continue
        if major <= 0.0 or minor / major < 0.45:
            continue
        shape_score = _cross_contour_score(contour, major)
        if shape_score is None:
            continue
        moments = cv2.moments(contour)
        if abs(moments["m00"]) > 1e-6:
            contour_center = (
                moments["m10"] / moments["m00"],
                moments["m01"] / moments["m00"],
            )
        else:
            contour_center = (
                x + 0.5 * (box_width - 1),
                y + 0.5 * (box_height - 1),
            )
        cross_region = (contour_center, (major, major), 0.0)
        if _central_line_direction_count(mask, cross_region, major) > 2:
            continue
        cross_pair = _find_center_cross(mask, cross_region, major)
        if cross_pair is None:
            continue
        intersection = _line_intersection(cross_pair[0], cross_pair[1])
        if intersection is None:
            continue
        center_x, center_y = intersection
        if not (0.0 <= center_x < width and 0.0 <= center_y < height):
            continue
        line_length = sum(
            math.hypot(float(line[2] - line[0]), float(line[3] - line[1]))
            for line in cross_pair
        )
        candidates.append(
            (
                2.0 + 0.1 * shape_score + 0.1 * line_length / major,
                center_x / width,
                center_y / height,
                normalized_size,
                (),
                cross_pair,
            )
        )

    # Fallback 2: either printed ring by itself.  Require a dark curved band,
    # light immediately inside and outside it, and reject spoke-like circles.
    for contour in contours:
        area = abs(cv2.contourArea(contour))
        if area < minimum_area or len(contour) < 20:
            continue
        ellipse = cv2.fitEllipse(contour)
        _, (axis_a, axis_b), _ = ellipse
        major = max(axis_a, axis_b)
        minor = min(axis_a, axis_b)
        normalized_size = major / float(width)
        if not minimum_normalized_size <= normalized_size <= maximum_normalized_size:
            continue
        # A single-ring fallback must remain close to circular.  The complete
        # multi-part detector above still accepts stronger perspective; this
        # stricter branch avoids oval labels and cable loops.
        if major <= 0.0 or minor / major < 0.60:
            continue
        fit_error = _ellipse_fit_error(contour, ellipse)
        if fit_error > 0.20:
            continue
        dark_coverage = _angular_band_coverage(
            mask, ellipse, (0.90, 0.96, 1.02), expect_dark=True
        )
        inside_light = _angular_band_coverage(
            mask, ellipse, (0.70, 0.76, 0.82), expect_dark=False
        )
        outside_light = _angular_band_coverage(
            mask, ellipse, (1.08, 1.14, 1.20), expect_dark=False
        )
        if (
            dark_coverage is None
            or inside_light is None
            or outside_light is None
            or dark_coverage < 0.62
            or inside_light < 0.48
            or outside_light < 0.52
            or _central_line_direction_count(mask, ellipse, major) > 2
        ):
            continue
        candidates.append(
            (
                1.0 + dark_coverage + inside_light + outside_light - fit_error,
                ellipse[0][0] / width,
                ellipse[0][1] / height,
                normalized_size,
                (ellipse,),
                (),
            )
        )

    if not candidates:
        return None
    best = max(candidates, key=lambda candidate: candidate[0])
    return best[1], best[2], best[3], best[4], best[5]
