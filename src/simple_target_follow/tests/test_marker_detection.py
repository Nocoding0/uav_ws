#!/usr/bin/env python3

import os
import sys
import unittest

import cv2
import numpy as np


PACKAGE_SRC = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "src")
)
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from simple_target_follow.marker_detection import detect_concentric_marker


class MarkerDetectionTest(unittest.TestCase):
    @staticmethod
    def marker_image(
        *,
        center=(640, 360),
        outer_axes=(150, 150),
        angle_deg=0.0,
        include_outer=True,
        include_inner=True,
        include_cross=True,
        cross_extent=0.38,
        inner_offset=(0, 0),
    ):
        image = np.full((720, 1280, 3), 255, dtype=np.uint8)
        outer_center = tuple(int(value) for value in center)
        outer_axes = tuple(int(value) for value in outer_axes)
        inner_center = (
            outer_center[0] + int(inner_offset[0]),
            outer_center[1] + int(inner_offset[1]),
        )
        inner_axes = (
            int(round(outer_axes[0] * 0.60)),
            int(round(outer_axes[1] * 0.60)),
        )
        if include_outer:
            cv2.ellipse(
                image,
                outer_center,
                outer_axes,
                angle_deg,
                0.0,
                360.0,
                (0, 0, 0),
                18,
            )
        if include_outer and include_inner:
            cv2.ellipse(
                image,
                inner_center,
                inner_axes,
                angle_deg,
                0.0,
                360.0,
                (0, 0, 0),
                16,
            )
        if include_cross:
            half_horizontal = int(round(outer_axes[0] * cross_extent))
            half_vertical = int(round(outer_axes[1] * cross_extent))
            angle = np.deg2rad(angle_deg)

            def rotated_endpoint(local_x, local_y):
                return (
                    int(
                        round(
                            outer_center[0]
                            + local_x * np.cos(angle)
                            - local_y * np.sin(angle)
                        )
                    ),
                    int(
                        round(
                            outer_center[1]
                            + local_x * np.sin(angle)
                            + local_y * np.cos(angle)
                        )
                    ),
                )

            cv2.line(
                image,
                rotated_endpoint(-half_horizontal, 0),
                rotated_endpoint(half_horizontal, 0),
                (0, 0, 0),
                12,
            )
            cv2.line(
                image,
                rotated_endpoint(0, -half_vertical),
                rotated_endpoint(0, half_vertical),
                (0, 0, 0),
                12,
            )
        return image

    def detect(self, image):
        return detect_concentric_marker(
            image,
            minimum_area=250.0,
            minimum_normalized_size=0.08,
            maximum_normalized_size=0.50,
        )

    def test_accepts_complete_double_ring_and_center_cross(self):
        detection = self.detect(self.marker_image())
        self.assertIsNotNone(detection)
        center_x, center_y, marker_size, ellipses, cross_lines = detection
        self.assertAlmostEqual(center_x, 0.5, delta=0.01)
        self.assertAlmostEqual(center_y, 0.5, delta=0.01)
        self.assertAlmostEqual(marker_size, 300.0 / 1280.0, delta=0.02)
        self.assertTrue(ellipses)
        self.assertTrue(cross_lines)

    def test_accepts_cross_spanning_both_rings(self):
        image = self.marker_image(cross_extent=1.0)
        detection = self.detect(image)
        self.assertIsNotNone(detection)
        self.assertTrue(detection[3])
        self.assertTrue(detection[4])

    def test_accepts_moderate_perspective_ellipse(self):
        detection = self.detect(
            self.marker_image(outer_axes=(150, 108), angle_deg=27.0)
        )
        self.assertIsNotNone(detection)

    def test_accepts_cross_without_rings(self):
        self.assertIsNotNone(
            self.detect(
                self.marker_image(include_outer=False, include_inner=False)
            )
        )

    def test_accepts_outer_ring_and_cross_without_inner_ring(self):
        detection = self.detect(self.marker_image(include_inner=False))
        self.assertIsNotNone(detection)
        self.assertFalse(bool(detection[3] and detection[4]))

    def test_accepts_double_ring_without_center_cross(self):
        detection = self.detect(self.marker_image(include_cross=False))
        self.assertIsNotNone(detection)
        self.assertFalse(bool(detection[3] and detection[4]))

    def test_accepts_single_outer_ring(self):
        self.assertIsNotNone(
            self.detect(
                self.marker_image(include_inner=False, include_cross=False)
            )
        )

    def test_accepts_nonconcentric_inner_ring(self):
        self.assertIsNotNone(
            self.detect(self.marker_image(inner_offset=(38, 0)))
        )

    def test_accepts_moderate_occlusion(self):
        image = self.marker_image()
        cv2.rectangle(image, (635, 180), (820, 350), (255, 255, 255), -1)
        self.assertIsNotNone(self.detect(image))

    def test_rejects_severely_incomplete_marker(self):
        image = self.marker_image()
        cv2.rectangle(image, (640, 100), (1000, 650), (255, 255, 255), -1)
        self.assertIsNone(self.detect(image))

    def test_accepts_blur_and_uneven_illumination(self):
        image = cv2.GaussianBlur(self.marker_image(), (21, 21), 6)
        gradient = np.linspace(
            0.50, 1.0, image.shape[1], dtype=np.float32
        )[None, :, None]
        image = np.clip(
            image.astype(np.float32) * gradient + 12.0, 0, 255
        ).astype(np.uint8)
        self.assertIsNotNone(self.detect(image))

    def test_rejects_round_grille_with_spokes(self):
        image = np.full((720, 1280, 3), 255, dtype=np.uint8)
        center = (640, 360)
        cv2.circle(image, center, 150, (0, 0, 0), 18)
        for angle in range(0, 180, 30):
            radians = np.deg2rad(angle)
            offset = (
                int(round(115 * np.cos(radians))),
                int(round(115 * np.sin(radians))),
            )
            cv2.line(
                image,
                (center[0] - offset[0], center[1] - offset[1]),
                (center[0] + offset[0], center[1] + offset[1]),
                (0, 0, 0),
                8,
            )
        self.assertIsNone(self.detect(image))

    def test_rejects_single_line(self):
        image = np.full((720, 1280, 3), 255, dtype=np.uint8)
        cv2.line(image, (500, 360), (780, 360), (0, 0, 0), 14)
        self.assertIsNone(self.detect(image))

    def test_rejects_l_shape(self):
        image = np.full((720, 1280, 3), 255, dtype=np.uint8)
        cv2.line(image, (520, 240), (520, 480), (0, 0, 0), 14)
        cv2.line(image, (520, 480), (760, 480), (0, 0, 0), 14)
        self.assertIsNone(self.detect(image))

    def test_rejects_solid_square(self):
        image = np.full((720, 1280, 3), 255, dtype=np.uint8)
        cv2.rectangle(image, (510, 230), (770, 490), (0, 0, 0), -1)
        self.assertIsNone(self.detect(image))

    def test_rejects_elongated_oval_label(self):
        image = np.full((720, 1280, 3), 255, dtype=np.uint8)
        cv2.ellipse(
            image, (640, 360), (170, 58), 0.0, 0.0, 360.0, (0, 0, 0), 18
        )
        self.assertIsNone(self.detect(image))

    def test_rejects_marker_outside_configured_size_range(self):
        self.assertIsNone(
            self.detect(self.marker_image(outer_axes=(45, 45)))
        )


if __name__ == "__main__":
    unittest.main()
