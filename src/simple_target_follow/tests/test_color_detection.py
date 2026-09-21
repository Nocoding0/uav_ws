import unittest
import cv2
import numpy as np
from simple_target_follow.color_detection import detect_color_target


class ColorDetectionTests(unittest.TestCase):
    def canvas(self):
        return np.zeros((480, 640, 3), np.uint8)

    def test_blue_center_and_bounds(self):
        image = self.canvas()
        cv2.rectangle(image, (270, 190), (370, 290), (255, 0, 0), -1)
        target, mask, reason = detect_color_target(image)
        self.assertEqual(reason, 'detected')
        self.assertAlmostEqual(target.center[0], 0.5)
        self.assertAlmostEqual(target.center[1], 0.5)
        self.assertEqual(target.bbox, (270, 190, 101, 101))
        self.assertEqual(mask.dtype, np.uint8)

    def test_wrong_color_noise_and_empty_rejected(self):
        image = self.canvas()
        cv2.rectangle(image, (200, 100), (300, 200), (0, 255, 0), -1)
        cv2.rectangle(image, (400, 100), (403, 103), (255, 0, 0), -1)
        self.assertIsNone(detect_color_target(image)[0])
        self.assertIsNone(detect_color_target(self.canvas())[0])

    def test_ambiguous_targets_rejected(self):
        image = self.canvas()
        for x in (100, 400):
            cv2.rectangle(image, (x, 100), (x+80, 180), (255, 0, 0), -1)
        target, _, reason = detect_color_target(image)
        self.assertIsNone(target)
        self.assertEqual(reason, 'ambiguous_targets')

    def test_clipped_target_and_color_background_rejected(self):
        image = self.canvas()
        cv2.rectangle(image, (0, 100), (100, 200), (255, 0, 0), -1)
        self.assertIsNone(detect_color_target(image)[0])
        image[:] = (255, 0, 0)
        self.assertIsNone(detect_color_target(image)[0])

    def test_red_hue_wrap_supported(self):
        for hue in (0, 179):
            hsv = np.zeros((480, 640, 3), np.uint8)
            hsv[100:200, 100:200] = (hue, 255, 255)
            image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            self.assertIsNotNone(detect_color_target(image, (170, 100, 60), (10, 255, 255))[0])

    def test_invalid_parameters_rejected(self):
        with self.assertRaises(ValueError):
            detect_color_target(self.canvas(), (180, 0, 0))
        with self.assertRaises(ValueError):
            detect_color_target(self.canvas(), minimum_area=-1)


if __name__ == '__main__':
    unittest.main()
