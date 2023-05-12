import unittest

import cv2
from cbdetect_py import (Corner, CornerType, Params, boards_from_corners,
                         find_corners)


class CBDetectTestCase(unittest.TestCase):
    def test_detect(self):
        image_paths = [
            ("../../example_data/e2.png", CornerType.SaddlePoint),
            ("../../example_data/e6.png", CornerType.MonkeySaddlePoint),
        ]

        for image_path, corner_type in image_paths:
            corners = Corner()
            boards = []
            params = Params()
            params.corner_type = corner_type

            img = cv2.imread(image_path, cv2.IMREAD_COLOR)

            find_corners(img, corners, params)
            # plot_corners(img, corners)
            boards_from_corners(img, corners, boards, params)
            # plot_boards(img, corners, boards, params)

            self.assertFalse(not corners.p, f"No corners found in image: {image_path}")
            self.assertFalse(not boards, f"No boards found in image: {image_path}")


if __name__ == "__main__":
    unittest.main()
