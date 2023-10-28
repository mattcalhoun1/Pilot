import unittest
from visual.visual_degrees import VisualDegreesCalculator
import logging

class TestVisualDegreesCalculator(unittest.TestCase):
    def setUp(self) -> None:
        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
        return super().setUp()

    def test_visual_degrees_height (self):
        # fov is 120 deg horz, 66 deg vert
        # image res is 1280 x 720

        # horizontal = 10.666 pixels per degree
        # vertical = 10.9 pixes per degree

        calc = VisualDegreesCalculator(horizontal_fov = 120.0, vertical_fov = 66.0, view_width=1280.0, view_height=720.0)
        vert_degrees = calc.caclulate_vertical_visual_degrees_given_height_pixels(100)

        self.assertEqual(round(vert_degrees, 2), 9.17)


    def test_pixels_for_horz_degrees (self):
        calc = VisualDegreesCalculator(horizontal_fov = 120.0, vertical_fov = 66.0, view_width=1280.0, view_height=720.0)
        # fov is 120 deg horz, 66 deg vert
        # image res is 1280 x 720

        # horizontal = 10.666 pixels per degree
        # vertical = 10.9 pixes per degree

        pixels = calc.calculate_pixels_given_horizontal_degree (2.0)
        self.assertEqual(round(pixels,1), round(10.666*2,1))


    def test_visual_degrees_degrees_width (self):
        # fov is 120 deg horz, 66 deg vert
        # image res is 1280 x 720

        # horizontal = 10.666 pixels per degree
        # vertical = 10.9 pixes per degree

        calc = VisualDegreesCalculator(horizontal_fov = 120.0, vertical_fov = 66.0, view_width=1280.0, view_height=720.0)
        horz_degrees = calc.caclulate_horizontal_visual_degrees_given_width_pixels(100)

        self.assertEqual(round(horz_degrees, 2), 9.38)


    def test_point_based_calcs (self):
        # fov is 120 deg horz, 66 deg vert
        # image res is 1280 x 720

        # horizontal = 10.666 pixels per degree
        # vertical = 10.9 pixes per degree

        calc = VisualDegreesCalculator(horizontal_fov = 120.0, vertical_fov = 66.0, view_width=1280.0, view_height=720.0)

        # point 1 : 25,20
        # point 2 : 60,70
        # vertical size : 50
        # horz size = 35

        # horz degrees: 5.722180574
        # vert degrees: 5.599337431
        vert_degrees = calc.caclulate_vertical_visual_degrees(x1=25.0,y1=20.0,x2=60.0,y2=70.0)
        self.assertEqual(round(vert_degrees,1),4.6)

        horz_degrees = calc.caclulate_horizontal_visual_degrees(x1=25.0,y1=20.0,x2=60.0,y2=70.0)
        self.assertEqual(round(horz_degrees,1),3.3)

    def test_slope_degrees (self):
        calc = VisualDegreesCalculator(horizontal_fov = 120.0, vertical_fov = 66.0, view_width=1280.0, view_height=720.0)

        # Forward positive
        slope_deg = calc.calculate_degrees_from_slope (x1=0, y1=0, x2=2, y2=2)
        self.assertEqual(45.0, slope_deg)

        # slope of 0
        slope_deg = calc.calculate_degrees_from_slope (x1=0, y1=0, x2=2, y2=0)
        self.assertEqual(0, slope_deg)

        # Forward negative
        slope_deg = calc.calculate_degrees_from_slope (x1=0, y1=0, x2=2, y2=-2)
        self.assertEqual(-45.0, slope_deg)

        # backward zero
        slope_deg = calc.calculate_degrees_from_slope (x1=0, y1=0, x2=-2, y2=0)
        self.assertEqual(0, slope_deg)

        # backward negative
        slope_deg = calc.calculate_degrees_from_slope (x1=0, y1=0, x2=-2, y2=-2)
        self.assertEqual(45.0, slope_deg)

        # backward positive
        slope_deg = calc.calculate_degrees_from_slope (x1=0, y1=0, x2=-2, y2=2)
        self.assertEqual(-45.0, slope_deg)

    def test_perspective_degrees(self):
        calc = VisualDegreesCalculator(horizontal_fov = 120.0, vertical_fov = 66.0, view_width=1280.0, view_height=720.0)

        # using https://www.desmos.com/calculator/1hg0vdgcf4

        # base is at 3,3
        # point is (below/right) at 3,1
        # i am at -1, -1

        # my perspective should be: 18 deg


        deg = calc.calculate_degrees_given_point_and_perspective (
            perspective_x=-1,
            perspective_y=-1,
            base_x=3,
            base_y=3,
            point_x=3,
            point_y=1)
        self.assertEqual(round(deg,1),18.4)

        # if i flip the points, they should still ahve the same degrees
        flipped_deg = calc.calculate_degrees_given_point_and_perspective (
            perspective_x=-1,
            perspective_y=-1,
            base_x=3,
            base_y=3,
            point_x=3,
            point_y=1)
        self.assertEqual(round(flipped_deg,1),18.4)

        # from up/right, looking down/left
        deg = calc.calculate_degrees_given_point_and_perspective (
            perspective_x=5,
            perspective_y=4,
            base_x=3,
            base_y=3,
            point_x=3,
            point_y=1)
        self.assertEqual(round(deg,1),29.7)

    def test_relative_heading(self):
        calc = VisualDegreesCalculator(horizontal_fov = 120.0, vertical_fov = 66.0, view_width=1280.0, view_height=720.0)
        
        relative_north = calc.calculate_relative_north(
            perspective_x=1,
            perspective_y=2,
            point_x=-3,
            point_y=1
        )

        self.assertEqual(104, round(relative_north))


        relative_north = calc.calculate_relative_north(
            perspective_x=1,
            perspective_y=2,
            point_x=5,
            point_y=1
        )

        self.assertEqual(-104, round(relative_north))


        relative_north = calc.calculate_relative_north(
            perspective_x=0,
            perspective_y=-4,
            point_x=5,
            point_y=1
        )

        self.assertEqual(-45, round(relative_north))

        relative_north = calc.calculate_relative_north(
            perspective_x=0,
            perspective_y=-4,
            point_x=-5,
            point_y=1
        )

        self.assertEqual(45, round(relative_north))
