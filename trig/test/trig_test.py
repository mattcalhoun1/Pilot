import unittest
from trig.trig import BasicTrigCalc
import logging

class TestBasicTrigCalc(unittest.TestCase):
    def setUp(self) -> None:
        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
        return super().setUp()

    def test_calc_far_side (self):
        calc = BasicTrigCalc()
        far_side = calc.calc_far_side (far_angle=20.0, base_side=30.0, top_side=25.0)
        self.assertEqual(round(far_side,2),10.75)


    def test_calc_base_angle (self):
        calc = BasicTrigCalc()
        base_angle = calc.calc_base_angle (far_side=10.75, base_side = 30.0, top_side=25.0)        
        self.assertEqual(round(base_angle,1),107.3)
    
    def test_calc_top_angle (self):
        calc = BasicTrigCalc()
        top_angle = calc.calc_top_angle (far_angle=20.0, base_side=30.0, top_side=25.0)

        self.assertEqual(round(top_angle,1), 52.7)

    def test_calc_top_side(self):
        calc = BasicTrigCalc()
        top_side = calc.calc_top_side (far_angle = 50.0, far_side=134.0, base_side=166.0)

        self.assertEqual(round(top_side,2),148.96)

    def test_calc_base_side(self):
        calc = BasicTrigCalc()
        base_side = calc.calc_base_side (far_angle = 50.0, far_side=134.0, top_side=148.96)

        self.assertEqual(round(base_side,2),166.0)

    def test_cartesian_heading (self):
        calc = BasicTrigCalc()

        north_cartesian = calc.convert_heading_to_cartesian (0.0)
        self.assertEqual(round(north_cartesian), 90)

        east_cartesian = calc.convert_heading_to_cartesian (90.0)
        self.assertEqual(round(east_cartesian), 0)

        west_cartesian = calc.convert_heading_to_cartesian (-90.0)
        self.assertEqual(round(west_cartesian), 180)

        south_cartesian = calc.convert_heading_to_cartesian (-180.0)
        self.assertEqual(round(south_cartesian), 270)

        other_south_cartesian = calc.convert_heading_to_cartesian (180.0)
        self.assertEqual(round(other_south_cartesian), 270)

        sw_cartesian = calc.convert_heading_to_cartesian (-150.0)
        self.assertEqual(round(sw_cartesian), 180 + 60)

        se_cartesian = calc.convert_heading_to_cartesian (150.0)
        self.assertEqual(round(se_cartesian), 360 - 60)

        nw_cartesian = calc.convert_heading_to_cartesian(-30.0)
        self.assertEqual(round(nw_cartesian), 90 + 30)

        ne_cartesian = calc.convert_heading_to_cartesian(30.0)
        self.assertEqual(round(ne_cartesian), 90 - 30)


    def test_unbounded_cartesian_heading (self):
        calc = BasicTrigCalc()

        east_cartesian = calc.convert_heading_to_cartesian (-360 + 90.0)
        self.assertEqual(round(east_cartesian), 0)

        west_cartesian = calc.convert_heading_to_cartesian (360 + -90.0)
        self.assertEqual(round(west_cartesian), 180)

        sw_cartesian = calc.convert_heading_to_cartesian (360 + -150.0)
        self.assertEqual(round(sw_cartesian), 180 + 60)

        se_cartesian = calc.convert_heading_to_cartesian (-360 + 150.0)
        self.assertEqual(round(se_cartesian), 360 - 60)

        nw_cartesian = calc.convert_heading_to_cartesian(360 + -30.0)
        self.assertEqual(round(nw_cartesian), 90 + 30)

        ne_cartesian = calc.convert_heading_to_cartesian(-360 + 30.0)
        self.assertEqual(round(ne_cartesian), 90 - 30)

    def test_coord_calc (self):
        calc = BasicTrigCalc()
        north_x, north_y = calc.get_coords_for_zeronorth_angle_and_distance (heading=0, x=10, y=10, distance=100, is_forward = True)
        self.assertEqual(round(north_x), 10)
        self.assertEqual(round(north_y), 110)

        rv_north_x, rv_north_y = calc.get_coords_for_zeronorth_angle_and_distance (heading=0, x=10, y=10, distance=100, is_forward = False)
        self.assertEqual(round(rv_north_x), 10)
        self.assertEqual(round(rv_north_y), -90)

        south_x, south_y = calc.get_coords_for_zeronorth_angle_and_distance (heading=-180, x=10, y=10, distance=100, is_forward = True)
        self.assertEqual(round(south_x), 10)
        self.assertEqual(round(south_y), -90)        

        rv_south_x, rv_south_y = calc.get_coords_for_zeronorth_angle_and_distance (heading=-180, x=10, y=10, distance=100, is_forward = False)
        self.assertEqual(round(rv_south_x), 10)
        self.assertEqual(round(rv_south_y), 110)        

        ne_x, ne_y = calc.get_coords_for_zeronorth_angle_and_distance (heading=45, x=-10, y=-10, distance=100, is_forward = True)
        self.assertEqual(round(ne_x), round(ne_y))
        self.assertGreater(ne_x, 50)
        self.assertLess(ne_x, 100)

        rv_ne_x, rv_ne_y = calc.get_coords_for_zeronorth_angle_and_distance (heading=45, x=-10, y=-10, distance=100, is_forward = False)
        self.assertEqual(round(rv_ne_x), round(rv_ne_y))
        self.assertLess(rv_ne_x, -70)
        self.assertGreater(rv_ne_x, -90)        


        nw_x, nw_y = calc.get_coords_for_zeronorth_angle_and_distance (heading=-45, x=-10, y=10, distance=100, is_forward = True)
        self.assertEqual(round(nw_x), -1 * round(nw_y))
        self.assertLess(nw_x, -80)
        self.assertGreater(nw_x, -110)

        rv_nw_x, rv_nw_y = calc.get_coords_for_zeronorth_angle_and_distance (heading=-45, x=-10, y=10, distance=100, is_forward = False)
        self.assertEqual(round(nw_x), -1 * round(nw_y))
        self.assertLess(rv_nw_x, 100)
        self.assertGreater(rv_nw_x, 60)


        sw_x, sw_y = calc.get_coords_for_zeronorth_angle_and_distance (heading=-135, x=-10, y=-10, distance=100, is_forward = True)
        self.assertEqual(round(ne_x), round(ne_y))
        self.assertLess(sw_x, -70)
        self.assertGreater(sw_x, -100)

        rv_sw_x, rv_sw_y = calc.get_coords_for_zeronorth_angle_and_distance (heading=-135, x=-10, y=-10, distance=100, is_forward = False)
        self.assertEqual(round(rv_sw_x), round(rv_sw_y))
        self.assertGreater(rv_sw_x, 50)
        self.assertLess(rv_sw_x, 100)
