import unittest
from trig.trig import BasicTrigCalc

class TestBasicTrigCalc(unittest.TestCase):
    def setUp(self) -> None:
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
