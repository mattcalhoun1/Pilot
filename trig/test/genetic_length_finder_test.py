import unittest
from trig.trig import BasicTrigCalc
from trig.genetic_length_finder import BaseTopLengthFinder
import logging

class TestGeneticLengthFinder(unittest.TestCase):
    def setUp(self) -> None:
        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
        return super().setUp()

    def test_basic_find_solution (self):
        # a known-good triangle
        #    angle : side
        # far   25.176 : 134
        # base  90.033 : 315
        # top   64.79  : 285

        # loop through a range of estimates and see what it comes up with
        for est_base,est_top in [(310,270),(315,285)]:
            far_angle = 25.176
            far_side = 134
            target_accuracy = 0.01
            finder = BaseTopLengthFinder(
                far_angle=far_angle,
                far_side=far_side,
                est_base=est_base,
                est_top=est_top
            )
            solutions = finder.find_lengths(
                max_num_solutions=3,
                target_accuracy=target_accuracy)
            
            #for prop_base, prop_top, diff in solutions:
            #    logging.getLogger(__name__).info(f"Proposed: far side: 134, base: {prop_base}, top: {prop_top}")

            self.assertEqual(3, len(solutions))

            # make sure each solution works
            calc = BasicTrigCalc()
            for proposed_base, proposed_top, diff in solutions:
                base_angle = calc.calc_base_angle(
                    far_side=far_side, 
                    base_side=proposed_base, 
                    top_side=proposed_top
                )

                # get the far angle and make sure it has not deviated by more than the desired accuracy
                proposed_far_angle = calc.calc_far_angle(far_side=far_side, base_side=proposed_base, top_side=proposed_top)
                self.assertLessEqual(abs(proposed_far_angle - far_angle)/far_angle, target_accuracy)



