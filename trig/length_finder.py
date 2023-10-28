from trig.trig import BasicTrigCalc
import logging

# Performs a step-by-step search to find possible triangle lengths, given a far angle and far side
class BaseTopLengthFinder:
    def __init__(self, far_angle, far_side, est_top, est_base, base_confidence = 0.8, top_confidence = 0.8):
        self.__far_angle = far_angle
        self.__far = far_side
        self.__est_top = est_top
        self.__est_base = est_base
        self.__trig_calc = BasicTrigCalc()
        self.__base_confidence = base_confidence
        self.__top_confidence = top_confidence

    # this method is recursive and adjusts lengths as necessary to get to a solution
    def __find_adjustment_lengths (self, curr_base, curr_top, step_size_pct = 0.01, max_depth = 100, max_steps_per_cycle = 1000, curr_depth = 0):

        # if we have hit max search depth, there is no solution within the search space we limited to
        if curr_depth == max_depth:
            return None, None

        # see if this is a viable solution
        try: 
            base_angle = self.__trig_calc.calc_base_angle(far_side=self.__far, base_side=curr_base, top_side=curr_top)

            # it is good, return this solution
            return curr_base, curr_top
        except Exception:
            # not a final solution yet, keep looking
            pass

        # given the current base and top length, calculate the
        # min distance increase (percentage-wise) of each that
        # would complete the triangle with the accepted far angle

        # do i need to weight the step sizes? if one position is more trusted than the other?

        # first for the base. Step size is inversely related to how much we have confidence
        # more confidence = smaller step sizes
        base_adjustment_step_size = self.__est_base * step_size_pct * (self.__top_confidence / self.__base_confidence)
        min_base_adjustment = 0.0
        min_base_adjustment_pct = 0.0
        have_base_solution = False
        base_adjusted_far_angle = None
        base_steps = 0
        while have_top_solution is False and base_steps < max_steps_per_cycle:
            base_steps += 1
            try:
                base_angle = self.__trig_calc.calc_base_angle(far_side=self.__far, base_side=curr_base + min_base_adjustment, top_side=curr_top)
                have_base_solution = True
                min_base_adjustment_pct = min_base_adjustment / curr_base

                # given that solution, find the top and far angle, and see how far we are from the known far angle
                base_adjusted_far_angle = self.__trig_calc.calc_far_angle (far_side=self.__far, base_side=curr_base + min_base_adjustment, top_side=curr_top)
            except Exception:
                # the triangle did not complete, take a step
                min_base_adjustment += base_adjustment_step_size

        # next the top
        top_adjustment_step_size = self.__est_top * step_size_pct * (self.__base_confidence / self.__top_confidence)
        min_top_adjustment = 0.0
        have_top_solution = False
        top_steps = 0
        top_adjusted_far_angle = None
        while have_top_solution is False and top_steps < max_steps_per_cycle:
            top_steps += 1
            try:
                base_angle = self.__trig_calc.calc_base_angle(far_side=self.__far, base_side=curr_base, top_side=curr_top + min_top_adjustment)
                have_top_solution = True
                min_top_adjustment_pct = min_top_adjustment / curr_top

                # given that solution, find the top and far angle, and see how far we are from the known far angle
                top_adjusted_far_angle = self.__trig_calc.calc_far_angle (far_side=self.__far, base_side=curr_base, top_side=curr_top + min_top_adjustment)
            except Exception:
                # the triangle did not complete, take a step
                min_top_adjustment += top_adjustment_step_size

        # which solution gets closer to the desired angle
        

        # see which solution required smaller adjustment, percentagewise. Go another step further down that path
        if have_base_solution and have_top_solution:
            if min_base_adjustment_pct < min_top_adjustment_pct:
                return self.__find_adjustment_lengths (curr_base + base_adjustment_step_size, curr_top, step_size_pct = step_size_pct, max_depth = max_depth, max_steps_per_cycle = max_steps_per_cycle, curr_depth = curr_depth + 1)
            else:
                return self.__find_adjustment_lengths (curr_base, curr_top + top_adjustment_step_size, step_size_pct = step_size_pct, max_depth = max_depth, max_steps_per_cycle = max_steps_per_cycle, curr_depth = curr_depth + 1)
        elif have_base_solution:
            return self.__find_adjustment_lengths (curr_base + base_adjustment_step_size, curr_top, step_size_pct = step_size_pct, max_depth = max_depth, max_steps_per_cycle = max_steps_per_cycle, curr_depth = curr_depth + 1)
        elif have_top_solution:
            return self.__find_adjustment_lengths (curr_base, curr_top + top_adjustment_step_size, step_size_pct = step_size_pct, max_depth = max_depth, max_steps_per_cycle = max_steps_per_cycle, curr_depth = curr_depth + 1)

        # there must be no solution
        return None,None

    def find_possible_lengths (self, step_sizes = [0.005,], max_steps = [1000,], max_depths=[100,]):
        possibilities = []
        for i,step_size in enumerate(step_sizes):
            base, top = self.__find_adjustment_lengths (
                curr_base=self.__est_base,
                curr_top=self.__est_top,
                step_size_pct=step_size,
                max_depth=max_depths[i],
                max_steps=max_steps[i],
                curr_depth=0,
            )
