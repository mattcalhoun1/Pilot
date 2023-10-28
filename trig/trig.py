import math
import logging

# performs basic trig calcs using termonology familiar in the perspective calculator
# everything is in degrees, not radians. These are basic trig calculations, there is no 'fuzziness' here
class BasicTrigCalc:
    def __init__(self):
        pass

    def calc_top_angle (self, far_angle, base_side, top_side):
        far_side = self.calc_far_side(far_angle=far_angle, base_side=base_side, top_side=top_side)
        base_angle = self.calc_base_angle(far_side=far_side, base_side=base_side, top_side=top_side)
        return 180 - (far_angle + base_angle)

    
    def calc_far_side (self, far_angle, base_side, top_side):
        # where a is far_side and A is far angle
        # a = √(b2 + c2 - 2bc·cos(A))
        return math.sqrt(base_side **2 + top_side **2 - (2 * base_side * top_side * math.cos(math.radians(far_angle))))

    def calc_top_side (self, far_angle, far_side, base_side):
        base_angle = math.degrees(math.asin( ( base_side * math.sin(math.radians(far_angle)) ) / far_side ) )
        top_angle = 180 - far_angle - base_angle

        return (far_side * (math.sin (math.radians(top_angle)))) / math.sin(math.radians(far_angle))

    def calc_base_side (self, far_angle, far_side, top_side):
        top_angle = math.degrees(math.asin( ( top_side * math.sin(math.radians(far_angle)) ) / far_side ) )
        base_angle = 180 - far_angle -top_angle

        return (far_side * (math.sin (math.radians(base_angle)))) / math.sin(math.radians(far_angle))

    def calc_base_angle (self, far_side, base_side, top_side):
        # where B is base_angle , a is far_side, b is base_side, c is top_side
        # ∠B = arccos( ( a2 + c2 - b2 ) / 2ac)
        base_angle_rad = math.acos((far_side ** 2 + top_side ** 2 - base_side ** 2) / (2 * far_side * top_side))
        return math.degrees(base_angle_rad)

    # given all the sides, find the far angle
    def calc_far_angle (self, far_side, base_side, top_side):
        part_one = None
        part_two = None
        try:
            # <C = arccos ((a2 + b2 - c2) / 2ab)
            part_one = top_side ** 2 + base_side ** 2 - far_side ** 2
            part_two = 2 * top_side * base_side

            far_rad = math.acos(part_one / part_two)
            return math.degrees(far_rad)
        except Exception as e:
            logging.getLogger(__name__).error(f"calc_far_angle error: {e}, far_side: {far_side}, base_side: {base_side}, top_side: {top_side}, part_one {part_one}, part_two {part_two}")

        return None