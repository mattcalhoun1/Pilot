from arduino.tank import Tank
from arduino.meccar import MecCar
from pyPS4Controller.controller import Controller
import time
import logging
import threading

class ControlledDrive:
    def __init__(self, vehicle: MecCar, pilot, pilot_nav):
        self.__vehicle = vehicle
        self.__listener = ControllerListener(interface="/dev/input/js0", connecting_using_ds4drv=False)
        self.__keep_going = False
        self.__pilot = pilot
        self.__pilot_nav = pilot_nav
        
    def start_driving(self):
        # you can start listening before controller is paired, as long as you pair it within the timeout window
        self.__listener.set_vehicle(self.__vehicle)
        self.__listener.set_pilot(self.__pilot)
        self.__listener.set_pilot_nav(self.__pilot_nav)
        self.__listener.on_exit(self.stop_driving)
        self.__keep_going = True

        if self.__vehicle.is_streaming_lidar():
            x = threading.Thread(target=self.__listener.listen, args=(60,))
            x.start()        
        else:
            self.__listener.listen(timeout=60)
        
        while (self.__keep_going):
            self.__vehicle.is_ready()
            time.sleep(.5)
        
        # wait for thread to finish
        if self.__vehicle.is_streaming_lidar():
            x.join()
        

    def stop_driving(self):
        self.__listener.stop = True
        self.__keep_going = False

class ControllerDirection:
    NONE = 0
    UP = 1
    DOWN = 2
    LEFT = 3
    RIGHT = 4
        
class ControllerListener(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        
        # for controlling fw/back/strafe movement
        self.__last_arrow_direction = ControllerDirection.NONE

        # for controlling cameras
        self.__last_left_direction = ControllerDirection.NONE
        self.__last_left_value = 0

        # for controlling rotation
        self.__last_right_direction = ControllerDirection.NONE
        self.__last_right_value = 0

        # used to ignore incoming commands if still processing previous one
        self.__busy = False 

    def set_vehicle(self, vehicle):
        self.__vehicle = vehicle

    def set_pilot(self, pilot):
        self.__pilot = pilot

    def set_pilot_nav(self, pilot_nav):
        self.__pilot_nav = pilot_nav

    def on_exit(self, exit_method):
        self.__exit_method = exit_method

    def on_square_release(self):
        if not self.__is_busy():
            self.__set_busy()
            logging.getLogger(__name__).info("Logging lidar")
            self.__pilot.log_lidar()
            self.__set_ready()

    def on_square_press(self):
        pass

    def on_circle_release(self):
        if not self.__is_busy():
            self.__set_busy()
            logging.getLogger(__name__).info("Locating landmarks")
            self.__pilot_nav.locate_landmarks()
            self.__set_ready()

    def on_circle_press(self):
        pass

    def on_share_release(self):
        if not self.__is_busy():
            self.__set_busy()
            field_map = self.__pilot_nav.get_field_map()
            find_objects = []
            for o in field_map.get_searchable_objects():
                find_objects.append(o)
            logging.getLogger(__name__).info(f"Locating any of the following objects: {o}")
            self.__pilot_nav.locate_objects(objects=find_objects)
            self.__set_ready()

    def on_share_press(self):
        pass

    def on_triangle_release(self):
        if not self.__is_busy():
            self.__set_busy()
            logging.getLogger(__name__).info("Getting position")
            self.__pilot_nav.get_coords_and_heading(display_landmarks_on_vehicle=True)
            self.__set_ready()

    def on_triangle_press(self):
        pass

    def on_options_press (self):
        pass

    def on_options_release (self):
        if not self.__is_busy():
            self.__set_busy()
            logging.getLogger(__name__).info("Getting quick position")
            self.__pilot_nav.get_coords_and_heading (
                allow_camera_reposition = False, 
                cam_start_default_position = False,
                display_landmarks_on_vehicle=True
            )
            self.__set_ready()

    def on_up_arrow_press (self):
        self.__last_arrow_direction = ControllerDirection.UP

    def on_down_arrow_press (self):
        self.__last_arrow_direction = ControllerDirection.DOWN

    def on_up_down_arrow_release (self):
        self.__adjust_movement()

    def on_left_arrow_press (self):
        self.__last_arrow_direction = ControllerDirection.LEFT

    def on_right_arrow_press (self):
        self.__last_arrow_direction = ControllerDirection.RIGHT

    def on_left_right_arrow_release (self):
        self.__adjust_movement()

    def on_L3_up(self, value):
        self.__last_left_direction = ControllerDirection.UP
        self.__last_left_value = value
        
    def on_L3_down(self, value):
        self.__last_left_direction = ControllerDirection.DOWN
        self.__last_left_value = value

    def on_L3_left(self, value):
        self.__last_left_direction = ControllerDirection.LEFT
        self.__last_left_value = value

    def on_L3_right(self, value):
        self.__last_left_direction = ControllerDirection.RIGHT
        self.__last_left_value = value

    def on_L3_press(self, value):
        self.__last_left_direction = ControllerDirection.NONE
        self.__last_left_value = value

    def on_L3_release(self):
        self.__adjust_cameras()

    def on_L3_x_at_rest(self):
        self.__adjust_cameras()

    def on_L3_y_at_rest(self):
        self.__adjust_cameras()

    def on_L1_press(self):
        pass

    def on_L1_release(self):
        self.__move_camera(0, False)

    def on_L2_press(self, val=None):
        pass

    def on_L2_release(self, val=None):
        self.__move_camera(0, True)

    def on_R1_press(self):
        pass

    def on_R1_release(self):
        self.__move_camera(1, False)

    def on_R2_press(self, val=None):
        pass

    def on_R2_release(self, val=None):
        self.__move_camera(1, True)

    def on_R3_up(self, value):
        self.__last_right_direction = ControllerDirection.UP
        self.__last_right_value = value
        
    def on_R3_down(self, value):
        self.__last_right_direction = ControllerDirection.DOWN
        self.__last_right_value = value

    def on_R3_left(self, value):
        self.__last_right_direction = ControllerDirection.LEFT
        self.__last_right_value = value

    def on_R3_right(self, value):
        self.__last_right_direction = ControllerDirection.RIGHT
        self.__last_right_value = value

    def on_R3_release(self):
        self.__adjust_rotation()

    def on_R3_x_at_rest(self):
        self.__adjust_rotation()

    def on_R3_y_at_rest(self):
        self.__adjust_rotation()

    def on_x_release(self):
       self.__exit_method()

    def __execute_movement (self, controller_direction):
        if controller_direction == ControllerDirection.UP:
            logging.getLogger(__name__).info("Drive Forward")
            self.__vehicle.forward_distance(20.0, 10)
        elif controller_direction == ControllerDirection.DOWN:
            logging.getLogger(__name__).info("Drive Reverse")
            self.__vehicle.reverse_distance(20.0, 10)
        elif controller_direction == ControllerDirection.LEFT:
            logging.getLogger(__name__).info("Strafe Left")
            self.__vehicle.strafe_left()
        elif controller_direction == ControllerDirection.RIGHT:
            logging.getLogger(__name__).info("Strafe Right")
            self.__vehicle.strafe_right()

    def __get_rotation (self, controller_direction, value):
        if controller_direction == ControllerDirection.UP:
            return 0.0
        elif controller_direction == ControllerDirection.DOWN:
            return 0.0
        elif controller_direction == ControllerDirection.LEFT:
            return -90.0
        elif controller_direction == ControllerDirection.RIGHT:
            return 90.0
        
        return 0.0

    def __adjust_rotation (self):
        if not self.__is_busy():
            try:
                self.__set_busy()
                rotation_degree = self.__get_rotation(controller_direction=self.__last_right_direction, value=self.__last_right_value)
                logging.getLogger(__name__).info(f"Rotate: {rotation_degree}")
                self.__vehicle.rotate(rotation_degree)

            except Exception as e:
                logging.getLogger(__name__).error(f"Adjust camera failed: {e}")
            self.__set_ready()

    def __adjust_movement (self):
        if not self.__is_busy():
            try:
                self.__set_busy()
                logging.getLogger(__name__).info(f"Moving")
                self.__execute_movement(controller_direction=self.__last_arrow_direction)
            except Exception as e:
                logging.getLogger(__name__).error(f"Adjust movement failed: {e}")
            self.__set_ready()

    def __adjust_cameras (self):
        if not self.__is_busy():
            try:
                self.__set_busy()
                alt_heading_num = None

                # Press or Left means default 33/-33 degree positions
                # Right means 90/-90
                # Up means facing forward
                # down means facing as far back as the cameras go
                if self.__last_left_direction == ControllerDirection.UP:
                    alt_heading_num = 0
                elif self.__last_left_direction == ControllerDirection.RIGHT:
                    alt_heading_num = 1
                elif self.__last_left_direction == ControllerDirection.LEFT:
                    alt_heading_num = None
                elif self.__last_left_direction == ControllerDirection.DOWN:
                    alt_heading_num = 2

                self.__pilot_nav.reposition_cameras (alt_heading_num=alt_heading_num)

            except Exception as e:
                logging.getLogger(__name__).error(f"Adjust camera failed: {e}")
            self.__set_ready()

    def __move_camera(self, cam_id, forward):
        if not self.__is_busy():
            try:
                self.__set_busy()

                cam_configs = self.__vehicle.get_cameras (timeout = 10.0, force_refresh = True)
                if cam_id in cam_configs:
                    curr_rotation = cam_configs[cam_id]['rotation']
                    max_rotation = cam_configs[cam_id]['max_rotation']
                    min_rotation = cam_configs[cam_id]['min_rotation']

                    rotation_increment = 25
                    next_rotation = curr_rotation
                    if forward:
                        if curr_rotation + rotation_increment <= max_rotation:
                            next_rotation += rotation_increment
                        else:
                            next_rotation = max_rotation
                    else:
                        if curr_rotation - rotation_increment >= min_rotation:
                            next_rotation -= rotation_increment
                        else:
                            next_rotation = min_rotation
                    
                    if next_rotation != curr_rotation:
                        logging.getLogger(__name__).info(f"Moving camera {cam_id} to {next_rotation}")
                        self.__vehicle.look (next_rotation, 90, wait_for_result=True)
                        cam_info = self.__vehicle.get_cameras (timeout = 10.0, force_refresh = True)
                        logging.getLogger(__name__).info(f"Cam Positions: {cam_info}")
                    else:
                        logging.getLogger(__name__).info(f"Camera {cam_id} already at limit in that direction")

            except Exception as e:
                logging.getLogger(__name__).error(f"Adjust camera failed: {e}")
            self.__set_ready()

    def __set_busy (self):
        self.__busy = True
        logging.getLogger(__name__).info("busy")
    
    def __set_ready (self):
        self.__busy = False
        logging.getLogger(__name__).info("ready")

    def __is_busy (self):
        return self.__busy
                


