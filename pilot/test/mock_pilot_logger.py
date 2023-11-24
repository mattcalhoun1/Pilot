from pilot.pilot_logger import PilotLogger
class MockPilotLogger (PilotLogger):
    def __init__(self):
        pass
    
    def log_lidar (self, lidar_map):
        return True
        
    
    def log_coordinates (self, map_id, x, y, heading, basis):
        return None

    def log_coordinates_and_images (self, map_id, x, y, heading, images, basis):
        pass

    def log_search_hit (self, map_id, object_type, est_visual_distance, est_lidar_dist, 
                        vehicle_relative_heading, est_x, est_y, vehicle_x, vehicle_y, vehicle_heading, 
                        confidence, camera_id, camera_angle, image_file, image_format):
        pass