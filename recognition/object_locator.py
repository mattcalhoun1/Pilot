
from camera.camera import Camera

class ObjectLocator:

    def find_objects_on_camera(self, camera : Camera, object_filter : list = None, min_confidence : float = 0.4) -> list:
        pass

    def find_objects_in_image(self, image, object_filter : list = None, min_confidence : float = 0.4) -> list:
        pass

    def read_label_file(self, file_path):
        with open(file_path, 'r') as f:
            lines = f.readlines()
        ret = {}
        for line in lines:
            pair = line.strip().split(maxsplit=1)
            ret[int(pair[0])] = pair[1].strip()
        return ret

    def get_latest_image (self):
        raise Exception("Not implemented in subclass")
