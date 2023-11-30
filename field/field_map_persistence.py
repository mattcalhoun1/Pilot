import json
from field.field_map import FieldMap

# handles reading/writing of field maps to/from json
class FieldMapPersistence:
    def __init__(self):
        pass

    def load_map (self, file_name : str) -> FieldMap:
        saved_map = None
        # Create map
        with open(file_name, 'r') as mapin:
            json_map = json.loads(mapin.read())
            saved_map = FieldMap(
                boundaries=json_map['boundaries'],
                shape=json_map['shape'],
                landmarks=json_map['landmarks'],
                obstacles=json_map['obstacles'],
                search=json_map['search'],
                near_boundaries=json_map['near_boundaries'],
                name='basement_v2'
            )

        return saved_map