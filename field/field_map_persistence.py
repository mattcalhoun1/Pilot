import json
from field.field_map import FieldMap

# handles reading/writing of field maps to/from json
class FieldMapPersistence:
    def __init__(self):
        pass

    def load_map (self, file_name : str) -> FieldMap:
        saved_map = None
        # Load from file map
        with open(file_name, 'r') as mapin:
            json_map = json.loads(mapin.read())
            saved_map = self.load_map_from_dict(json_map=json_map)

        return saved_map
    
    def load_map_from_dict (self, json_map : dict) -> FieldMap:
        return FieldMap(
                boundaries=json_map['boundaries'] if 'boundaries' in json_map else None,
                shape=json_map['shape'] if 'shape' in json_map else None,
                landmarks=json_map['landmarks'] if 'landmarks' in json_map else None,
                obstacles=json_map['obstacles'] if 'obstacles' in json_map else None,
                search=json_map['search'] if 'search' in json_map else None,
                near_boundaries=json_map['near_boundaries'] if 'near_boundaries' in json_map else None,
                name='dynamic'
        )
            