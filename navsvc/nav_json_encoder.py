import json
import numpy as np
from datetime import datetime

# Recursively encodes (for json) a dict/list
# which may include numpy float32s, and other 
# non-json-friendly types, prior to sending to nav svc
class NavJsonEncoder(json.JSONEncoder):
    def encode(self, obj, raw = False):
        altered = obj
        if isinstance(obj, float):
            return round(obj, 2)
        elif isinstance(obj, np.float32):
            return round(obj.item(), 2)
        elif isinstance(obj, datetime):
            return datetime.strftime(obj, '%Y-%m-%d %H:%M:%S')
        elif isinstance(obj, list):
            new_list = []
            for item in obj:
                new_list.append(self.encode(item, True))
            altered = new_list
        elif isinstance(obj, dict):
            new_dict = {}
            for k in obj:
                new_dict[k] = self.encode(obj[k], True)
            altered = new_dict

        if raw == False:
            return json.JSONEncoder.encode(self, altered)
        else:
            return altered