import unittest
import logging
import numpy as np
import json
from navsvc.nav_json_encoder import NavJsonEncoder
from datetime import datetime

class TestPilotNavigation(unittest.TestCase):
    def setUp(self) -> None:
        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
        return super().setUp()

    def testNestedNumpyExample (self):
        test = {
            'key1':'val1',
            'key2': {
                'subkey1':1,
                'subkey2':2
            },
            'key3': {
                'sk1':[np.float32(123.45), np.float32(456.45)]
            },
            'key4': {
                'sk4.1': np.float32(123.45)
            },
            'key5': datetime.now()
        }

        json_str = json.dumps(test, cls=NavJsonEncoder, indent=2)
        logging.getLogger(__name__).info(json_str)