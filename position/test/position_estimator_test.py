import unittest
from position.position_estimator import PositionEstimator
from position.confidence import Confidence
from position.estimator_mode import EstimatorMode
from field.field_map import FieldMap
import logging
import json

class TestPositionEstimator(unittest.TestCase):
    def setUp(self) -> None:
        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
        return super().setUp()

    def get_basic_map (self):
        return FieldMap( {
            'n1': {
                'x':-40.0,
                'y':38.0,
                'height':24.0,
                'width':2.5,
                'altitude':22.75
            },
            'n2': {
                'x':-106.0,
                'y':-79.0,
                'height':44.0,
                'width':2.5,
                'altitude':40.5
            },
            'e1': {
                'x':152.0,
                'y':-220.0,
                'height':24.0,
                'width':2.5,
                'altitude':22.75
            }
        })

    def get_alt_map (self):
        return FieldMap( 
            landmarks= {
                "n_light": {
                    "pattern":"3",
                    "type":"light",
                    "model":"lights",
                    "x":26,
                    "y":132,
                    "height":43.5,
                    "altitude":40,
                    "confidence":0.25
                },
				"e_light": {
				    "pattern":"2",
				    "type":"light",
				    "model":"lights",
				    "x":136,
				    "y":-28,
				    "height":11,
				    "altitude":29,
		                    "confidence":0.25
				},                
                "e_ball": {
                    "pattern":"na",
                    "type":"gazing_ball",
                    "model":"basement",
                    "x":72,
                    "y":1,
                    "height":10.5,
                    "altitude":5.25,
                    "confidence":0.6
                },
                "w_tree": {
                    "pattern":"na",
                    "type":"cat_tree",
                    "model":"basement",
                    "x":-93,
                    "y":-52,
                    "height":24.5,
                    "altitude":12.25,
                            "confidence":0.6
                },   
				"w_house": {
				    "pattern":"na",
				    "type":"house",
				    "model":"basement",
				    "x":-57,
				    "y":1,
				    "height":7.75,
				    "altitude":3.875,
		                    "confidence":0.6
				},                
            },
            shape="rectangle",
            boundaries = {
                "xmin":-50,
                "ymin":-150,
                "xmax":100,
                "ymax":0
            },
            near_boundaries = {
                "xmin":-100,
                "ymin":-170,
                "xmax":120,
                "ymax":10
            })


    def test_extract_angles (self):
        curr_map = self.get_basic_map()

        located_objects = [
            {'n1':{'x1':1162.5, 'y1':321.76, 'x2':1209.53, 'y2':516.0, 'confidence':0.4, 'camera_heading':70.0}},
            {'n2':{'x1':308.05, 'y1':135.06, 'x2':352.7, 'y2':545.3, 'confidence':0.4, 'camera_heading':70.0}},
            {'e1':{'x1':819.0, 'y1':378.0, 'x2':864.0, 'y2':767.0, 'confidence':0.4, 'camera_heading':170.0}},

        ]

        # get visual degrees for each point
        estimator = PositionEstimator(curr_map, horizontal_fov = 44.0, vertical_fov = 27.333, view_width=1280.0, view_height=720.0)

        angles = estimator.extract_object_view_angles(located_objects=located_objects)
        # Expect:
        #    {
        #    "n1": [
        #        {
        #        "height_pix": 194.24,
        #        "height_deg": 7.373836,
        #        "width_pix": 47.02999999999997,
        #        "width_deg": 1.6166562499999992,
        #        "confidence": 0.4
        #        }
        #    ],
        #    "n2": [
        #        {
        #        "height_pix": 410.23999999999995,
        #        "height_deg": 15.573735999999997,
        #        "width_pix": 44.64999999999998,
        #        "width_deg": 1.5348437499999994,
        #        "confidence": 0.4
        #        }
        #    ]
        #    }
        self.assertIn('n1', angles)
        self.assertEqual(1, len(angles['n1']))
        self.assertEqual(7.4, round(angles['n1'][0]['height_deg'],1))
        self.assertEqual(1.6, round(angles['n1'][0]['width_deg'],1))
        self.assertIn('n2', angles)
        self.assertEqual(1, len(angles['n2']))
        self.assertEqual(15.6, round(angles['n2'][0]['height_deg'],1))
        self.assertEqual(1.5, round(angles['n2'][0]['width_deg'],1))
        self.assertIn('e1', angles)

        #logging.getLogger(__name__).info(json.dumps(angles, indent=2))

        # make sure the relative px/degrees were added and correct
        self.assertIn('n1', angles['n2'][0]['relative_px'])
        self.assertEqual(29.41, round(angles['n2'][0]['relative_deg']['n1'],2))
        self.assertEqual(88.16, round(angles['n1'][0]['relative_deg']['e1'],2))
        self.assertIn('n1', angles['n2'][0]['relative_deg'])
        self.assertEqual(-117.57, round(angles['e1'][0]['relative_deg']['n2'],2))


    def test_extract_distances (self):
        # and also visual degrees for distance between each point
        curr_map = self.get_basic_map()

        # the second instance of n1 should not be taken into account for
        # distances, since it has lower confidence
        located_objects = [
            {'n1':{'x1':1162.5, 'y1':321.76, 'x2':1209.53, 'y2':516.0, 'confidence':0.4, 'camera_heading':70.0}},
            {'n2':{'x1':308.05, 'y1':135.06, 'x2':352.7, 'y2':545.3, 'confidence':0.4, 'camera_heading':70.0}},
            {'n1':{'x1':5.5, 'y1':23.76, 'x2':6.53, 'y2':50.0, 'confidence':0.3, 'camera_heading':70.0}},
        ]

        # get visual degrees for each point
        estimator = PositionEstimator(curr_map, horizontal_fov = 44.0, vertical_fov = 27.333, view_width=1280.0, view_height=720.0)

        angles = estimator.extract_object_view_angles(located_objects=located_objects)
        distances = estimator.extract_distances(angles, 41.75)

        # Expect:
        #    {
        #    "n1": {
        #        "ground": 184.7625649951914,
        #        "top": 184.89512006432818,
        #        "bottom": 187.34515052064287
        #    },
        #    "n2": {
        #        "ground": 160.9884311076136,
        #        "top": 162.32016957387287,
        #        "bottom": 162.43391610698
        #    }
        #    }
        self.assertEqual(round(distances['n1']['ground'],1),184.8)
        self.assertEqual(round(distances['n1']['top'],1),184.9)
        self.assertEqual(round(distances['n1']['bottom'],1),187.3)        

        self.assertEqual(round(distances['n2']['ground'],1),161.0)
        self.assertEqual(round(distances['n2']['top'],1),162.3)
        self.assertEqual(round(distances['n2']['bottom'],1),162.4)        

    def test_coordinate_is_possible (self):
        # and also visual degrees for distance between each point
        curr_map = self.get_basic_map()

        # the second instance of n1 should not be taken into account for
        # distances, since it has lower confidence
        located_objects = [
            {'n1':{'x1':1162.5, 'y1':321.76, 'x2':1209.53, 'y2':516.0, 'confidence':0.4, 'camera_heading':70.0}},
            {'n2':{'x1':308.05, 'y1':135.06, 'x2':352.7, 'y2':545.3, 'confidence':0.4, 'camera_heading':70.0}},
            #{'n1':{'x1':5.5, 'y1':23.76, 'x2':6.53, 'y2':50.0, 'confidence':0.3, 'camera_heading':70.0}},
            {'e1':{'x1':819.0, 'y1':378.0, 'x2':864.0, 'y2':767.0, 'confidence':0.4, 'camera_heading':180.0}},
        ]

        # get visual degrees for each point
        estimator = PositionEstimator(curr_map, horizontal_fov = 44.0, vertical_fov = 27.333, view_width=1280.0, view_height=720.0)

        angles = estimator.extract_object_view_angles(located_objects=located_objects)


        # actual coords , given this info, are (31,-221)
        # expected visual angle n1/e1 = 105
        # expected visual angle n1/n2 = 29
        # expected visual angle n2/e1 = 133
        self.assertTrue(estimator.is_possible(31,-221,angles))

        #logging.getLogger(__name__).info('~'*20)

        # not possible
        self.assertFalse(estimator.is_possible(-221,31,angles))

        # also not possible
        self.assertFalse(estimator.is_possible(50,50,angles))



    def test_estimate_coordinates (self):
        # and also visual degrees for distance between each point
        curr_map = self.get_basic_map()

        # the second instance of n1 should not be taken into account for
        # distances, since it has lower confidence
        located_objects = [
            {'n1':{'x1':1162.5, 'y1':321.76, 'x2':1209.53, 'y2':516.0, 'confidence':0.4, 'camera_heading':0.0}},
            {'n2':{'x1':308.05, 'y1':135.06, 'x2':352.7, 'y2':545.3, 'confidence':0.4, 'camera_heading':0.0}},
            {'n1':{'x1':5.5, 'y1':23.76, 'x2':6.53, 'y2':50.0, 'confidence':0.3, 'camera_heading':0.0}},
            {'e1':{'x1':819.0, 'y1':378.0, 'x2':864.0, 'y2':767.0, 'confidence':0.4, 'camera_heading':180.0}},
        ]

        # get visual degrees for each point
        estimator = PositionEstimator(curr_map, horizontal_fov = 44.0, vertical_fov = 27.333, view_width=1280.0, view_height=720.0)

        angles = estimator.extract_object_view_angles(located_objects=located_objects)
        #logging.getLogger(__name__).info(f"ANGLES: {json.dumps(angles, indent=3)}")        
        distances = estimator.extract_distances(angles, 41.75)
        #logging.getLogger(__name__).info(f"DIST: {json.dumps(distances, indent=3)}")        
        coords = estimator.find_possible_coordinates(
            view_angles=angles, 
            distances=distances, 
            allowed_variance=0.5, 
            allowed_heading_variance = 0.09,
            estimator_mode=EstimatorMode.FAST)

        #logging.getLogger(__name__).info(json.dumps(coords, indent=3))

        #heading = estimator.get_heading(x, )

        # actual coords , given this info, are (31,-221)
        # expected visual angle n1/e1 = 105
        # expected visual angle n1/n2 = 29
        # expected visual angle n2/e1 = 133

        # what is the average
        #x = [p[0] for p in coords]
        #y = [p[1] for p in coords]
        #centroid = (sum(x) / len(coords), sum(y) / len(coords))
        #logging.getLogger(__name__).info(f"Centroid: {centroid}")


    def test_get_heading (self):
        # and also visual degrees for distance between each point
        curr_map = self.get_basic_map()

        # the second instance of n1 should not be taken into account for
        # distances, since it has lower confidence
        located_objects = [
            {'n1':{'x1':1162.5, 'y1':321.76, 'x2':1209.53, 'y2':516.0, 'confidence':0.4, 'camera_heading':0.0}},
            {'n2':{'x1':308.05, 'y1':135.06, 'x2':352.7, 'y2':545.3, 'confidence':0.4, 'camera_heading':0.0}},
            {'n1':{'x1':5.5, 'y1':23.76, 'x2':6.53, 'y2':50.0, 'confidence':0.3, 'camera_heading':0.0}},
            {'e1':{'x1':819.0, 'y1':378.0, 'x2':864.0, 'y2':767.0, 'confidence':0.4, 'camera_heading':180.0}},
        ]

        # get visual degrees for each point
        estimator = PositionEstimator(curr_map, horizontal_fov = 44.0, vertical_fov = 27.333, view_width=1280.0, view_height=720.0)
        angles = estimator.extract_object_view_angles(located_objects=located_objects)
 
        heading = estimator.get_heading(x=31.0, y=-221.0, angles=angles)

        self.assertEqual(35, round(heading))

    def test_get_possible_headings_one_cam (self):
        curr_map = self.get_alt_map()
        view_angles =  {
            'n_light': [{
                'center_x': 543.7956695556641, 'center_y': 386.95453655719757, 'height_pix': 295.76466155052185, 'height_deg': 11.71535347700119, 
                'width_pix': 26.913687229156494, 'width_deg': 1.0207508205204474, 'confidence': 0.2915846, 'image_heading': 143.5, 
                'relative_deg': {
                    'e_ball': 38.879695415293305
                }, 
                'relative_px': {
                    'e_ball': 1025.12380194664
                }
            }],
            'e_ball': [{
                'center_x': 1568.919471502304, 'center_y': 671.9952927827835, 'height_pix': 163.3990285396576, 'height_deg': 6.472299182415008, 
                'width_pix': 133.17504405975342, 'width_deg': 5.050907158851624, 'confidence': 0.99997115, 'image_heading': 143.5, 
                'relative_deg': {
                    'n_light': -38.879695415293305
                },
                'relative_px': {
                    'n_light': -1025.12380194664
                }
            }]}
        
        #    'FOV_H': 62.2,
        #    'FOV_V': 48.8,
        #    'CLASS': 'CV2Camera',
        #    'HIGHRES':True,
        #    'FLIPPED':True,
        #    'DEFAULT_FOCUS':5,
        #    'AUTO_OPTIMIZE':True,
        #    'IMAGE_RESOLUTION':ImageResolution(1640.0,1232.0)        
        
        estimator = PositionEstimator(curr_map, horizontal_fov = 62.2, vertical_fov = 48.8, view_width=1640.0, view_height=1232.0)
        possible = estimator.get_possible_headings (x=5, y=-62.0, view_angles=view_angles)
        
        # pointed in a direction left of north (~ -30 degrees)
        self.assertGreater(len(possible), 0)
        for p in possible:
            self.assertLess(p, 0)
        #logging.getLogger(__name__).info(f"Possible: {possible}")

    def test_get_possible_headings_two_cam_facing_w (self):
        curr_map = self.get_alt_map()
        view_angles =  {
            'n_light': [{
                'center_x': 543.1892178058624, 'center_y': 386.53448647260666, 'height_pix': 295.97507441043854, 'height_deg': 11.723688012361526, 
                'width_pix': 28.498242378234863, 'width_deg': 1.080847973125737, 'confidence': 0.3024889, 'image_heading': 143.5, 
                'relative_deg': {'e_ball': 38.93780536959811, 'w_tree': -94.44849948276833}, 
                'relative_px': {'e_ball': 1026.6559615135193, 'w_tree': -2490.281979931512}
            }], 
            'e_ball': [{
                'center_x': 1569.8451793193817, 'center_y': 673.1834007501602, 'height_pix': 160.3851945400238, 'height_deg': 6.352920043468475,
                'width_pix': 132.1736764907837, 'width_deg': 5.012928462028504, 'confidence': 0.99996865, 'image_heading': 143.5,
                'relative_deg': {'n_light': -38.93780536959811, 'w_tree': -133.38630485236644}, 
                'relative_px': {'n_light': -1026.6559615135193, 'w_tree': -3516.9379414450314}
            }],
            'w_tree': [{
                'center_x': 874.1291028261185, 'center_y': 513.8776059150696, 'height_pix': 332.4282693862915, 'height_deg': 13.16761326789856, 
                'width_pix': 270.7974445819855, 'width_deg': 10.270488446950912, 'confidence': 0.9999937, 'image_heading': 36.5, 
                'relative_deg': {'n_light': 94.44849948276833, 'e_ball': 133.38630485236644}, 
                'relative_px': {'n_light': 2490.281979931512, 'e_ball': 3516.9379414450314}
            }]
        }
        
        #    'FOV_H': 62.2,
        #    'FOV_V': 48.8,
        #    'CLASS': 'CV2Camera',
        #    'HIGHRES':True,
        #    'FLIPPED':True,
        #    'DEFAULT_FOCUS':5,
        #    'AUTO_OPTIMIZE':True,
        #    'IMAGE_RESOLUTION':ImageResolution(1640.0,1232.0)        
        
        estimator = PositionEstimator(curr_map, horizontal_fov = 62.2, vertical_fov = 48.8, view_width=1640.0, view_height=1232.0)
        possible = estimator.get_possible_headings (x=5, y=-62.0, view_angles=view_angles)
        
        # pointed in a direction left of north (~ -30 degrees)
        self.assertGreater(len(possible), 0)
        for p in possible:
            self.assertLess(p, 0)
        #logging.getLogger(__name__).info(f"Possible: {possible}")

    def test_get_possible_headings_two_cam_facing_e (self):
        curr_map = self.get_alt_map()
        view_angles =  {
                'e_light': [{
                    'center_x': 783.043719470501, 'center_y': 417.2680324316025, 'height_pix': 100.59967064857483, 'height_deg': 3.9847921490669247, 
                    'width_pix': 29.597980618476868, 'width_deg': 1.122557557603208, 'confidence': 0.36424, 'image_heading': 143.5, 
                    'relative_deg': {'e_ball': -26.370195172220107, 'w_house': -121.81967641548775}, 
                    'relative_px': {'e_ball': -695.2913196533918, 'w_house': -3211.9657447170403}
                    }],
                'e_ball': [{
                    'center_x': 87.75239981710911, 'center_y': 646.1304459571838, 'height_pix': 164.56817245483398, 'height_deg': 6.518609428405761, 
                    'width_pix': 158.47885057330132, 'width_deg': 6.0106003083288675, 'confidence': 0.99890065, 'image_heading': 143.5, 
                    'relative_deg': {'e_light': 26.370195172220107, 'w_house': -95.44948124326764}, 
                    'relative_px': {'e_light': 695.2913196533918, 'w_house': -2516.6744250636484}
                }],
                'w_house': [{
                    'center_x': 392.2998397052288, 'center_y': 641.7328970432281, 'height_pix': 126.37001848220825, 'height_deg': 5.005565667152404, 
                    'width_pix': 176.35370522737503, 'width_deg': 6.688536868989468, 'confidence': 0.9999349, 'image_heading': 36.5, 
                    'relative_deg': {'e_light': 121.81967641548775, 'e_ball': 95.44948124326764}, 
                    'relative_px': {'e_light': 3211.9657447170403, 'e_ball': 2516.6744250636484}}
                ]}
        
        #    'FOV_H': 62.2,
        #    'FOV_V': 48.8,
        #    'CLASS': 'CV2Camera',
        #    'HIGHRES':True,
        #    'FLIPPED':True,
        #    'DEFAULT_FOCUS':5,
        #    'AUTO_OPTIMIZE':True,
        #    'IMAGE_RESOLUTION':ImageResolution(1640.0,1232.0)        
        
        estimator = PositionEstimator(curr_map, horizontal_fov = 62.2, vertical_fov = 48.8, view_width=1640.0, view_height=1232.0)
        possible = estimator.get_possible_headings (x=5, y=-62.0, view_angles=view_angles)
        
        # pointed in a direction left of north (~ -30 degrees)
        self.assertGreater(len(possible), 0)
        for p in possible:
            self.assertGreater(p, 0)
        #logging.getLogger(__name__).info(f"Possible: {possible}")

    def test_not_enough_located_objects (self):
        curr_map = self.get_basic_map()

        # the second instance of n1 should not be taken into account for
        # distances, since it has lower confidence
        located_objects = [
            #{'n1':{'x1':1162.5, 'y1':321.76, 'x2':1209.53, 'y2':516.0, 'confidence':0.4, 'camera_heading':70.0}},
            #{'n2':{'x1':308.05, 'y1':135.06, 'x2':352.7, 'y2':545.3, 'confidence':0.4, 'camera_heading':70.0}},
            {'e1':{'x1':819.0, 'y1':378.0, 'x2':864.0, 'y2':767.0, 'confidence':0.4, 'camera_heading':180.0}},
        ]

        # get visual degrees for each point
        estimator = PositionEstimator(curr_map, horizontal_fov = 44.0, vertical_fov = 27.333, view_width=1280.0, view_height=720.0)
        x, y, heading, confidence, basis = estimator.get_coords_and_heading (located_objects,  41.75)

        self.assertIsNone(x)
        self.assertIsNone(y)
        self.assertIsNone(heading)

        #logging.getLogger(__name__).info(f"X:{x},y:{y},heading:{heading}")

        
    def Xtest_coords_and_heading (self):
        curr_map = self.get_basic_map()

        # the second instance of n1 should not be taken into account for
        # distances, since it has lower confidence
        located_objects = [
            {'n1':{'x1':1162.5, 'y1':321.76, 'x2':1209.53, 'y2':516.0, 'confidence':0.4, 'camera_heading':70.0}},
            {'n2':{'x1':308.05, 'y1':135.06, 'x2':352.7, 'y2':545.3, 'confidence':0.4, 'camera_heading':70.0}},
            #{'n1':{'x1':5.5, 'y1':23.76, 'x2':6.53, 'y2':50.0, 'confidence':0.3, 'camera_heading':70.0}},
            {'e1':{'x1':819.0, 'y1':378.0, 'x2':864.0, 'y2':767.0, 'confidence':0.4, 'camera_heading':180.0}},
        ]

        # get visual degrees for each point
        estimator = PositionEstimator(curr_map, horizontal_fov = 44.0, vertical_fov = 27.333, view_width=1280.0, view_height=720.0)

        max_retries = 8
        found_solution = False
        for r in range(max_retries):
            x, y, heading, confidence, basis = estimator.get_coords_and_heading (located_objects,  41.75)
            if x is not None:
                found_solution = True
                # actual coordinates are 31, -221 with heading of around 11
                self.assertGreaterEqual(x,-50)
                self.assertLessEqual(x,90)

                self.assertGreaterEqual(y,-300)
                self.assertLessEqual(y,-100)

                self.assertGreaterEqual(heading,-8)
                self.assertLessEqual(heading,21)
                self.assertEqual(Confidence.CONFIDENCE_MEDIUM, confidence)

                logging.getLogger(__name__).info(f"FAST : ({x},{y} - Heading {heading})")
                break
        
        if found_solution == False:
            self.fail(f"No possible coords found in {max_retries} retries")
        #x, y, heading, confidence = estimator.get_coords_and_heading (located_objects,  41.75, estimator_mode = EstimatorMode.PRECISE)
        #logging.getLogger(__name__).info(f"PRECISE : ({x},{y} - Heading {heading})")

        #x, y, heading, confidence = estimator.get_coords_and_heading (located_objects,  41.75, estimator_mode = EstimatorMode.VERY_PRECISE)
        #logging.getLogger(__name__).info(f"VERY PRECISE : ({x},{y} - Heading {heading})")
