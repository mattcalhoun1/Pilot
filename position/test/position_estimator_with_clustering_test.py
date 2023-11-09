import unittest
from position.position_estimator_with_clustering import PositionEstimatorWithClustering
from position.confidence import Confidence
from position.estimator_mode import EstimatorMode
from field.field_map import FieldMap
import logging
import json

class TestPositionEstimatorWithClustering(unittest.TestCase):
    def setUp(self) -> None:
        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
        return super().setUp()

    def get_alt_map (self):
        return FieldMap( 
            landmarks= {
                "n_light": {
                    "pattern":"3",
                    "type":"light",
                    "model":"lights",
                    "x":26,
                    "y":132,
                    "height":43,
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
                "nw_light": {
                    "pattern":"4",
                    "type":"light",
                    "model":"lights",
                    "x":-112,
                    "y":130,
                    "height":42,
                    "altitude":21,
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
            },
            )

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
        
        #estimator = PositionEstimatorWithClustering(curr_map, horizontal_fov = 62.2, vertical_fov = 48.8, view_width=1640.0, view_height=1232.0)
        estimator = PositionEstimatorWithClustering(curr_map, horizontal_fov = 62.2, vertical_fov = 48.8, view_width=1640.0, view_height=1232.0)        
        possible = estimator.get_possible_headings (x=5, y=-62.0, view_angles=view_angles)
        
        # pointed in a direction left of north (~ -30 degrees)
        self.assertGreater(len(possible), 0)
        for p in possible:
            self.assertLess(p, 0)
        logging.getLogger(__name__).info(f"Possible: {possible}")

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
        
        estimator = PositionEstimatorWithClustering(curr_map, horizontal_fov = 62.2, vertical_fov = 48.8, view_width=1640.0, view_height=1232.0)
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
        
        estimator = PositionEstimatorWithClustering(curr_map, horizontal_fov = 62.2, vertical_fov = 48.8, view_width=1640.0, view_height=1232.0)
        possible = estimator.get_possible_headings (x=5, y=-62.0, view_angles=view_angles)
        
        # pointed in a direction left of north (~ -30 degrees)
        self.assertGreater(len(possible), 0)
        for p in possible:
            self.assertGreater(p, 0)
        logging.getLogger(__name__).info(f"Possible: {possible}")

        
    def test_coords_and_heading (self):
        curr_map = self.get_alt_map()

        # the second instance of n1 should not be taken into account for
        # distances, since it has lower confidence
        located_objects =  [
            {
                'e_light': {
                    'id': 'e_light', 'time': 1692628315.9, 'x1': 668.4199168682098, 'x2': 708.1502503156662, 
                    'y1': 373.9696774482727, 'y2': 478.90106439590454, 'confidence': 0.4475695, 'camera_heading': 156.0
                }
            }, {
                'n_light': {
                    'id': 'n_light', 'time': 1692628317.9, 'x1': 1484.8546743392944, 'x2': 1523.4735455513, 
                    'y1': 137.253227353096, 'y2': 485.55761194229126, 'confidence': 0.35439932, 'camera_heading': 24.0
                }
            }, {
                'nw_light': {
                    'id': 'nw_light', 'time': 1692628317.9, 'x1': 497.9999496936798, 'x2': 531.7095794677734, 
                    'y1': 357.876118183136, 'y2': 615.5538032054901, 'confidence': 0.3126791, 'camera_heading': 24.0
                }
            }]

        # get visual degrees for each point
        #estimator = PositionEstimatorWithClustering(curr_map, horizontal_fov = 62.2, vertical_fov = 48.8, view_width=1640.0, view_height=1232.0)
        estimator = PositionEstimatorWithClustering(curr_map, horizontal_fov = 71.0, vertical_fov = 49.4, view_width=1640.0, view_height=1232.0)
        x, y, heading, confidence, basis = estimator.get_coords_and_heading (located_objects = located_objects,  view_altitude = 8.75, estimator_mode = EstimatorMode.FAST)

        # actual coordinates are 6, -50 with heading of around 30
        self.assertGreaterEqual(x,0)
        self.assertLessEqual(x,70)

        self.assertGreaterEqual(y,-100)
        self.assertLessEqual(y,-25)

        self.assertGreaterEqual(heading,6)
        self.assertLessEqual(heading,40)
        self.assertEqual(Confidence.CONFIDENCE_MEDIUM, confidence)

        logging.getLogger(__name__).info(f"FAST : ({x},{y} - Heading {heading})")

        #x, y, heading, confidence = estimator.get_coords_and_heading (located_objects,  41.75, estimator_mode = EstimatorMode.PRECISE)
        #logging.getLogger(__name__).info(f"PRECISE : ({x},{y} - Heading {heading})")

        #x, y, heading, confidence = estimator.get_coords_and_heading (located_objects,  41.75, estimator_mode = EstimatorMode.VERY_PRECISE)
        #logging.getLogger(__name__).info(f"VERY PRECISE : ({x},{y} - Heading {heading})")

