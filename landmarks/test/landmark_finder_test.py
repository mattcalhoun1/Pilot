import unittest
from field.field_map import FieldMap
from landmarks.landmark_finder import LandmarkFinder
from camera.camera_info import CameraInfo
from camera.image_resolution import ImageResolution
import logging
import statistics

class TestLandmarkFinder(unittest.TestCase):
    def setUp(self) -> None:
        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
        return super().setUp()

    def __landmark_locations_with_outliers (self):
        return [
            # n1, 1 outlier
            {'n1':{'id':'n1', 'x1':50.01,'y1':100.0, 'x2':250.0, 'y2':200.0, 'time':101.12, 'confidence': 0.5}},
            {'n1':{'id':'n1', 'x1':50.1,'y1':100.01, 'x2':250.01, 'y2':200.01, 'time':101.01, 'confidence': 0.5}},
            {'n1':{'id':'n1', 'x1':50.05,'y1':100.02, 'x2':250.02, 'y2':200.02, 'time':101.2, 'confidence': 0.5}},
            {'n1':{'id':'n1', 'x1':50.09,'y1':100.01, 'x2':250.05, 'y2':200.01, 'time':101.3, 'confidence': 0.5}},

            {'n1':{'id':'n1', 'x1':50,'y1':150, 'x2':230, 'y2':350, 'time':101.4, 'confidence': 0.5}},

            # n2 , no outliers
            {'n2':{'id':'n2', 'x1':151.01,'y1':101.0, 'x2':250.3, 'y2':300.01, 'time':101.01, 'confidence': 0.5}},
            {'n2':{'id':'n2', 'x1':150.02,'y1':100.9, 'x2':251.0, 'y2':301.0, 'time':101.11, 'confidence': 0.5}},
            {'n2':{'id':'n2', 'x1':150.04,'y1':100.8, 'x2':249.8, 'y2':300.4, 'time':101.21, 'confidence': 0.5}},
            {'n2':{'id':'n2', 'x1':149.98,'y1':101.1, 'x2':250.1, 'y2':300.5, 'time':101.31, 'confidence': 0.5}},

        ]

    def test_get_smoothed_landmarks (self):
        finder = self.__get_finder()
        landmarks = self.__landmark_locations_with_outliers()

        smoothed_data = finder.get_smoothed_landmarks (landmarks, allowed_dev = 1.75, min_sample_size = 3, max_ticks_to_use = 3)

        #logging.getLogger(__name__).info(f"{smoothed_data}")

        # we shoudl get an avg of the last 3 instances for both
        self.assertEqual(len(smoothed_data), 2)

        self.assertEqual(101.3, smoothed_data['n1']['time'])
        self.assertEqual(101.3, smoothed_data['n2']['time'])

        self.assertEqual(round(smoothed_data['n1']['x1'],1), round(statistics.mean([50.01, 50.05, 50.09]),1))
        self.assertEqual(round(smoothed_data['n2']['y2'],1), round(statistics.mean([301.0, 300.4, 300.5]),1))

    def test_get_landmark_sightings_merged_by_time_tenths (self):
        finder = self.__get_finder()
        landmarks = self.__landmark_locations_with_outliers()

        minus_outliers = finder.get_landmark_records_minus_outliers(
            landmark_records=landmarks,
            allowed_dev=1.75, 
            min_sample_size=3
        )
        time_sorted = finder.get_landmark_locations_by_time (minus_outliers)

        landmark_sightings_merged = finder.get_landmark_sightings_merged_by_time (
            all_landmark_records = time_sorted,
            fill_missing_values = True,
            match_threshold = 1,
            seconds_rounding_decimal = 1, # group by nearest hundredth of second
            max_pair_distance = 0.3, # don't pair sightings > .3 seconds apart,
            max_time_slices = 100,
            max_time_slices_per_landmark = 20)
        
        # we should have 4 time slices
        # Each landmark sighting had a close pair
        self.assertEqual(4, len(landmark_sightings_merged))

        # check a couple of this slices
        self.assertEqual(len(landmark_sightings_merged[101.0]),2)
        self.assertEqual(len(landmark_sightings_merged[101.1]),2)

        #logging.getLogger(__name__).info(landmark_sightings_merged)

        n1 = None
        n2 = None
        for landmark in landmark_sightings_merged[101.1]:
            if landmark['id'] == 'n1':
                n1 = landmark
            elif landmark['id'] == 'n2':
                n2 = landmark
        self.assertEqual(n1['x1'],50.01)
        self.assertEqual(n2['x1'],150.02)

        n1 = None
        n2 = None
        for landmark in landmark_sightings_merged[101.3]:
            if landmark['id'] == 'n1':
                n1 = landmark
            elif landmark['id'] == 'n2':
                n2 = landmark
        self.assertEqual(n1['x1'],50.09)
        self.assertEqual(n2['x1'],149.98)


    def test_get_landmark_sightings_merged_by_time_hundredths (self):
        finder = self.__get_finder()
        landmarks = self.__landmark_locations_with_outliers()

        minus_outliers = finder.get_landmark_records_minus_outliers(
            landmark_records=landmarks,
            allowed_dev=1.75, 
            min_sample_size=3
        )
        time_sorted = finder.get_landmark_locations_by_time (minus_outliers)

        landmark_sightings_merged = finder.get_landmark_sightings_merged_by_time (
            all_landmark_records = time_sorted,
            fill_missing_values = True,
            match_threshold = 1,
            seconds_rounding_decimal = 2, # group by nearest hundredth of second
            max_pair_distance = 0.1, # don't pair sightings > .1 seconds apart,
            max_time_slices = 100,
            max_time_slices_per_landmark = 20)
        
        # we should have almost double the number.
        # only overlap is the 101.01 time slice
        # total will be 7 time slices
        self.assertEqual(7, len(landmark_sightings_merged))

        # check a couple of this slices
        self.assertEqual(len(landmark_sightings_merged[101.01]),2)

        #logging.getLogger(__name__).info(landmark_sightings_merged)

        n1 = None
        n2 = None
        for landmark in landmark_sightings_merged[101.01]:
            if landmark['id'] == 'n1':
                n1 = landmark
            elif landmark['id'] == 'n2':
                n2 = landmark
        self.assertEqual(n1['x1'],50.1)
        self.assertEqual(n2['x1'],151.01)

        n1 = None
        n2 = None
        for landmark in landmark_sightings_merged[101.11]:
            if landmark['id'] == 'n1':
                n1 = landmark
            elif landmark['id'] == 'n2':
                n2 = landmark
        self.assertEqual(n1['x1'],50.01)
        self.assertEqual(n2['x1'],150.02)


    def test_remove_outliers (self):
        finder = self.__get_finder()
        landmarks = self.__landmark_locations_with_outliers()

        # there is a single outlier in the set

        minus_outliers = finder.get_landmark_records_minus_outliers(
            landmark_records=landmarks,
            allowed_dev=1.75, 
            min_sample_size=3
        )

        self.assertEqual(len(landmarks)-1, len(minus_outliers))

    def test_get_tallest_and_widest(self):
        finder = self.__get_finder()
        landmarks = self.__landmark_locations_with_outliers()
        minus_outliers = finder.get_landmark_records_minus_outliers(
            landmark_records=landmarks,
            allowed_dev=1.75, 
            min_sample_size=3
        )
        time_sorted = finder.get_landmark_locations_by_time (minus_outliers)

        tallest, widest = finder.get_tallest_and_widest_visible (time_sorted, oldest_allowed = 5.0)

        self.assertEqual(widest, 'n1')
        self.assertEqual(tallest, 'n2')
        

    def test_get_locations_by_time(self):
        finder = self.__get_finder()
        landmarks = self.__landmark_locations_with_outliers()
        time_sorted = finder.get_landmark_locations_by_time (landmarks)

        self.assertEqual(2, len(time_sorted))

        self.assertEqual(5, len(time_sorted['n1']))
        self.assertEqual(101.4, time_sorted['n1'][0]['time'])
        self.assertEqual(4, len(time_sorted['n2']))
        self.assertEqual(101.31, time_sorted['n2'][0]['time'])

    def __get_finder (self):
        camera_config = CameraInfo.CameraConfig['PI3_STD_HQ_CROPPED_0']

        field_map = FieldMap({
                'n1': {
                    'pattern':'3',
                    'type':'light',
                    'model':'lights',
                    'x':26,
                    'y':132,
                    'height':43.5,
                    #'extension_length':24,
                    'center_altitude':40
                },
                'n2': {
                    'pattern':'2',
                    'type':'light',
                    'model':'lights',
                    'x':25,
                    'y':51,
                    'height':11.5,
                    #'extension_length':25,
                    'center_altitude':12.5
                },
                # -35, -12 
            })
        return LandmarkFinder(
            camera_config=camera_config,
            field_map=field_map
        )     