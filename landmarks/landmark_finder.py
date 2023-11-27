from camera.image_resolution import ImageResolution
from field.field_map import FieldMap
import statistics
import time
import logging

class LandmarkFinder:
    def __init__ (self, camera_config, field_map : FieldMap, apply_smoothing = True, default_object_id_filter = ['light']):
        self.__camera_config = camera_config
        self.__field_map = field_map
        self.__image_resolution = camera_config['IMAGE_RESOLUTION']
        self.__apply_smoothing = apply_smoothing

        self.__landmark_sighting_cache = {}
        self.__max_cached_sightings_per_landmark = 10 
        self.__max_sighting_age = 10.0 # 10 seconds
        self.__default_object_id_filter = default_object_id_filter
        self.__barrel_distortion_at_edge = camera_config['BARREL_DISTORTION_AT_EDGE']


    def get_camera_config (self):
        return self.__camera_config

    def locate_landmarks (self, object_locations, id_filter = None, confidence_threshold = None):
        raw_locations = self.extract_landmarks_from_locations (
            object_locations,
            id_filter = id_filter if id_filter is not None else self.__default_object_id_filter, 
            confidence_threshold = confidence_threshold)

        if self.__apply_smoothing:
            self.__cache_landmarks(raw_sightings=raw_locations)

            # get a smoothed version utilizing the latest, plus what we have in the cache
            smoothed = self.get_smoothed_landmarks(
                all_landmarks=self.__get_raw_looking_landmarks_from_cache(),
                max_ticks_to_use=3,
                allowed_dev=1.7,
                min_sample_size=3)
            if len(smoothed) > 0:
                return smoothed

        return raw_locations

    def __get_raw_looking_landmarks_from_cache (self):
        raw_looking = []
        now = time.time()
        to_trim = [] # which landmark items need trimmed (remove oldest)
        for lid in self.__landmark_sighting_cache:
            for sighting in self.__landmark_sighting_cache[lid]:
                if now - sighting['time'] <= self.__max_sighting_age:
                    raw_looking.append({lid:sighting})
                else:
                    to_trim.append(lid)

        # trim old data
        for trim in to_trim:
            del self.__landmark_sighting_cache[trim][0]
        
        #logging.getLogger(__name__).info(raw_looking)

        return raw_looking
    
    def __cache_landmarks(self, raw_sightings):
        for sight_inst in raw_sightings:
            for lid in sight_inst:
                if lid not in self.__landmark_sighting_cache:
                    self.__landmark_sighting_cache[lid] = []

                if len(self.__landmark_sighting_cache[lid]) >= self.__max_cached_sightings_per_landmark:
                    del self.__landmark_sighting_cache[lid][0]
            
                self.__landmark_sighting_cache[lid].append(sight_inst[lid])

    def extract_landmarks_from_locations (self, object_locations, id_filter = ['light'], confidence_threshold = 0.25):
        # this shoudl be implemented in a subclass
        raise Exception ("Not implemented in subclass!!")

    def get_field_map (self):
        return self.__field_map

    def get_landmark_config (self, landmark_id):
        return self.__field_map.get_landmark_position(landmark_id)

    def get_image_resolution (self):
        return self.__image_resolution

    # uses all available groups and timestamps to calculate best possible distance values
    def get_smoothed_landmarks (self, all_landmarks, max_ticks_to_use = 3, allowed_dev=2.0, min_sample_size=3):
        smoothed_landmarks = {}

        # First remove outliers
        filtered = self.get_landmark_records_minus_outliers(landmark_records=all_landmarks, allowed_dev=allowed_dev, min_sample_size=min_sample_size)
        
        # order by time
        time_sorted = self.get_landmark_locations_by_time (filtered)

        # now get all measurements grouped by timestamp
        landmark_ts_pairs = self.get_landmark_sightings_merged_by_time(
            all_landmark_records=time_sorted,
            fill_missing_values=True,
            match_threshold=1,# must have at least this many instances of a given landmark to try and match
            seconds_rounding_decimal=1,
            max_pair_distance=0.2)

        if len(landmark_ts_pairs) > 0:
            logging.getLogger(__name__).debug("Landmark smoothing applied")
            # now we have a group of measurement group point-in-times keyed by time
            # get the avg of the given max_ticks for each one
            available_times = sorted(landmark_ts_pairs.keys())
            selected_times = available_times[(-1*max_ticks_to_use):] if len(available_times) > max_ticks_to_use else available_times
            #logging.getLogger(__name__).info(f"Available: {available_times}, Selected: {selected_times}")

            all_x1 = {}
            all_y1 = {}
            all_x2 = {}
            all_y2 = {}
            all_corrected_height = {}
            all_conf = {}

            try:
                for time_slice in selected_times:
                    for landmark_sighting in landmark_ts_pairs[time_slice]:
                        lid = landmark_sighting['id']
                        if lid not in all_x1:
                            all_x1[lid] = []
                            all_x2[lid] = []
                            all_y1[lid] = []
                            all_y2[lid] = []
                            all_conf[lid] = []
                            all_corrected_height[lid] = []
                        all_x1[lid].append(landmark_sighting['x1'])
                        all_x2[lid].append(landmark_sighting['x2'])
                        all_y1[lid].append(landmark_sighting['y1'])
                        all_y2[lid].append(landmark_sighting['y2'])
                        all_conf[lid].append(landmark_sighting['confidence'])
                        all_corrected_height[lid].append(landmark_sighting['corrected_height'])

                for lid in all_x1:
                    avgd_grp = {
                        'id':lid,
                        'time':selected_times[-1]
                    }

                    if len(all_x1[lid]) > 1:
                        avgd_grp['x1'] = statistics.mean(all_x1[lid])
                        avgd_grp['x2'] = statistics.mean(all_x2[lid])
                        avgd_grp['y1'] = statistics.mean(all_y1[lid])
                        avgd_grp['y2'] = statistics.mean(all_y2[lid])
                        avgd_grp['confidence'] = statistics.mean(all_conf[lid])
                        avgd_grp['corrected_height'] = statistics.mean(all_corrected_height[lid])
                    else: # just get first one
                        avgd_grp['x1'] = all_x1[lid][0]
                        avgd_grp['x2'] = all_x2[lid][0]
                        avgd_grp['y1'] = all_y1[lid][0]
                        avgd_grp['y2'] = all_y2[lid][0]
                        avgd_grp['confidence'] = all_conf[lid][0]
                        avgd_grp['corrected_height'] = all_corrected_height[lid][0]

                    smoothed_landmarks[lid] = avgd_grp
            except Exception as e:
                # this can happin if emitters go out of view
                logging.getLogger(__name__).warning(f"Landmark smoothing failed {e}")

        return smoothed_landmarks


    # the lists of gruops should be ordered such that the oldest is last in the array
    def get_landmark_sightings_merged_by_time (self, all_landmark_records, fill_missing_values, match_threshold = 1, seconds_rounding_decimal = 2, max_pair_distance = 0.2, max_time_slices = 100, max_time_slices_per_landmark = 20):
        # landmarks are a map to lists of point-in-time landmark sighting records
        # if there is more than one landmark present, we can create some matched records
        # to improve calculations

        # first we want the biggest, that will usually be the most accurate
        tallest, widest = self.get_tallest_and_widest_visible(all_landmark_records=all_landmark_records)

        if tallest is None or widest is None:
            return {}

        processed = []
        process_order = [tallest,widest] + list(all_landmark_records.keys())
        total_slices = 0

        records_by_time = {}
        records_by_time_and_id = {}

        for lid in process_order:
            if lid not in processed:
                records_by_time_and_id[lid] = {}
                processed.append(lid)
                for i,meas_group in enumerate(reversed(all_landmark_records[lid])):
                    if i < max_time_slices_per_landmark:
                        this_time = round(meas_group['time'], seconds_rounding_decimal)
                        if this_time not in records_by_time:
                            # dont add too many
                            if total_slices <= max_time_slices:
                                records_by_time[this_time] = [meas_group]
                                total_slices += 1
                        else:
                            records_by_time[this_time].append(meas_group)

                        if this_time not in records_by_time_and_id[lid]:
                            records_by_time_and_id[lid][this_time] = meas_group

        completed_records = {}

        # if we are told to fill missing values, start merging
        if fill_missing_values:
            # for each timestamp, if it doesn't have a value for a given group id, attempt to find a close one
            num_landmarks = len(all_landmark_records.keys())

            # known timestamps for each groups, sorted, so we only do the sort once
            landmark_timestamps = {}
            for lid in all_landmark_records:
                landmark_timestamps[lid] = sorted(records_by_time_and_id[lid].keys())

            for ts in records_by_time:
                if len(records_by_time[ts]) == num_landmarks:
                    completed_records[ts] = records_by_time[ts]
                else:
                    for lid in all_landmark_records.keys():
                        present_groups = [g['id'] for g in records_by_time[ts]]
                        if lid not in present_groups:
                            # find the nearest record
                            closest = landmark_timestamps[lid][-1]
                            for available in landmark_timestamps[lid]:
                                if abs(ts - available) < abs(ts - closest):
                                    closest = available
                                elif available > closest:
                                    break

                            # if it's not too far, copy this one
                            if abs(ts - closest) <= max_pair_distance:
                                records_by_time[ts].append(records_by_time_and_id[lid][closest])

                # if the number of matches now meets the threshold, add this timestamp to the completed records
                if len(records_by_time[ts]) >= match_threshold:
                    completed_records[ts] = records_by_time[ts]

        elif match_threshold > 1:
            # do not include timestamps that have less than the given threshold
            for ts in records_by_time:
                if len(records_by_time[ts]) >= match_threshold:
                    completed_records[ts] = records_by_time[ts]
        else:
            completed_records = records_by_time

        # now we have a map of hundredths of second time slices to lists of measurement groups corresponding to those times
        # merge them in a sensible way
        return completed_records

    
    # return the gropu id for whichever appears tallest now, and whichever appears widest now
    # this expects the locations grouped id, sorted by time
    def get_tallest_and_widest_visible (self, all_landmark_records, oldest_allowed = 5.0):
        all_landmark_records_copy = all_landmark_records

        if len(all_landmark_records_copy) == 0:
            return None,None

        # we expect lists of group records keyed by group id. if we just have individual records by group id, fix that
        if isinstance(all_landmark_records[[*all_landmark_records.keys()][0]], dict):
            all_landmark_records_copy = {}
            for lid in all_landmark_records:
                all_landmark_records_copy[lid] = [all_landmark_records[lid],]

        tallest_avg = 0
        tallest_avg_group_id = None

        widest_avg = 0
        widest_avg_group_id = None

        for grp in all_landmark_records_copy:
            heights = []
            widths = []
            for point_in_time in all_landmark_records_copy[grp]:
                heights.append(self.get_landmark_height(point_in_time))
                widths.append(self.get_landmark_width(point_in_time))

            avg_height = statistics.mean(heights)
            avg_width = statistics.mean(widths)

            if avg_height > tallest_avg:
                tallest_avg = avg_height
                tallest_avg_group_id = grp

            if avg_width > widest_avg:
                widest_avg = avg_width
                widest_avg_group_id = grp

        return tallest_avg_group_id, widest_avg_group_id

    # rearranges the map of landmark records, which has records repeated
    # so it is a map of landmark IDs to lists of locations
    def get_landmark_locations_by_time (self, landmark_records):
        sorted_landmarks = {}
        for l_instances in landmark_records:
            for lid in l_instances:
                this_landmark_sighting = l_instances[lid]

                if lid not in sorted_landmarks:
                    sorted_landmarks[lid] = [ this_landmark_sighting ]
                else:
                    # insert this sighting , newest first
                    insert_location = None
                    for i,sightings in enumerate(sorted_landmarks[lid]):
                        if sorted_landmarks[lid][i]['time'] < this_landmark_sighting['time']:
                            insert_location = i
                            break
                    if insert_location is None:
                        sorted_landmarks[lid].append(this_landmark_sighting)
                    else:
                        sorted_landmarks[lid].insert(insert_location, this_landmark_sighting)
        return sorted_landmarks

    # for a given group, returns all the measurements for it that dont
    # seem to be outliers
    def get_landmark_records_minus_outliers (self, landmark_records, allowed_dev = 1.5, min_sample_size = 3):
        # remove any outliers
        filtered = []

        landmark_vals = {}

        if len(landmark_records) > min_sample_size:
            for landmark_inst in landmark_records:
                for lid in landmark_inst:
                    if lid not in landmark_vals:
                        landmark_vals[lid] = {}
                        landmark_vals[lid]['x_vals'] = []
                        landmark_vals[lid]['y_vals'] = []
                        landmark_vals[lid]['viz_heights'] = []
                        landmark_vals[lid]['viz_widths'] = []

                    r = landmark_inst[lid]
                    x,y = self.get_landmark_center(r)
                    landmark_vals[lid]['x_vals'].append(x)
                    landmark_vals[lid]['y_vals'].append(y)
                    landmark_vals[lid]['viz_heights'].append(self.get_landmark_height(r))
                    landmark_vals[lid]['viz_widths'].append(self.get_landmark_width(r))

            for lid in landmark_vals:
                # dont try calcuating averages if there are not enough samples
                if len(landmark_vals[lid]['x_vals']) > 1:
                    landmark_vals[lid]['x_std'] = statistics.stdev(landmark_vals[lid]['x_vals'])
                    landmark_vals[lid]['x_mean'] = statistics.mean(landmark_vals[lid]['x_vals'])
                    landmark_vals[lid]['y_std'] = statistics.stdev(landmark_vals[lid]['y_vals'])
                    landmark_vals[lid]['y_mean'] = statistics.mean(landmark_vals[lid]['y_vals'])
                    landmark_vals[lid]['h_std'] = statistics.stdev(landmark_vals[lid]['viz_heights'])
                    landmark_vals[lid]['h_mean'] = statistics.mean(landmark_vals[lid]['viz_heights'])
                    landmark_vals[lid]['w_std'] = statistics.stdev(landmark_vals[lid]['viz_widths'])
                    landmark_vals[lid]['w_mean'] = statistics.mean(landmark_vals[lid]['viz_widths'])

            for landmark_inst in landmark_records:
                for lid in landmark_inst:
                    r = landmark_inst[lid]
                    x,y = self.get_landmark_center(r)
                    h = self.get_landmark_height(r)
                    w = self.get_landmark_width(r)

                    # if there wasnt enough of a sample size for this type, do not filter
                    if len(landmark_vals[lid]['x_vals']) > min_sample_size:
                        if abs(x - landmark_vals[lid]['x_mean']) <= (landmark_vals[lid]['x_std'] * allowed_dev) and abs(y - landmark_vals[lid]['y_mean']) <= (landmark_vals[lid]['y_std'] * allowed_dev):
                            if abs(h - landmark_vals[lid]['h_mean']) <= (landmark_vals[lid]['h_std'] * allowed_dev) and abs(w - landmark_vals[lid]['w_mean']) <= (landmark_vals[lid]['w_std'] * allowed_dev):
                                filtered.append({lid:r})
                    else:
                        filtered.append({lid:r})

        else:
            return landmark_records
        
        return filtered

    def get_landmark_center (self, landmark):
        return abs(landmark['x2'] - landmark['x1']) / 2 + landmark['x1'],abs(landmark['y2'] - landmark['y1']) / 2 + landmark['y1']

    def get_landmark_height (self, landmark):
        if 'corrected_height' in landmark:
            return landmark['corrected_height']
        return abs(landmark['y2'] - landmark['y1'])

    def get_landmark_width (self, landmark):
        if 'corrected_width' in landmark:
            return landmark['corrected_width']
        return abs(landmark['x2'] - landmark['x1'])

    def get_height_distortion_multiplier_at_x (self, x):
        # get the center's distance from y center of image
        image_x_center = self.get_image_resolution().get_width() / 2
        dist_from_image_center = abs(x - image_x_center)
        distortion_multiplier = (dist_from_image_center / image_x_center) * self.__barrel_distortion_at_edge
        #logging.getLogger(__name__).info(f"Applying edge size correction of {distortion_multiplier} percent")
        return distortion_multiplier