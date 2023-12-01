from field.field_renderer import FieldRenderer
from field.field_map_persistence import FieldMapPersistence
from field.field_scaler import FieldScaler
from position.confidence import Confidence
import logging

def get_game_state ():
    return {
        'MecCar': {
            'position' : [
                (-50.0, 100.0, -44.0, Confidence.CONFIDENCE_HIGH),
                (-20.0, 40.0, -120.0, Confidence.CONFIDENCE_HIGH),
                (10.0, 0.0, 44.0, Confidence.CONFIDENCE_HIGH),
                (20.0, 20.0, 33.0, Confidence.CONFIDENCE_HIGH),
                (22.0, -5.0, -100.0, Confidence.CONFIDENCE_HIGH)
            ],
            'look': [
                (-20.0, 40.0, -120.0, -160.0, 160.0, 50.0),
                (10.0, 0.0, 44.0, -160.0, 160.0, 50.0),
                (22.0, -5.0, -100.0, -160.0, 160.0, 50.0)
            ],
        },
        'Tank': {
            'position' : [
                (-100.0, 200.0, -170.0, Confidence.CONFIDENCE_HIGH),
                (-75.0, 180.0, -46.0, Confidence.CONFIDENCE_HIGH),
                (-10.0, 120.0, 170.0, Confidence.CONFIDENCE_HIGH),
            ],
            'look': [
                (-100.0, 200.0, -170.0, -160.0, 160.0, 50.0),
                (-75.0, 180.0, -46.0, -160.0, 160.0, 50.0),
                (-10.0, 120.0, 170.0, -160.0, 160.0, 50.0),
            ],
        }
    }    


if __name__ == '__main__':
    logging.basicConfig(format='%(asctime)s [%(levelname)s] %(module)s:%(message)s', level=logging.INFO)
    field_map = FieldMapPersistence().load_map('/home/matt/projects/LVPS_Simulation/lvps/visual/resources/basement_v2.json')

    # we want to leave 10% around the edges in case a robot goes off map
    # the rendered field will be square
    rendered_height = 640
    rendered_width = 640

    max_scaled_side_length = (min(rendered_height, rendered_width)) * .9
    max_map_side = max(field_map.get_width(), field_map.get_length())

    scale_factor = 1
    while ((max_map_side * scale_factor) > max_scaled_side_length):
        scale_factor -= 0.05

    logging.getLogger(__name__).info(f"Map will be scaled down to {scale_factor}")

    scaler = FieldScaler(field_map=field_map, scaled_height=rendered_height, scaled_width=rendered_width, scale_factor=scale_factor, invert_x_axis=True, invert_y_axis=True)
    renderer = FieldRenderer(field_map=field_map, map_scaler=scaler)

    gs = get_game_state()
    for v in gs:
        renderer.update_agent_state (v, gs[v]['position'], gs[v]['look'])

    # render the game state from the MecCar's perspective
    renderer.save_field_image('/tmp/map.png', add_game_state=True, agent_id='MecCar', other_agents_visible=True)

