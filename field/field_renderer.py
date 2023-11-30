import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import logging
from field.field_map import FieldMap
from field.field_scaler import FieldScaler

# renders an image of the map
class FieldRenderer:
    def __init__(self, field_map : FieldMap, map_scaler : FieldScaler):
        self.__field_map = field_map
        self.__map_scaler = map_scaler
        self.__game_state = {}
    
    def add_game_state (self, player_id, position_history, look_history):
        self.__game_state[player_id] = {
            'position':position_history,
            'look':look_history
        }

    def save_field_image (self, image_file, add_game_state = False, player_id = None, other_players_visible = False):
        fig = self.render_field_image(add_game_state=add_game_state, player_id=player_id, other_players_visible=other_players_visible)

        fig.canvas.print_png(image_file)
        #with open(image_file, 'w') as outfile:
        #    fig.canvas.print_png(outfile)

        #fig.savefig(image_file, dpi=dpi, transparent=True)

    def render_field_image(self, add_game_state = False, player_id = None, other_players_visible = False, width_inches=8, height_inches=8):
        fig, ax = plt.subplots()
        ax.set_xlim(self.__map_scaler.get_scaled_width())
        ax.set_ylim(self.__map_scaler.get_scaled_height())

        #ax.xaxis.set_ticks([])
        #ax.yaxis.set_ticks([])

        fig.set_size_inches(w=width_inches, h=height_inches)
        fig.set_dpi(200)
        self.__draw_boundaries(ax)

        #ax.set_facecolor('black')
        #ax.axis(False)
        #fig.tight_layout()

        if add_game_state:
            self.__draw_game_state(player_id, ax, True)
        if other_players_visible:
            for p in self.__game_state:
                if p != player_id:
                    self.__draw_game_state(p, ax, False)

        # obstacles need to cover all past state data
        self.__draw_obstacles(ax)

        # current player's position is visible over everything
        if add_game_state:
            self.__draw_current_postion(player_id, ax)

        fig.patch.set_visible(False)
        ax.axis('off')

        return fig

    def __get_alpha_based_on_age(self, age, age_max, alpha_min = 0.2, alpha_max = 0.7):
        capped_age = min(age, age_max)
        age_in_pct = capped_age / age_max
        alpha_range = alpha_max - alpha_min

        alpha_offset = (1 - age_in_pct) * alpha_range
        return alpha_min + alpha_offset


    # draws the game state. bool indicates if it's to be shown from this player's perspective
    def __draw_game_state (self, player_id, ax, is_current_player = True):
        # current position is the newest entry in travel history
        stale_history = 10 # how many moves back are considered 'stale'

        alpha_multiplier = 1
        colors = {
            'position':'green',
            'look':'yellow',
            'breadcrumb':'gray'
        }
        if is_current_player == False:
            alpha_multiplier = 0.5
            colors = {
                'position':'blue',
                'look':'lightblue',
                'breadcrumb':'white'
            }



        # draw look histroy
        l_num = 0
        l_history_len = len(self.__game_state[player_id]['look'])
        for x,y,heading,min_angle,max_angle,dist in self.__game_state[player_id]['look']:
            # draw a cone indicating the direciton of the look
            look_age = l_history_len - l_num
            scaled_x, scaled_y = self.__map_scaler.get_scaled_coords(x,y)
            scaled_dist = self.__map_scaler.scale_lvps_distance_to_sim(dist)

            mp_heading = 180 + heading if heading > 0 else 180 - abs(heading)
            wedge_angle_start = min_angle + mp_heading
            wedge_angle_end = max_angle + mp_heading

            look_arc = mpatches.Wedge(
                (scaled_x, scaled_y), 
                r=scaled_dist, # fix this
                fc=colors['look'],
                theta1=wedge_angle_start, # fix this
                theta2=wedge_angle_end, # fix this
                alpha=self.__get_alpha_based_on_age(look_age, stale_history, alpha_min=0.0, alpha_max=0.4)*alpha_multiplier
            )
            ax.add_patch(look_arc)
            l_num += 1

        p_num = 0
        p_history_len = len(self.__game_state[player_id]['position'])
        last_scaled_x = None
        last_scaled_y = None
        for x,y,heading in self.__game_state[player_id]['position']:
            # draw a dot on the position, with alpha indicating how old it is
            position_age = p_history_len - p_num
            scaled_x, scaled_y = self.__map_scaler.get_scaled_coords(x,y)
            pos_circle = mpatches.Circle(
                [scaled_x, scaled_y], 
                radius=3.0, 
                linewidth=0,
                color=colors['position'], 
                alpha=self.__get_alpha_based_on_age(position_age, stale_history, alpha_min=0, alpha_max=1.0)*alpha_multiplier,
                fill=True
            )
            ax.add_patch(pos_circle)

            if last_scaled_x is not None:
                breadcrumb = mpatches.ConnectionPatch((last_scaled_x, last_scaled_y), (scaled_x,scaled_y),
                    "data", "data",
                    arrowstyle ="-|>",
                    shrinkA = 2, shrinkB = 2,
                    mutation_scale = 4, 
                    linestyle=":",
                    alpha=self.__get_alpha_based_on_age(position_age, stale_history, alpha_min=0, alpha_max=0.5)*alpha_multiplier,
                    fc =colors['breadcrumb'])
                
                
                #mpatches.Arrow(
                #    last_scaled_x, 
                #    last_scaled_y, 
                #    scaled_x, 
                #    scaled_y, 
                #    2.0, 
                #    alpha=self.__get_alpha_based_on_age(position_age, stale_history, alpha_min=0.05, alpha_max=0.5))
                ax.add_patch(breadcrumb)

            last_scaled_x = scaled_x
            last_scaled_y = scaled_y
            p_num += 1

    # draws the game state. bool indicates if it's to be shown from this player's perspective
    def __draw_current_postion (self, player_id, ax):
        last_position_index = len(self.__game_state[player_id]['position']) - 1
        if last_position_index >= 0:
            x,y,heading = self.__game_state[player_id]['position'][last_position_index]
            scaled_x, scaled_y = self.__map_scaler.get_scaled_coords(x,y)
            pos_circle = mpatches.Circle(
                [scaled_x, scaled_y], 
                radius=2.0, 
                linewidth=0,
                color='red', 
                alpha=1.0,
                fill=True
            )
            ax.add_patch(pos_circle)

    def __draw_boundaries (self, ax):
        bx_min, by_min, bx_max, by_max = self.__field_map.get_boundaries()

        scaled_bx_min, scaled_by_min = self.__map_scaler.get_scaled_coords(lvps_x=bx_min, lvps_y=by_min)
        scaled_bx_max, scaled_by_max = self.__map_scaler.get_scaled_coords(lvps_x=bx_max, lvps_y=by_max)

        ax.add_patch(mpatches.Rectangle([scaled_bx_min, scaled_by_min], scaled_bx_max - scaled_bx_min, scaled_by_max - scaled_by_min,
             edgecolor = 'white',
             facecolor = 'white',
             fill=True,
             lw=2))
        
    def __draw_obstacles (self, ax):
        for o in self.__field_map.get_obstacles():
            ox_min, oy_min, ox_max, oy_max = self.__field_map.get_obstacle_bounds(o)

            scaled_ox_min, scaled_oy_min = self.__map_scaler.get_scaled_coords(lvps_x=ox_min, lvps_y=oy_min)
            scaled_ox_max, scaled_oy_max = self.__map_scaler.get_scaled_coords(lvps_x=ox_max, lvps_y=oy_max)

            ax.add_patch(mpatches.Rectangle((scaled_ox_min, scaled_oy_min), scaled_ox_max - scaled_ox_min, scaled_oy_max - scaled_oy_min,
                edgecolor = 'gray',
                facecolor = 'gray',
                fill=True,
                lw=1))
