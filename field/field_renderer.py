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
        self.__agent_state = {}
        self.__search_state = {}

    def get_map_scaler (self):
        return self.__map_scaler
    
    def update_agent_state (self, agent_id, position_history, look_history):
        self.__agent_state[agent_id] = {
            'position':position_history,
            'look':look_history
        }

    def update_search_state (self, agent_id, target_type, estimated_x, estimated_y):
        if target_type not in self.__search_state:
            self.__search_state[target_type] = []
        self.__search_state[target_type].append(estimated_x, estimated_y, agent_id)

    def save_field_image (self, image_file, add_game_state = False, agent_id = None, other_agents_visible = False):
        fig = self.render_field_image(add_game_state=add_game_state, agent_id=agent_id, other_agents_visible=other_agents_visible)

        fig.canvas.print_png(image_file)
        plt.close()
        #with open(image_file, 'w') as outfile:
        #    fig.canvas.print_png(outfile)

        #fig.savefig(image_file, dpi=dpi, transparent=True)

    def render_field_image(self, add_game_state = False, agent_id = None, other_agents_visible = False, width_inches=8, height_inches=8):
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
            self.__draw_game_state(agent_id, ax, True)
        if other_agents_visible:
            for p in self.__agent_state:
                if p != agent_id:
                    self.__draw_game_state(p, ax, False)

        # obstacles need to cover all past state data
        self.__draw_obstacles(ax)

        # current agent's position is visible over everything
        if add_game_state:
            self.__draw_current_postion(agent_id, ax)

        fig.patch.set_visible(False)
        ax.axis('off')

        return fig

    def __get_alpha_based_on_age(self, age, age_max, alpha_min = 0.2, alpha_max = 0.7):
        capped_age = min(age, age_max)
        age_in_pct = capped_age / age_max
        alpha_range = alpha_max - alpha_min

        alpha_offset = (1 - age_in_pct) * alpha_range
        return alpha_min + alpha_offset


    # draws the game state. bool indicates if it's to be shown from this agent's perspective
    def __draw_game_state (self, agent_id, ax, is_current_agent = True):
        # current position is the newest entry in travel history
        stale_history = 10 # how many moves back are considered 'stale'

        alpha_multiplier = 1
        colors = {
            'position':'green',
            'look':'yellow',
            'breadcrumb':'gray'
        }
        if is_current_agent == False:
            alpha_multiplier = 0.5
            colors = {
                'position':'blue',
                'look':'lightblue',
                'breadcrumb':'white'
            }



        # draw look histroy
        if agent_id in self.__agent_state:
            if 'look' in self.__agent_state[agent_id]:
                l_num = 0
                l_history_len = len(self.__agent_state[agent_id]['look'])
                for x,y,heading,min_angle,max_angle,dist in self.__agent_state[agent_id]['look']:
                    # draw a cone indicating the direciton of the look
                    look_age = l_history_len - l_num
                    scaled_x, scaled_y = self.__map_scaler.get_scaled_coords(x,y)
                    scaled_dist = self.__map_scaler.scale_lvps_distance_to_sim(dist)

                    mp_heading = 90 - heading # this gets our heading in terms of matplotlib

                    # now we want to face the reverse (what we want to be facing the empty part of the wedge)
                    if mp_heading > 180:
                        mp_heading -= 180
                    else:
                        mp_heading += 180

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

            if 'position' in self.__agent_state[agent_id]:
                p_num = 0
                p_history_len = len(self.__agent_state[agent_id]['position'])
                last_scaled_x = None
                last_scaled_y = None
                for x,y,heading,conf in self.__agent_state[agent_id]['position']:
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

    # draws the game state. bool indicates if it's to be shown from this agent's perspective
    def __draw_current_postion (self, agent_id, ax):
        if agent_id in self.__agent_state and 'position' in self.__agent_state[agent_id]:
            last_position_index = len(self.__agent_state[agent_id]['position']) - 1
            if last_position_index >= 0:
                x,y,heading,confidence = self.__agent_state[agent_id]['position'][last_position_index]
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