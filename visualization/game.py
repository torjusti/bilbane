import arcade
import numpy as np

from model.car import Car
from visualization.utils import create_arc_outline

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

SPRITE_SCALING_CAR = 0.25


class SlotCarGame(arcade.Window):
    space_pressed = False

    def __init__(self, width, height, track):
        super().__init__(width, height)

        self.track = track
        self.car_sprites = arcade.SpriteList()
        self.track_bounds = self.track.get_track_bounds()
        self.explosions_list = None

        self.explosion_texture_list = []
        self.cars_crashed = []

        columns = 8
        count = 51
        sprite_width = 256
        sprite_height = 256
        file_name = 'visualization/images/spritesheet.png'

        # Load the explosions from a sprite sheet
        self.explosion_texture_list = arcade.load_spritesheet(file_name, sprite_width,
                                                              sprite_height, columns, count)

        arcade.set_background_color(arcade.color.WHITE)

        self.round_timer = {}  # Car to current round time
        self.round_times = {}  # Car to list of round times
        self.global_time = 0  # The total time since game start

    def setup_track(self):
        self.track_element_list = arcade.ShapeElementList()

        for coord in self.track.straight_track_coordinates:
            coord[:2] = self.transform(*coord[:2])
            coord[2:4] = self.transform(*coord[2:])
            shape = arcade.create_line(*coord, arcade.color.BLACK)
            self.track_element_list.append(shape)

        for coord in self.track.turn_track_coordinates:
            coord[:2] = self.transform(*coord[:2])
            coord[2:4] = self.scale_length(coord[2]), self.scale_length(coord[3])
            shape = create_arc_outline.create_arc_outline(*coord[0:4], arcade.color.BLACK, *coord[4:6])
            self.track_element_list.append(shape)

    def setup(self):
        self.setup_track()
        self.explosions_list = arcade.SpriteList()
        for car in self.track.cars:
            car_sprite = arcade.Sprite('visualization/images/car.png', SPRITE_SCALING_CAR)
            car_sprite.center_x, car_sprite.center_y = self.transform(0, 0)
            self.car_sprites.append(car_sprite)
            self.round_timer[car] = 0
            self.round_times[car] = []

    def transform(self, x, y):
        """ Take car and track coordinates, and calculate to pixel coordinates. """
        coordinate = np.array([x, y])
        difference = (self.track_bounds[1] - self.track_bounds[0])
        max_diff = difference.max()
        normalized = (coordinate - self.track_bounds[0]) / max_diff
        # TODO: Calculate magic number 0.1 in a better way.
        return (normalized + 0.1) * min(SCREEN_WIDTH, SCREEN_HEIGHT)

    def scale_length(self, length):
        """ Scale a length from the car/track length system, to a length in
        pixels corresponding to the transform method. """
        max_diff = (self.track_bounds[1] - self.track_bounds[0]).max()
        normalized = length / max_diff
        return normalized * min(SCREEN_WIDTH, SCREEN_HEIGHT)

    def on_draw(self):
        arcade.start_render()
        self.track_element_list.draw()
        for car in self.car_sprites:
            car.draw()
        self.explosions_list.draw()
        self.draw_time()

    def draw_time(self):
        color = {-1: arcade.color.BLUE, 1: arcade.color.RED}
        best_times = {}
        for i, car in enumerate(reversed(self.track.cars)):
            s = f"{self.seconds_to_string(self.round_timer[car], decimals=.5)}"
            arcade.draw_text(s, 0, 12 * i, color[car.lane], 12)
            times = self.round_times[car]
            if times:
                best_times[car] = min(times)

        if best_times.keys():
            sort = sorted(best_times.items(), key=lambda e: e[1])
            for i, (car, time) in enumerate(sort):
                s = f"{self.seconds_to_string(best_times[car], decimals=3)}"
                arcade.draw_text(s, 0, self.height - 12 * (i + 2), color[car.lane], 12)

    @staticmethod
    def seconds_to_string(seconds, decimals=1.0):
        minutes = int(seconds) // 60
        sec = int(seconds) % 60
        if decimals == 0.5:
            # Round down to half second
            milli = int((seconds - (sec + minutes * 60)) * 1000)
            decimals = 1
            milli = 0 if milli < 500 else 5
        else:
            milli = int((seconds - (sec + minutes * 60))*10 ** decimals)

        return f"{minutes:02d}:{sec:02d}:{milli:0{int(decimals)}d}"

    def update(self, delta_time):
        self.track.step(delta_time)
        self.global_time += delta_time

        for i, car_sprite in enumerate(self.car_sprites):
            car = self.track.cars[i]
            car_sprite.center_x, car_sprite.center_y = self.transform(car.x, car.y)
            car_sprite.angle = car.phi

            if car.is_crashed:
                if i not in self.cars_crashed:
                    self.cars_crashed.append(i)
                    explosion = Explosion(self.explosion_texture_list)
                    explosion.center_x, explosion.center_y = self.transform(car.x, car.y)
                    explosion.update()
                    self.explosions_list.append(explosion)
                else:
                    self.explosions_list.update()
                    for explosion in self.explosions_list:
                        explosion.center_x, explosion.center_y = self.transform(car.x, car.y)
            else:
                self.explosions_list.update()
                if i in self.cars_crashed:
                    self.cars_crashed.remove(i)

            self.round_time_step(car, delta_time)

    def round_time_step(self, car: Car, delta_time):
        self.round_timer[car] += delta_time

        if car.rail_progress == 0 and car.rail == self.track.rails[0]:
            # Lap completed, only if speed is nonzero.
            if np.linalg.norm(car.vel_vec) == 0:
                return
            new_time = self.round_timer[car]
            self.round_times[car].append(new_time)
            self.round_timer[car] = 0  # Reset clock

    def on_key_press(self, symbol: int, modifiers: int):
        """
        Numbers from 1-9 corresponds to lowest speed setting, to max speed setting. Every other key is zero speed.
        """
        if 49 <= symbol <= 57:
            speed = symbol - 48
        else:
            speed = 0

        for car in self.track.cars:
            if car.key_control:
                if not car.is_crashed:
                    car.controller_input = speed / 9


class Explosion(arcade.Sprite):
    """ This class creates an explosion animation """

    def __init__(self, texture_list): #fix an explosion to a car
        super().__init__()

        # Start at the first frame
        self.current_texture = 0
        self.textures = texture_list


    def update(self):
        # Update to the next frame of the animation. If we are at the end
        # of our frames, then delete this sprite.
        self.current_texture += 1
        if self.current_texture < len(self.textures):
            self.set_texture(self.current_texture)
        else:
            self.remove_from_sprite_lists()

def start_game(track):
    game = SlotCarGame(SCREEN_WIDTH, SCREEN_HEIGHT, track)
    game.setup()
    arcade.run()
