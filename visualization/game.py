import arcade
import math
import numpy as np
import random
from visualization.utils import create_arc_outline
from model.car import Car

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

CAR_LENGTH = 14e-2
DEFAULT_YAW = -90

class SlotCarGame(arcade.Window):
    space_pressed = False

    def __init__(self, width, height, track):
        super().__init__(width, height, title='Slot Car Racer')

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
        self.best_times = {} # Record times for each car
        self.global_time = 0  # The total time since game start
        self.previous_rails = {} # Previous rail for car

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
        sprites = random.sample(['ambulance', 'audi', 'black_viper', 'car', 'mini_truck',
                                 'mini_van', 'police', 'taxi', 'truck'], len(self.track.cars))

        self.explosions_list = arcade.SpriteList()
        for i, car in enumerate(self.track.cars):
            car_sprite = arcade.Sprite(f'visualization/images/{sprites[i]}.png')
            car_sprite.scale = (CAR_LENGTH / car_sprite.height) * self.get_scaling_factor()
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

    def get_scaling_factor(self):
        max_diff = (self.track_bounds[1] - self.track_bounds[0]).max()
        return min(SCREEN_WIDTH, SCREEN_HEIGHT) / max_diff

    def scale_length(self, length):
        """ Scale a length from the car/track length system, to a length in
        pixels corresponding to the transform method. """
        return length * self.get_scaling_factor()

    def on_draw(self):
        arcade.start_render()
        self.track_element_list.draw()
        for car in self.car_sprites:
            car.draw()
        self.explosions_list.draw()
        self.draw_time()

    def draw_time(self):
        color = {-1: arcade.color.BLUE, 1: arcade.color.RED}

        for i, car in enumerate(reversed(self.track.cars)):
            lap_time = self.seconds_to_string(self.round_timer[car], decimals=.5)
            best_time = round(self.best_times[car], 3) \
                if car in self.best_times else 'N/A'
            s = f"Lap time for car {i + 1}: {lap_time} (Best: {best_time})"
            arcade.draw_text(s, 3, 15 * i + 3, color[car.lane], 12)

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

    def update(self, _):
        # Instead of letting Arcade control the frame rate, we manually run at
        # 60 fps in order to prevent the frame rate from affecting the physics.
        delta_time = 1 / 60

        self.global_time += delta_time

        for car in self.track.cars:
            # If this car is AI-controlled, update the input with the new state.
            if car.controller and not car.is_crashed:
                car.controller_input = car.controller.step()

        steps_per_frame = 1
        for i in range(steps_per_frame):
            self.track.step(delta_time/steps_per_frame)

        for i, car_sprite in enumerate(self.car_sprites):
            car = self.track.cars[i]

            car_sprite.center_x, car_sprite.center_y = self.transform(car.pos_vec[0], car.pos_vec[1])
            car_sprite.angle = car.phi * 180 / np.pi + DEFAULT_YAW

            if car.is_crashed:
                if i not in self.cars_crashed:
                    self.cars_crashed.append(i)
                    explosion = Explosion(self.explosion_texture_list)
                    explosion.center_x, explosion.center_y = self.transform(car.pos_vec[0], car.pos_vec[1])
                    explosion.update()
                    self.explosions_list.append(explosion)
                else:
                    self.explosions_list.update()
                    for explosion in self.explosions_list:
                        explosion.center_x, explosion.center_y = self.transform(car.pos_vec[0], car.pos_vec[1])
            else:
                self.explosions_list.update()
                if i in self.cars_crashed:
                    self.cars_crashed.remove(i)

            self.round_time_step(self.track.cars[i], delta_time)

    def round_time_step(self, car: Car, delta_time):
        self.round_timer[car] += delta_time

        if car in self.previous_rails and self.previous_rails[car] == self.track.rails[-1] \
                and car.rail == self.track.rails[0]:
            # Lap completed, only if speed is nonzero.
            if np.linalg.norm(car.vel_vec) == 0:
                return

            new_time = self.round_timer[car]
            self.best_times[car] = min(self.best_times.get(car, math.inf), new_time)
            self.round_times[car].append(new_time)
            self.round_timer[car] = 0  # Reset clock

        self.previous_rails[car] = car.rail

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
        """
        if 49 == symbol:
            increment = 0.01
        elif symbol == 57:
            increment = -0.01
        else:
            increment = 0

        for car in self.track.cars:
            if car.key_control:
                if not car.is_crashed:
                    new_input = car.controller_input + increment
                    if new_input < 0:
                        new_input = 0
                    if new_input > 1:
                        new_input = 1
                    car.controller_input = new_input
        """


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
