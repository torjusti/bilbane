import arcade
import math
import numpy as np
import random
from visualization.utils import create_arc_outline
from model.car import Car

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

SPRITE_SCALING_CAR = 0.25
DEFAULT_YAW = -90

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
            shape = create_arc_outline.create_arc_outline(*coord[0:4], arcade.color.BLACK,*coord[4:6])
            self.track_element_list.append(shape)

    def setup(self):
        self.setup_track()
        sprites = random.sample(['ambulance', 'audi', 'black_viper', 'car', 'mini_truck',
                                 'mini_van', 'police', 'taxi', 'truck'], len(self.track.cars))

        self.explosions_list = arcade.SpriteList()
        for i, car in enumerate(self.track.cars):
            car_sprite = arcade.Sprite(f'visualization/images/{sprites[i]}.png', SPRITE_SCALING_CAR)
            car_sprite.center_x, car_sprite.center_y = self.transform(0, 0)
            self.car_sprites.append(car_sprite)

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

    def update(self, delta_time):
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
