import arcade
import math
from visualization.utils import create_arc_outline as cao
import numpy as np

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

SPRITE_SCALING_CAR = 0.25
INIT_CENTER_X = SCREEN_WIDTH/2
INIT_CENTER_Y = SCREEN_HEIGHT/2


class SlotCarGame(arcade.Window):
    space_pressed = False

    def __init__(self, width, height, track):
        super().__init__(width, height)
        self.track = track
        self.car_sprites = arcade.SpriteList()
        self.track_bounds = self.track.get_track_bounds()
        arcade.set_background_color(arcade.color.WHITE)
        

    def setup_track(self):
        straight_track_coordinates,turn_track_coordinated =\
        self.track.get_track_coordinates()
        self.track_element_list = arcade.ShapeElementList()

        for coord in straight_track_coordinates:
            coord[:2] = self.transform(*coord[:2])
            coord[2:4] = self.transform(*coord[2:])
            shape = arcade.create_line(*coord,arcade.color.BLACK)
            self.track_element_list.append(shape)

        for coord in turn_track_coordinated:
            coord[:2] = self.transform(*coord[:2])
            coord[2:4] = self.scale_length(coord[2]), self.scale_length(coord[3])
            shape = cao.create_arc_outline(*coord[0:4],arcade.color.BLACK,*coord[4:6])
            self.track_element_list.append(shape)
        
    
    # def setup(self):
        
    #     for car in track.cars:
    #         car_sprite = arcade.Sprite('visualization/images/car.png', SPRITE_SCALING_CAR)
    #         car_sprite.center_x = INIT_CENTER_X
    #         car_sprite.center_y = INIT_CENTER_Y
        

    def setup(self):
        self.setup_track()
        for car in self.track.cars:
            car_sprite = arcade.Sprite('visualization/images/car.png', SPRITE_SCALING_CAR)
            car_sprite.center_x, car_sprite.center_y = self.transform(0, 0)
            self.car_sprites.append(car_sprite)

    def transform(self, x, y):
        """
        Take car and track coordinates, and calculate to pixel coordinates.
        """
        coordinate = np.array([x, y])
        difference = (self.track_bounds[1] - self.track_bounds[0])
        max_diff = difference.max()
        normalized = (coordinate - self.track_bounds[0]) / max_diff
        # TODO calculate padding for the screen in a general way.
        return normalized * min(SCREEN_WIDTH, SCREEN_HEIGHT)

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

    def update(self, delta_time):
        self.track.step(delta_time)

        for i, car_sprite in enumerate(self.car_sprites):
            car = self.track.cars[i]
            car_sprite.center_x, car_sprite.center_y = self.transform(car.x, car.y)
            car_sprite.angle = car.yaw

    def on_key_press(self, symbol: int, modifiers: int):
        """
        if 49 <= symbol <= 57:
            speed = symbol - 47
        else:
            speed = 0

        for car in self.track.cars:
            car.speed = speed / 9 * Car.MAX_SPEED
        """


def start_game(track):
    game = SlotCarGame(SCREEN_WIDTH, SCREEN_HEIGHT, track)
    game.setup()
    arcade.run()
