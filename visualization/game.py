import arcade
import numpy as np

from model.model import Car
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
        for car in self.track.cars:
            car_sprite = arcade.Sprite('visualization/images/car.png', SPRITE_SCALING_CAR)
            car_sprite.center_x, car_sprite.center_y = self.transform(0, 0)
            self.car_sprites.append(car_sprite)
            self.round_timer[car] = 0
            self.round_times[car] = []

    def transform(self, x, y):
        """
        Take car and track coordinates, and calculate to pixel coordinates.
        """
        coordinate = np.array([x, y])
        difference = (self.track_bounds[1] - self.track_bounds[0])
        max_diff = difference.max()
        normalized = (coordinate - self.track_bounds[0]) / max_diff
        # TODO calculate magic number 0.1 in a better way.
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

        self.draw_time()

    def draw_time(self):
        arcade.draw_text(self.seconds_to_string(self.global_time), 0, 0, arcade.color.BLACK, 12)
        color = {0: arcade.color.BLUE, 1: arcade.color.RED}

        for i, car in enumerate(reversed(self.track.cars)):
            times = self.round_times[car]
            if not times:
                continue
            best_time = min(times)
            s = f"{i + 1} : {self.seconds_to_string(best_time)}"
            arcade.draw_text(s, 0, 12 * (i + 1), color[i], 12)

    @staticmethod
    def seconds_to_string(seconds):
        minutes = int(seconds) // 60
        sec = int(seconds) % 60
        milli = int((seconds - sec) * 1000)
        return f"{minutes:02d}:{sec:02d}:{milli:03d}"

    def update(self, delta_time):
        self.track.step(delta_time)
        self.global_time += delta_time

        for i, car_sprite in enumerate(self.car_sprites):
            car = self.track.cars[i]
            car_sprite.center_x, car_sprite.center_y = self.transform(car.x, car.y)
            car_sprite.angle = car.yaw
            self.round_time_step(car, delta_time)

    def round_time_step(self, car: Car, delta_time):
        self.round_timer[car] += delta_time

        if car.rail_progress == 0 and car.rail == self.track.rails[0]:
            # Lap completed
            new_time = self.round_timer[car]
            self.round_times[car].append(new_time)
            self.round_timer[car] = 0  # Reset clock

    def on_key_press(self, symbol: int, modifiers: int):
        """
        Numbers from 1-9 corresponds to lowest speed setting, to max speed setting. Every other key is zero speed.
        """
        if 49 <= symbol <= 57:
            speed = symbol - 47
        else:
            speed = 0

        for car in self.track.cars:
            if car.key_control:
                car.speed = speed / 9 * Car.MAX_SPEED


def start_game(track):
    game = SlotCarGame(SCREEN_WIDTH, SCREEN_HEIGHT, track)
    game.setup()
    arcade.run()
