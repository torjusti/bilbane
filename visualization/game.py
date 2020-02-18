import arcade
import numpy as np

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

    def setup(self):
        for car in self.track.cars:
            car_sprite = arcade.Sprite('visualization/images/car.png', SPRITE_SCALING_CAR)
            car_sprite.center_x, car_sprite.center_y = self.transform(0, 0)
            self.car_sprites.append(car_sprite)

    def transform(self, x, y):
        coordinate = np.array([x, y])
        difference = (self.track_bounds[1] - self.track_bounds[0])
        max_diff = difference.max()
        normalized = (coordinate - self.track_bounds[0]) / max_diff
        return (normalized + 0.1) * min(SCREEN_WIDTH, SCREEN_HEIGHT)

    def on_draw(self):
        arcade.start_render()

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
