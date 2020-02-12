import arcade

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

SPRITE_SCALING_CAR = 0.25


class SlotCarGame(arcade.Window):
    space_pressed = False

    def __init__(self, width, height):
        super().__init__(width, height)

        arcade.set_background_color(arcade.color.WHITE)

    def setup(self, track):
        self.track = track
        self.car_sprites = arcade.SpriteList()

        for car in track.cars:
            car_sprite = arcade.Sprite('visualization/images/car.png', SPRITE_SCALING_CAR)
            car_sprite.center_x = SCREEN_WIDTH / 2
            car_sprite.center_y = SCREEN_HEIGHT / 2
            self.car_sprites.append(car_sprite)

    def on_draw(self):
        arcade.start_render()

        for car in self.car_sprites:
            car.draw()

    def update(self, delta_time):
        self.track.step(delta_time)

        for i, car_sprite in enumerate(self.car_sprites):
            car_sprite.center_x = self.track.cars[i].x + SCREEN_WIDTH / 2
            car_sprite.center_y = self.track.cars[i].y + SCREEN_HEIGHT / 2
            car_sprite.angle = self.track.cars[i].yaw

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
    game = SlotCarGame(SCREEN_WIDTH, SCREEN_HEIGHT)
    game.setup(track)
    arcade.run()
