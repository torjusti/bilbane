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

        self.car_sprite = arcade.Sprite('visualization/images/car.png', SPRITE_SCALING_CAR)

        self.car_sprite.center_x = SCREEN_WIDTH / 2
        self.car_sprite.center_y = SCREEN_HEIGHT / 2

    def on_draw(self):
        arcade.start_render()
        self.car_sprite.draw()


    def update(self, delta_time):
        self.track.step(delta_time)

        self.car_sprite.center_x = self.track.cars[0].x + SCREEN_WIDTH / 2
        self.car_sprite.center_y = self.track.cars[0].y + SCREEN_HEIGHT / 2
        self.car_sprite.angle = self.track.cars[0].yaw


def start_game(track):
    game = SlotCarGame(SCREEN_WIDTH, SCREEN_HEIGHT)
    game.setup(track)
    arcade.run()
