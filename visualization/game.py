import arcade
import math
from visualization.utils import create_arc_outline as cao
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

SPRITE_SCALING_CAR = 0.25
INIT_CENTER_X = SCREEN_WIDTH/2
INIT_CENTER_Y = SCREEN_HEIGHT/2

class SlotCarGame(arcade.Window):
    space_pressed = False

    def __init__(self, width, height):
        super().__init__(width, height)

        arcade.set_background_color(arcade.color.WHITE)

    def setup_track(self):
        straight_track_coordinates,turn_track_coordinated =\
        self.track.get_track_coordinates(INIT_CENTER_X,INIT_CENTER_Y)

        self.track_element_list = arcade.ShapeElementList()

        for coord in straight_track_coordinates:
            shape = arcade.create_line(*coord,arcade.color.BLACK) 
            self.track_element_list.append(shape)

        for coord in turn_track_coordinated:
            shape = cao.create_arc_outline(*coord[0:4],arcade.color.BLACK,*coord[4:6])
            self.track_element_list.append(shape)
        
    
    def setup(self, track):
        self.track = track
        self.car_sprites = arcade.SpriteList()
        self.setup_track()
        for car in track.cars:
            car_sprite = arcade.Sprite('visualization/images/car.png', SPRITE_SCALING_CAR)
            car_sprite.center_x = INIT_CENTER_X
            car_sprite.center_y = INIT_CENTER_Y
            self.car_sprites.append(car_sprite)

    def on_draw(self):
        arcade.start_render()
        self.track_element_list.draw()
        for car in self.car_sprites:
            car.draw()

        self.car_sprite = arcade.Sprite('visualization/images/car.png', SPRITE_SCALING_CAR)

        self.car_sprite.center_x = INIT_CENTER_X
        self.car_sprite.center_y = INIT_CENTER_Y         

    def update(self, delta_time):
        self.track.step(delta_time)

        for i, car_sprite in enumerate(self.car_sprites):
            car_sprite.center_x = self.track.cars[i].x + INIT_CENTER_X
            car_sprite.center_y = self.track.cars[i].y + INIT_CENTER_Y
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
        self.car_sprite.center_x = self.track.cars[0].x + INIT_CENTER_X
        self.car_sprite.center_y = self.track.cars[0].y + INIT_CENTER_Y
        self.car_sprite.angle = self.track.cars[0].yaw


def start_game(track):
    game = SlotCarGame(SCREEN_WIDTH, SCREEN_HEIGHT)
    game.setup(track)
    arcade.run()
