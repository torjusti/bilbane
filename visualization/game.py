import arcade
import math
from visualization.utils import create_arc_outline as cao
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

SPRITE_SCALING_CAR = 0.25
INIT_CENTER_X = SCREEN_WIDTH/4
INIT_CENTER_Y = SCREEN_HEIGHT/4

class SlotCarGame(arcade.Window):
    space_pressed = False

    def __init__(self, width, height):
        super().__init__(width, height)

        arcade.set_background_color(arcade.color.WHITE)

    def setup_track(self):
        track_coordinates = self.track.get_track_coordinates(INIT_CENTER_X,INIT_CENTER_Y)
        self.track_element_list = arcade.ShapeElementList()
        for i in range(len(track_coordinates)-1):
            if track_coordinates[i+1][3] == 0:
                shape = arcade.create_line(track_coordinates[i][0],track_coordinates[i][1],
                track_coordinates[i+1][0],track_coordinates[i+1][1],arcade.color.BLACK)
                self.track_element_list.append(shape)
            else:
                start_ang = min(track_coordinates[i][2],track_coordinates[i+1][2])*180/math.pi-90
                end_ang = max(track_coordinates[i][2],track_coordinates[i+1][2])*180/math.pi-90
                width = abs(track_coordinates[i][0]-track_coordinates[i+1][0])
                height = abs(track_coordinates[i][1]-track_coordinates[i+1][1])        
                if track_coordinates[i+1][0]-track_coordinates[i][0]<0 and\
                    track_coordinates[i+1][1]-track_coordinates[i][1]>0:
                    center_x = track_coordinates[i][0] +\
                        (track_coordinates[i+1][0]-track_coordinates[i][0])
                    center_y = track_coordinates[i][1]
                elif track_coordinates[i+1][0]-track_coordinates[i][0]>0 and\
                    track_coordinates[i+1][1]-track_coordinates[i][1]<0:
                    center_x = track_coordinates[i][0] +\
                        (track_coordinates[i+1][0]-track_coordinates[i][0])
                    center_y = track_coordinates[i][1]
                else:
                    center_x = track_coordinates[i][0]
                    center_y = track_coordinates[i][1]+\
                        (track_coordinates[i+1][1]-track_coordinates[i][1])
                shape = cao.create_arc_outline(center_x,center_y,width,height,arcade.color.BLACK,start_ang,
                end_ang)
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
        self.track.get_track_coordinates(INIT_CENTER_X,INIT_CENTER_Y)

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
