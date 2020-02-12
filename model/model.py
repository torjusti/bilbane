import math
import numpy as np
import car

DEFAULT_YAW = -90
G_ACC = 9.81


class Rail:
    RAIL_WIDTH = .10
    LANE_EDGE_DIST = .3
    LANE_LANE_DIST = .4
    RESISTANCE_PER_UNIT_LENGTH = 1

    # The coordinates is the middle point of the rail in the front.
    global_x = None
    global_y = None
    global_angle = None

    next_rail = None

    Lane1 = 1
    Lane2 = -1

    resistances = None


class StraightRail(Rail):
    def __init__(self, length):
        if length <= 0:
            raise ValueError('Rail length must be positive')

        self.length = length
        self.resistances = np.asarray([self.length, self.length]) * RESISTANCE_PER_UNIT_LENGTH

    def get_length(self, lane):
        return self.length


class TurnRail(Rail):
    Left = 1
    Right = -1

    def __init__(self, radius, angle, direction):
        if radius <= 0:
            raise ValueError('Radius must be positive')

        self.radius = radius  # counted from the middle of the rail
        self.angle = angle
        self.direction = direction
        self.resistances = np.asarray([get_lane_length(self, Rail.Lane1), get_lane_length(self, Rail.Lane2)]) * RESISTANCE_PER_UNIT_LENGTH

    @property
    def length(self):
        return self.radius * self.angle

    def get_lane_length(self, lane):
        return (self.radius - self.direction * lane * Rail.LANE_LANE_DIST / 2) * self.angle


class Track:
    def __init__(self, rails, cars):
        self.rails = rails
        self.cars = cars
        for car in self.cars:
            car.rail = rails[0]

        if not self.initialize_rail_coordinates():
            raise ValueError('Track did not form a loop')

        self.resistances = np.asarray([0,0])
        for rail in self.rails:
            self.resistances[0] += rail.resistances[0]
            self.resistances[1] += rail.resistances[1]

    def initialize_rail_coordinates(self):
        x, y, angle = 0, 0, 0

        for i, rail in enumerate(self.rails):
            rail.global_angle = angle
            rail.global_x = x
            rail.global_y = y
            rail.next_rail = self.rails[(i + 1) % len(self.rails)]

            if isinstance(rail, StraightRail):
                x += math.cos(angle) * rail.length
                y += math.sin(angle) * rail.length
            elif isinstance(rail, TurnRail):
                delta_yaw = rail.direction * (angle + rail.angle + math.pi * 3 / 2)

                x += rail.radius * math.cos(delta_yaw) + math.cos(angle \
                                                                  + math.pi / 2) * rail.radius * rail.direction

                y += rail.radius * math.sin(delta_yaw) + math.sin(angle \
                                                                  + math.pi / 2) * rail.radius * rail.direction

                angle += rail.angle

        return x <= 1e-9 and y <= 1e-9

    def place_car(self, car, dist):
        """
        :param car:
        :param dist: distance moved in meter
        """
        pass

    def step(self, delta_time):
        for car in self.cars:
            rail = car.rail

            car.rail_progress += delta_time * car.speed / rail.length

            car.rail_progress = min(car.rail_progress, 1)

            if isinstance(rail, StraightRail):
                car.x = rail.global_x + math.cos(rail.global_angle) * car.rail_progress * rail.length
                car.y = rail.global_y + math.sin(rail.global_angle) * car.rail_progress * rail.length
            elif isinstance(rail, TurnRail):
                delta_yaw = rail.direction * (rail.global_angle + rail.angle \
                                              * car.rail_progress + math.pi * 3 / 2)

                car.x = rail.global_x + rail.radius * math.cos(delta_yaw) + math.cos(
                    rail.global_angle + math.pi / 2) * rail.radius * rail.direction

                car.y = rail.global_y + rail.radius * math.sin(delta_yaw) + math.sin(
                    rail.global_angle + math.pi / 2) * rail.radius * rail.direction

                car.yaw = (rail.global_angle + rail.angle * car.rail_progress \
                           * rail.direction) * 180 / math.pi + DEFAULT_YAW

            if car.rail_progress == 1:
                car.rail = car.rail.next_rail
                car.rail_progress = 0


