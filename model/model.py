import math

import numpy as np

DEFAULT_YAW = -90


class Rail:
    RAIL_WIDTH = .10
    LANE_EDGE_DIST = .3
    LANE_LANE_DIST = .4

    # The coordinates is the middle point of the rail in the front.
    global_x = None
    global_y = None
    global_angle = None

    next_rail = None

    Lane1 = 1
    Lane2 = -1


class StraightRail(Rail):
    def __init__(self, length):
        if length <= 0:
            raise ValueError('Rail length must be positive')

        self.length = length

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
    

    @property
    def length(self):
        return self.radius * self.angle

    def get_length(self, lane):
        return (self.radius - self.direction * lane * Rail.LANE_LANE_DIST / 2) * self.angle


class Track:
    def __init__(self, rails, cars):
        self.rails = rails
        self.cars = cars
        for car in self.cars:
            car.rail = rails[0]

        if not self.initialize_rail_coordinates():
            raise ValueError('Track did not form a loop')

    def _get_turn_circle(self, rail):
        """ Compute the center of the circle describing a turn rail, and the car entry angle. """
        circle_x = rail.global_x + rail.radius * math.cos(rail.global_angle + rail.direction * math.pi / 2)
        circle_y = rail.global_y + rail.radius * math.sin(rail.global_angle + rail.direction * math.pi / 2)
        initial_angle = math.atan2(rail.global_y - circle_y, rail.global_x - circle_x)
        return circle_x, circle_y, initial_angle

    def initialize_rail_coordinates(self):
        x, y, angle = 0, 0, 0
        self.straight_track_coordinates = []
        self.turn_track_coordinates = []

        for i, rail in enumerate(self.rails):
            rail.global_angle = angle
            rail.global_x = x
            rail.global_y = y

            rail.next_rail = self.rails[(i + 1) % len(self.rails)]

            if isinstance(rail, StraightRail):
                x += math.cos(angle) * rail.length
                y += math.sin(angle) * rail.length
                self.straight_track_coordinates.append([rail.global_x, rail.global_y, x, y])
            elif isinstance(rail, TurnRail):
                circle_x, circle_y, initial_angle = self._get_turn_circle(rail)
                x = circle_x + rail.radius * math.cos(initial_angle + rail.direction * rail.angle)
                y = circle_y + rail.radius * math.sin(initial_angle + rail.direction * rail.angle)
                angle += rail.angle

                start_ang = initial_angle 
                end_ang = rail.angle + initial_angle
                
                if(rail.direction == TurnRail.Right):
                     start_ang += 2*math.pi-rail.angle
                     end_ang += 2*math.pi-rail.angle
                
                self.turn_track_coordinates.append([circle_x, circle_y, rail.radius, rail.radius,
                 start_ang*180/math.pi, end_ang*180/math.pi])

        return abs(x) <= 1e-9 and abs(y) <= 1e-9

    def get_track_bounds(self):
        """
        Calculate the coordinates of the bottom left, and the top right corners of the track.
        """
        bottom_left, top_right = np.zeros(2), np.zeros(2)
        for rail in self.rails:
            if isinstance(rail, StraightRail):
                continue
            circle_x, circle_y, initial_angle = self._get_turn_circle(rail)
            coord = np.array([circle_x, circle_y])
            radius = rail.radius + 0.5 * Rail.RAIL_WIDTH

            circle_bottom_left = coord - radius
            bottom_left = np.minimum(bottom_left, circle_bottom_left)

            circle_top_right = coord + radius
            top_right = np.maximum(top_right, circle_top_right)

        return bottom_left, top_right

    def place_car(self, car, dist):
        """
        :param car:
        :param dist: distance moved in meter
        """
        pass

        
    def step(self, delta_time):
        for car in self.cars:
            rail = car.rail

            car.rail_progress += delta_time * car.speed / rail.get_length(car.lane)

            car.rail_progress = min(car.rail_progress, 1)

            if isinstance(rail, StraightRail):
                car.x = rail.global_x + math.cos(rail.global_angle) * car.rail_progress * rail.length
                car.y = rail.global_y + math.sin(rail.global_angle) * car.rail_progress * rail.length

                car.x += math.cos(rail.global_angle + math.pi / 2 * car.lane) * Rail.LANE_LANE_DIST / 2
                car.y += math.sin(rail.global_angle + math.pi / 2 * car.lane) * Rail.LANE_LANE_DIST / 2
            elif isinstance(rail, TurnRail):
                circle_x, circle_y, initial_angle = self._get_turn_circle(rail)

                angle = initial_angle + rail.angle * car.rail_progress * rail.direction

                car.x = circle_x + rail.radius * math.cos(angle)
                car.y = circle_y + rail.radius * math.sin(angle)

                yaw = rail.global_angle + rail.angle * car.rail_progress * rail.direction

                car.x += math.cos(yaw + math.pi / 2 * car.lane) * Rail.LANE_LANE_DIST / 2
                car.y += math.sin(yaw + math.pi / 2 * car.lane) * Rail.LANE_LANE_DIST / 2

                car.yaw = yaw * 180 / math.pi + DEFAULT_YAW

            if car.rail_progress == 1:
                car.rail = car.rail.next_rail
                car.rail_progress = 0


class Car:
    MAX_SPEED = 500

    # Position of the car in the global coordinate system.
    x = 0
    y = 0

    speed = 0
    thrust = 0

    # Index of the track the car is situated on.
    rail = None
    # How far along the car is on the current rail.
    rail_progress = 0

    lane = None

    # Yaw with respect to the global coordiante system.
    yaw = DEFAULT_YAW

    def __init__(self, lane, speed):
        self.lane = lane
        self.speed = speed

    def distance_moved(self, delta_time):
        """
        :param delta_time:
        :return: distance in meter
        """
        self.rail_progress += delta_time * self.speed / self.rail.get_length(self.lane)
        self.rail_progress = min(self.rail_progress, 1)

        if self.rail_progress == 1:
            self.rail = self.rail.next_rail
