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

        for i, rail in enumerate(self.rails):
            rail.global_angle = angle
            rail.global_x = x
            rail.global_y = y

            rail.next_rail = self.rails[(i + 1) % len(self.rails)]

            if isinstance(rail, StraightRail):
                x += math.cos(angle) * rail.length
                y += math.sin(angle) * rail.length
            elif isinstance(rail, TurnRail):
                circle_x, circle_y, initial_angle = self._get_turn_circle(rail)
                x = circle_x + rail.radius * math.cos(initial_angle + rail.direction * rail.angle)
                y = circle_y + rail.radius * math.sin(initial_angle + rail.direction * rail.angle)
                angle += rail.angle

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

            circle_bottom_left = coord - rail.radius
            bottom_left = np.minimum(bottom_left, circle_bottom_left)

            circle_top_right = coord + rail.radius
            top_right = np.maximum(top_right, circle_top_right)

        return bottom_left, top_right

    def place_car(self, car, dist):
        """
        :param car:
        :param dist: distance moved in meter
        """
        pass

    def get_track_coordinates(self,init_x,init_y):
        """" 
        Returns one array for with coordinates for straight rails, x_start,y_start,x_end, y_end
        and one array with coordinates for turn rails, x_center, y_center, width, height, start_ang, end_ang
        
        """
        straight_track_coordinates = []
        turn_track_coordinates = []
        x_end = init_x
        y_end = init_y
        angle = 0
        for rail in self.rails:
            x_start = x_end
            y_start = y_end
            if isinstance(rail, StraightRail):
                x_end += math.cos(angle) * rail.length
                y_end += math.sin(angle) * rail.length
                straight_track_coordinates.append([x_start, y_start, x_end, y_end])
            elif isinstance(rail, TurnRail):
                #center_x, center_y, initial_angle = self._get_turn_circle(rail)
                center_x = x_start + rail.radius * math.cos(angle + rail.direction * math.pi / 2)
                center_y = y_start + rail.radius * math.sin(angle + rail.direction * math.pi / 2)
                initial_angle = math.atan2(y_start - center_y, x_start - center_x)
    
                x_end = center_x + rail.radius * math.cos(initial_angle + rail.direction * rail.angle)
                y_end = center_y + rail.radius * math.sin(initial_angle + rail.direction * rail.angle)
                
                width = rail.radius
                height = rail.radius
                start_ang = (initial_angle) * 180/math.pi
                end_ang = (rail.angle + initial_angle) * 180/math.pi
                
                if(rail.direction == TurnRail.Right):
                     start_ang += 360-rail.angle*180/math.pi
                     end_ang += 360-rail.angle*180/math.pi
                #print("start ", start_ang)
                #print("end ", end_ang)
                angle += rail.angle
                turn_track_coordinates.append([center_x, center_y, width, height, start_ang, end_ang]) 

        return straight_track_coordinates,turn_track_coordinates
        
    def step(self, delta_time):
        for car in self.cars:
            rail = car.rail

            car.rail_progress += delta_time * car.speed / rail.length

            car.rail_progress = min(car.rail_progress, 1)

            if isinstance(rail, StraightRail):
                car.x = rail.global_x + math.cos(rail.global_angle) * car.rail_progress * rail.length
                car.y = rail.global_y + math.sin(rail.global_angle) * car.rail_progress * rail.length
            elif isinstance(rail, TurnRail):
                circle_x, circle_y, initial_angle = self._get_turn_circle(rail)

                angle = initial_angle + rail.angle * car.rail_progress * rail.direction

                car.x = circle_x + rail.radius * math.cos(angle)
                car.y = circle_y + rail.radius * math.sin(angle)

                car.yaw = (rail.global_angle + rail.angle * car.rail_progress * rail.direction) * 180 / math.pi + DEFAULT_YAW

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
