import math

SPEED = 500

DEFAULT_YAW = -90

class StraightRail:
    def __init__(self, length):
        if length <= 0:
            raise ValueError('Rail length must be positive')

        self.length = length


class TurnRail:
    Left = 1
    Right = -1

    def __init__(self, radius, angle, direction):
        if radius <= 0:
            raise ValueError('Radius must be positive')

        self.radius = radius
        self.angle = angle
        self.direction = direction

    @property
    def length(self):
        return self.radius * self.angle


class Track:
    def __init__(self, rails, cars):
        self.rails = rails
        self.cars = cars

        if not self.initialize_rail_coordinates():
            raise ValueError('Track did not form a loop')

    def initialize_rail_coordinates(self):
        x, y, angle = 0, 0, 0

        for rail in self.rails:
            rail.global_angle = angle
            rail.global_x = x
            rail.global_y = y

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

    def step(self, delta_time):
        for car in self.cars:
            rail = self.rails[car.rail]

            car.rail_progress += delta_time * SPEED / rail.length

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
                car.rail = (car.rail + 1) % len(self.rails)
                car.rail_progress = 0


class Car:
    # Position of the car in the global coordinate system.
    x = 0
    y = 0

    # Index of the track the car is situated on.
    rail = 0
    # How far along the car is on the current rail.
    rail_progress = 0

    # Yaw with respect to the global coordiante system.
    yaw = DEFAULT_YAW
