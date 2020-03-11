import math
import numpy as np

DEFAULT_YAW = -90


class Rail:
    LANE_EDGE_DIST = .039
    LANE_LANE_DIST = .0765
    RAIL_WIDTH = LANE_LANE_DIST + 2 * LANE_EDGE_DIST

    RESISTANCE_PER_UNIT_LENGTH = .033

    # The coordinates is the middle point of the rail in the front.
    global_x = None
    global_y = None
    global_angle = None # Angle with x-axis, positive CCW

    next_rail = None

    Lane1 = 1
    Lane2 = -1

    resistances = None


class StraightRail(Rail):
    def __init__(self, length):
        if length <= 0:
            raise ValueError('Rail length must be positive')

        self.length = length
        self.resistances = np.asarray([self.length, self.length]) * self.RESISTANCE_PER_UNIT_LENGTH

    def get_length(self, lane):
        return self.length


class TurnRail(Rail):
    Left = 1
    Right = -1

    def __init__(self, radius, angle, direction):
        if radius <= 0:
            raise ValueError('Radius must be positive')

        self.radius = radius  # counted from the middle of the rail
        self.angle = angle  # Number of radians rotated relative to global coordinate system by travelling along the entire rail
        self.direction = direction  # 1 for left turn, -1 for right turn
        self.resistances = np.asarray([self.get_length(Rail.Lane1), self.get_length(Rail.Lane2)]) * self.RESISTANCE_PER_UNIT_LENGTH

    @property
    def length(self):
        return self.radius * self.angle

    def get_length(self, lane):
        return self.get_lane_radius(lane) * self.angle

    def get_rail_center(self):
        # TODO: Asummes 2D
        pos_vec = np.asarray([self.global_x, self.global_y, 0])

        orientation = np.asarray([np.cos(self.global_angle), np.sin(self.global_angle), 0])

        # Left turn => center is 90 degrees CCW relative to orientation
        # Right turn => center is 90 degrees CW relative to orientation
        rot_matrix = np.asarray([[0, -self.direction, 0], [self.direction, 0, 0], [0, 0, 1]])

        vec_start_to_center = np.dot(rot_matrix, orientation) * self.radius
        global_center_coords = pos_vec + vec_start_to_center
        return global_center_coords

    def get_lane_radius(self, lane):
        return self.radius - self.direction * lane * Rail.LANE_LANE_DIST / 2


class Track:
    def __init__(self, rails, cars):
        self.rails = rails
        self.cars = cars

        if not self.initialize_rail_coordinates():
            raise ValueError('Track did not form a loop')

        self.resistances = np.zeros(2)
        for rail in self.rails:
            self.resistances[0] += rail.resistances[0]
            self.resistances[1] += rail.resistances[1]

    def _get_turn_circle(self, rail):
        """ Compute the center of the circle describing a turn rail, and the car entry angle. """
        # TODO: Compute using Numpy method above.
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
                angle += rail.angle * rail.direction

                start_ang = initial_angle
                end_ang = rail.angle + initial_angle

                if rail.direction == TurnRail.Right:
                     start_ang += 2 * math.pi-rail.angle
                     end_ang += 2 * math.pi-rail.angle

                self.turn_track_coordinates.append([circle_x, circle_y, rail.radius, rail.radius,
                    start_ang * 180 / math.pi, end_ang * 180 / math.pi])

        return abs(x) <= 1e-9 and abs(y) <= 1e-9

    def get_track_bounds(self):
        """
        Calculate the coordinates of the bottom left, and the top right corners of the track.
        """
        bottom_left, top_right = np.zeros(2), np.zeros(2)
        for rail in self.rails:
            if isinstance(rail, StraightRail):
                continue
            circle_x, circle_y, _ = self._get_turn_circle(rail)
            coord = np.array([circle_x, circle_y])
            radius = rail.radius + 0.5 * Rail.RAIL_WIDTH

            circle_bottom_left = coord - radius
            bottom_left = np.minimum(bottom_left, circle_bottom_left)

            circle_top_right = coord + radius
            top_right = np.maximum(top_right, circle_top_right)

        return bottom_left, top_right

    def step(self, delta_time):
        for car in self.cars:
            car.get_new_state_track_locked(delta_time)
