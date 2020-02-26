import numpy as np
from point_mass_car import PointMassCar

from model import model


class RigidBodyCar(PointMassCar):
    # Car will crash if skid iangle s larger than this value
    MAX_SKID_ANGLE = np.pi / 6

    # Car properties
    length         = None
    width          = None
    height         = None
    rho_front_axel = None
    rho_rear_axel  = None
    rho_pin        = None
    rho_mag        = None
    mag_width      = None
    inertia        = None

    def __init__(self, lane, track, key_control=False):
        super().__init__(lane, track, key_control)
        self.is_point_mass = False

        self.length = 0.142
        self.width  = 0.061
        self.height = 0.025

        self.rho_pin = np.asarray([0.068, 0, 0])
        self.rho_mag = np.asarray([-0.032, 0, 0])

        self.rho_front_axel = np.asarray([0.054, 0, 0])
        self.rho_rear_axel  = np.asarray([-0.037, 0, 0])

        self.mag_width = 0.025

        ixx = self.mass * (self.width  ** 2 + self.height ** 2) / 12
        iyy = self.mass * (self.length ** 2 + self.height ** 2) / 12
        izz = self.mass * (self.length ** 2 + self.width  ** 2) / 12
        self.inertia = np.asarray([[ixx, 0, 0], [0, iyy, 0], [0, 0, izz]])

    def get_new_pos(self, delta_time):
        """
        Purpose: Get new position of the car
        Args:
            delta_time -- time step in seconds
        Returns:
            new_pos_vec -- ndarray containing new car position (x,y,z)
        """

        new_acc_vec = self.get_total_force() / self.mass
        new_vel_vec = self.vel_vec + new_acc_vec * delta_time
        new_pos_vec = new_vel_vec * delta_time + .5 * new_acc_vec * delta_time

        return new_pos_vec

    def get_new_vel(self, delta_time):
        """
        Purpose: Get new velocity of the car
        Args:
            delta_time -- time step in seconds
        Returns:
            new_vel_vec -- ndarray containing new car velocity (u, v, w)
        """

        new_acc_vec = self.get_total_force() / self.mass
        old_vel_vec = np.asarray([np.linalg.norm(self.vel_vec), 0, 0])
        new_vel_vec = old_vel_vec + new_acc_vec * delta_time

        new_vel_vec[0] = max(0, new_vel_vec[0])

        return new_vel_vec

    def get_new_angles(self, new_pos_vec, new_rail):
        """
        Purpose: Get new rotation of the car relative to global coordinate system
        Assumptions:
            - Car is a point mass
            - 2D
        Args:
            new_pos_vec -- ndarray containing new car position (x,y,z)
            new_rail -- instance of Rail, the rail on which the car is located when it has moved to new_pos_vec
        Returns:
            new_angle_vec -- ndarray containing new rotation of car relative to global coordinate system (roll, pitch, yaw)
        """

        # If on a turn
        if isinstance(new_rail, model.TurnRail):
            pos_vec_COR = new_rail.get_rail_center()
            lane_radius = new_rail.get_lane_radius(self.lane)
            left_turn = (new_rail.direction == model.TurnRail.Left)

            pos_vec_rel = new_pos_vec - pos_vec_COR  # Vector from centre of rail to car
            pos_vec_up = np.asarray(
                [0, self.rail.direction * lane_radius, 0])  # Vector from centre of rail in positive y(local) coord
            dot_product = np.dot(pos_vec_rel, pos_vec_up)  # Dotproduct between the two
            relative_angle = np.arccos(
                np.clip(dot_product / (lane_radius * lane_radius), -1.0,
                        1.0))  # Angle between Y(global) and y(local) axis

            if pos_vec_rel[0] < 0:  # If relative position vector points to the right
                yaw = relative_angle  # Angle between X(global) and x(local) axis
            else:
                yaw = 2 * np.pi - relative_angle  # Angle between X(global) and x(local) axis

            if left_turn:
                yaw = yaw + np.pi  # Left turn offsets the calculations above with 180 degrees because of flipped x(local) axis

            if (yaw >= 2 * np.pi):
                yaw = yaw - 2 * np.pi
            if (yaw < 0):
                yaw = yaw + 2 * np.pi

        # Else we are on a straight
        else:
            yaw = new_rail.global_angle

        new_angle_vec = np.asarray([0, 0, yaw])
        return new_angle_vec

    ####################################################################################################################################################
    # Calculate forces

    def get_pin_friction(self):
        """
        Purpose: Calculate friction force acting on pin from rail
        Formula: F_pin = mu_pin * L,    L = lateral force from rail on pin
        Returns:
            f2_vec -- ndarray containing the components of the pin friction force acting on the car (in x-, y- and z-direction)
        """

        l_vec = self.get_lateral_pin_force()
        L = np.linalg.norm(l_vec)
        f2_vec_ = np.asarray([-self.mu_pin * L, 0, 0])

        if np.linalg.norm(self.vel_vec) < 1e-3:
            return np.zeros_like(f2_vec_.shape)

        return f2_vec

    def get_lateral_friction(self):
        """
        Purpose: Calculate friction force acting on tires from track
        Formula: F_tire = min(mu_tire * N, m * v^2 / r),  N = normal force
        Returns:
            f3_vec -- ndarray containing the components of the tire friction force acting on the car (in x-, y- and z-direction)
        """

        f3_vec = np.zeros(3)

        if isinstance(self.rail, model.TurnRail):
            n_vec = self.get_normal_force()
            N = np.linalg.norm(n_vec)
            centripetal_force = np.linalg.norm(self.get_centrifugal_force())
            f3_vec[1] = self.rail.direction * np.minimum(self.mu_tire * N,
                                                         centripetal_force)  # Friction cannot exceed centripetal force

        if np.linalg.norm(self.vel_vec) < 1e-3:
            return np.zeros_like(f3_vec.shape)

        return f3_vec

    def get_magnet_force(self):
        """
        Purpose: Calculate force from lane acting on the car's magnet
        Returns:
            m_vec -- ndarray containing the components of the magnetic force acting on the car (in x-, y- and z-direction)
        """

        # TODO: Make valid for skidding car (skid => magnet not above rail)

        m_vec = np.asarray([0, 0, -self.mag_coeff])

        return m_vec

    def get_thrust_force(self):
        """
        Purpose: Calculate thrust force acting on the car's tires from track, due to forced rotation of the tires (by the car's motor)
        Formula: T = eta * U^2 / R,     eta = efficieny
                                        U   = track voltage
                                        R   = total resitance of the car's lane
        Returns:
            t_vec -- ndarray containing the components of the thrust force acting on the car (in x-, y- and z-direction)
        """

        U = self.controller_input * self.MAX_VOLTAGE

        if (self.lane == model.Rail.Lane1):
            R = self.track.resistances[0]
        elif (self.lane == model.Rail.Lane2):
            R = self.track.resistances[1]
        else:
            raise ValueError("Invalid lane value")

        T = self.motor_eta * (U ** 2) / R
        t_vec = np.asarray([T, 0, 0])

        return t_vec

    def get_drag_force(self):
        """
        Purpose: Calculate drag force acting on tires from track
        Formula: D = .5 * rho * A * C_d * v^2
        Returns:
            d_vec -- ndarray containing the components of the drag force acting on the car (in x-, y- and z-direction)
        """

        # TODO: Make valid for skidding car (skid => area and drag coefficient change)

        RHO = 1.2  # density of air
        D = .5 * RHO * self.area * self.drag_coeff * np.dot(self.vel_vec, self.vel_vec)
        d_vec = np.asarray([-D, 0, 0])

        if np.linalg.norm(self.vel_vec) < 1e-3:
            return np.zeros_like(d_vec.shape)

        return d_vec

    def get_lateral_pin_force(self):
        """
        Purpose: Calculate lateral force from the track acting on the car's pin
        Formula: sum(F_centrifugal) = lateral friction + lateral pin force,
                 sum(F_centrifugal) = ma = m * v^2 / r
                 => lateral pin force = m * v^2 / r - lateral friction
        Returns:
            l_vec -- ndarray containing the components of the tire friction force acting on the car (in x-, y- and z-direction)
        """

        l_vec = np.zeros(3)

        if isinstance(self.rail, model.TurnRail):  # Non-zero only if car is on a TurnRail
            cent_vec = self.get_centrifugal_force()
            l_vec = - cent_vec - self.get_lateral_friction()

        return l_vec

    def get_centrifugal_force(self):
        """
        Purpose: Calculate centrifugal force experienced by the car
        Formula: F = ma = mv^2/r
        Returns:
            cent_vec -- ndarray containing the components of the centrifugal force experienced by the car (in x-, y- and z-direction)
        """

        cent_vec = np.zeros(3)

        if isinstance(self.rail, model.TurnRail):  # Non-zero only if car is on a TurnRail
            cent_magnitude = np.dot(self.vel_vec, self.vel_vec) * self.mass / self.rail.get_lane_radius(self.lane)
            cent_vec = -self.rail.direction * np.asarray([0, cent_magnitude, 0])

        return cent_vec

    ###################################################################################################################################
    # Calculate momenta

    ###################################################################################################################################
    # Helper functions

    def get_car_track_angle(self):
        # TODO: Implement for car that is not a point mass
        return 0
