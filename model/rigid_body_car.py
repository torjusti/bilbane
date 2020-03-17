import numpy as np
from point_mass_car import PointMassCar

from model import model

TOL = 1e-3

class RigidBodyCar(PointMassCar):
    # Car will crash if skid angle is larger than this value
    MAX_SKID_ANGLE = np.pi / 6

    # Car properties
    length = None
    width = None
    height = None

    rho_front_axel = None
    rho_rear_axel = None
    rho_pin = None
    rho_mag = None

    mag_width = None

    mu_kin = None

    inertia = None

    def __init__(self, lane, track, key_control=False):
        super().__init__(lane, track, key_control)
        self.is_point_mass = False

        self.length = 0.142
        self.width = 0.061
        self.height = 0.025

        self.rho_front_axel = np.asarray([0.054, 0, 0])
        self.rho_rear_axel = np.asarray([-0.037, 0, 0])
        self.rho_pin = np.asarray([0.068, 0, 0])
        self.rho_mag = np.asarray([-0.032, 0, 0])

        self.mag_width = 0.025

        self.mu_kin = 0.5

        ixx = self.mass * (self.width ** 2 + self.height ** 2) / 12
        iyy = self.mass * (self.length ** 2 + self.height ** 2) / 12
        izz = self.mass * (self.length ** 2 + self.width ** 2) / 12
        self.inertia = np.asarray([[ixx, 0, 0], [0, iyy, 0], [0, 0, izz]])


    ####################################################################################################################################################
    # Calculate forces

    def get_pin_friction(self, pos_cg, vel_cg):
        """
        Purpose: Calculate friction force acting on pin from rail
        Formula: F_pin = mu_pin * L,    L = lateral force from rail on pin
        Returns:
            f2_vec -- ndarray containing the components of the pin friction force acting on the car (in x-, y- and z-direction)
        """

        f2_vec = np.zeros(3)

        l_vec = self.get_lateral_pin_force(pos_cg, vel_cg)
        L = np.linalg.norm(l_vec)
        f2_vec_ = np.asarray([-self.mu_pin * L, 0, 0])

        if np.linalg.norm(vel_cg) < TOL:
            return np.zeros_like(f2_vec_.shape)

        f2_vec = self.rotate(f2_vec_, self.get_gamma_angle(pos_cg))

        return f2_vec

    def get_lateral_friction(self, pos_cg, vel_cg):
        """
        Purpose: Calculate friction force acting on tires from track
        Formula: F_tire = min(mu_tire * N, m * v^2 / r),  N = normal force
        Returns:
            f3_vec -- ndarray containing the components of the tire friction force acting on the car (in x-, y- and z-direction)
        """

        f3_vec = np.zeros(3)

        if isinstance(self.rail, model.TurnRail):

            rho_w = np.linalg.norm((self.rho_front_axel + self.rho_rear_axel) / 2)

            f_hjul_magnitude = - np.linalg.norm(self.get_pin_torque(pos_cg, vel_cg)) / rho_w
            f_hjul = np.asarray([0, -f_hjul_magnitude, 0])
            n_vec = self.get_normal_force(pos_cg, vel_cg)
            N = np.linalg.norm(n_vec)
            f_sladd_magnitude = self.mu_tire * N

            f_kin = - self.mu_kin * N * np.asarray([1,0,0]) # TODO: Fix proper unit vector

            if f_hjul_magnitude <= f_sladd_magnitude:
                f3_vec = -f_hjul
            else:
                f3_vec = -f_hjul - f_kin

        if np.linalg.norm(vel_cg) < TOL:
            return np.zeros_like(f3_vec.shape)

        return f3_vec

    def get_magnet_force(self, pos_cg, vel_cg):
        """
        Purpose: Calculate force from lane acting on the car's magnet
        Returns:
            m_vec -- ndarray containing the components of the magnetic force acting on the car (in x-, y- and z-direction)
        """

        # TODO: Make valid for skidding car (skid => magnet not above rail)

        m_vec = np.asarray([0, 0, -self.mag_coeff])

        return m_vec

    def get_drag_force(self, pos_cg, vel_cg):
        """
        Purpose: Calculate drag force acting on tires from track
        Formula: D = .5 * rho * A * C_d * v^2
        Returns:
            d_vec -- ndarray containing the components of the drag force acting on the car (in x-, y- and z-direction)
        """

        # TODO: Make valid for skidding car (skid => area and drag coefficient change)

        d_vec = np.zeros(3)

        RHO = 1.2  # density of air
        D = .5 * RHO * self.area * self.drag_coeff * np.dot(vel_cg, vel_cg)
        d_vec_ = np.asarray([-D, 0, 0])

        if np.linalg.norm(vel_cg) < TOL:
            return np.zeros_like(d_vec_.shape)

        d_vec = self.rotate(d_vec_, self.get_gamma_angle(pos_cg))

        return d_vec

    def get_lateral_pin_force(self, pos_cg, vel_cg):
        """
        Purpose: Calculate lateral force from the track acting on the car's pin
        Formula: sum(F_centrifugal) = lateral friction + lateral pin force,
                 sum(F_centrifugal) = ma = m * v^2 / r
                 => lateral pin force = m * v^2 / r - lateral friction
        Returns:
            l_vec -- ndarray containing the components of the tire friction force acting on the car (in x-, y- and z-direction)
        """

        l_vec = np.zeros(3)
        """
        rail_center_to_cg_vec = self.get_rail_center_to_cg()
        radial_unit_vector = rail_center_to_cg_vec / np.linalg.norm(rail_center_to_cg_vec)
        radial_speed = np.dot(self.vel_vec, radial_unit_vector)

        l_vec_magnitude = 1 / np.cos(self.ksi) * (self.mass * (radial_speed ** 2) / self.rail.get_lane_radius(self.lane) - np.dot(self.get_lateral_friction(), radial_unit_vector))

        l_vec_ = np.asarray([0, -l_vec_magnitude, 0])

        l_vec = self.rotate(l_vec_, self.get_gamma_angle())
        """

        centripetal_magnitude = np.linalg.norm(self.get_centrifugal_force(pos_cg, vel_cg))

        other_forces = None
        other_forces_global = self.rotate(other_forces, -self.theta)

        # TODO: Other forces should include pin friction, but

        r_ccg = self.get_rail_center_to_cg()
        e_ccg = r_ccg / np.linalg.norm(r_ccg)

        l_vec_magnitude = (1/np.cos(self.get_beta_angle())) * (centripetal_magnitude - np.dot(other_forces_global, e_ccg))

        vec = self.get_rail_center_to_pin() / np.linalg.norm(self.get_rail_center_to_pin())
        l_vec_direction = -self.rotate(vec, self.theta)

        return l_vec_magnitude * l_vec_direction

    def get_centrifugal_force(self):
        """
        Purpose: Calculate centrifugal force experienced by the car
        Formula: F = ma = mv^2/r
        Returns:
            cent_vec -- ndarray containing the components of the centrifugal force experienced by the car (in x-, y- and z-direction)
        """

        cent_vec = np.zeros(3)

        # TODO: Invalid radius, car can follow circular paths of different radii

        if isinstance(self.rail, model.TurnRail):  # Non-zero only if car is on a TurnRail
            cent_magnitude = np.dot(self.vel_vec, self.vel_vec) * self.mass / self.rail.get_lane_radius(self.lane)
            cent_vec = -self.rail.direction * np.asarray([0, cent_magnitude, 0])

        return cent_vec

    ####################################################################################################################
    # Calculate momenta

    def get_pin_torque(self):
        return np.cross(self.rho_pin, (self.get_lateral_pin_force() + self.get_pin_friction()))

    def get_wheel_torque(self):
        rho_wheel = (self.rho_front_axel + self.rho_rear_axel) / 2
        return np.cross(rho_wheel, self.get_lateral_friction())

    def get_total_torque(self):
        return self.get_pin_torque() + self.get_wheel_torque()

    def get_angular_acceleration(self):
        return np.dot(np.linalg.inv(self.inertia), self.get_total_torque())

    def get_angular_velocity(self):
        pass

    ####################################################################################################################
    # Helper functions

    def get_car_track_angle(self):
        # TODO: Implement for car that is not a point mass
        return 0

    def rotate(self, vector, angle):
        rot_matrix = np.asarray([[np.cos(angle), np.sin(angle), 0], [-np.sin(angle), np.cos(angle)], 0], [0, 0, 1])
        rotated_vector = np.dot(rot_matrix, vector)
        return rotated_vector

    def get_rail_center_to_cg(self):
        rail_center = self.rail.get_rail_center()
        return self.pos_vec - rail_center

    def get_rail_center_to_pin(self):
        return self.get_rail_center_to_cg() + self.rho_pin

    def get_pin_position(self):
        return self.pos_vec + self.rho_pin

    def get_pin_velocity(self):
        pass

    def get_pin_acceleration(self):
        pass

    def get_beta_angle(self):
        r_ccg = self.get_rail_center_to_cg()
        r_cp = self.get_rail_center_to_pin()
        return np.arccos(np.dot(r_ccg, r_cp) / (np.linalg.norm(r_ccg)*np.linalg.norm(r_cp)))

    def get_gamma_angle(self):
        r_cp = self.get_rail_center_to_pin()
        e_y = self.rotate(np.asarray([0, 1, 0]), self.theta)
        gamma_value = np.arccos(np.dot(r_cp, e_y) / np.linalg.norm(r_cp))
        gamma_sign = 0
        if isinstance(self.rail, model.TurnRail):
            if np.linalg.norm(self.get_rail_center_to_cg()) < self.rail.get_lane_radius(self.lane):
                gamma_sign = 1
            elif np.linalg.norm(self.get_rail_center_to_cg()) < self.rail.get_lane_radius(self.lane):
                gamma_sign = -1
        return gamma_value * gamma_sign

