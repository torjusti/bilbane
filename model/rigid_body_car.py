import numpy as np
from model.point_mass_car import PointMassCar

from model import model

TOL = 1e-3

class RigidBodyCar(PointMassCar):
    # Car will crash if skid angle is larger than this value
    MAX_SKID_ANGLE = np.Inf

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

    omega = None

    def __init__(self, lane, track, key_control=False, track_locked=False):
        super().__init__(lane, track, key_control, track_locked)
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

        self.omega = 0.0
        self.pos_vec = -self.rho_pin + np.asarray([0, self.lane * model.Rail.LANE_LANE_DIST / 2, 0])

    def crash_check(self):
        return np.abs(self.get_gamma_angle(self.pos_vec, self.phi)) > self.MAX_SKID_ANGLE

    def reset(self):
        """
        Purpose: Reset all car variables after a crash has completed.
        Args:
        Returns:
            void
        """

        self.is_crashed = False
        self.crash_time = 0

        self.pos_vec = -self.rho_pin + np.asarray([0, self.lane * model.Rail.LANE_LANE_DIST / 2, 0])
        self.vel_vec = np.zeros(3)
        self.phi = 0.0
        self.omega = 0.0

        self.rail = self.track.rails[0]
        self.rail_progress = 0
        self.laps_completed = 0

        self.controller_input = 0

    def physics_update(self, delta_time):
        new_pos_vec, new_vel_vec, new_phi, new_omega = self.get_physics_state(delta_time)

        # Update rail progress based on new velocity from physics calculations
        self.rail_progress = self.get_rail_progress(new_pos_vec, new_vel_vec, delta_time)

        # Overwrite new_pos_vec and new_phi if locked to track
        if self.track_locked:
            pass # TODO

        # Move to next rail if reached end of current rail
        if self.rail_progress == 1:
            self.rail = self.rail.next_rail
            self.rail_progress = 0

        # Update state
        self.pos_vec = new_pos_vec
        self.vel_vec = new_vel_vec
        self.phi = new_phi
        self.omega = new_omega
        print(self.pos_vec)

    def get_physics_state(self, delta_time):
        """
        Purpose: Get new state of the car, according to the physics model.
        Args:
            delta_time (float) -- size of time step (in seconds).
        Returns:
            new_pos_vec (ndarray, shape=[3,]) -- new car position, (X,Y,Z) (in meters).
            new_vel_vec (ndarray, shape=[3,]) -- new car velocity (in meters per second).
            new_phi (float) -- new yaw of car (in radians).
        """

        # Save old state
        old_pos_vec = self.pos_vec
        old_vel_vec = self.vel_vec
        old_phi = self.phi
        old_omega = self.omega
        old_c_in = self.controller_input

        # Calculate new state
        new_pos_vec, new_vel_vec = self.get_new_pos_and_vel(old_pos_vec, old_vel_vec, old_phi, old_c_in, delta_time)
        #new_phi = self.get_phi(new_pos_vec)
        #new_omega = 0
        new_phi, new_omega = self.get_phi_and_omega(new_pos_vec, new_vel_vec, old_phi, old_omega, old_c_in, delta_time)

        return new_pos_vec, new_vel_vec, new_phi, new_omega

    def get_phi_and_omega(self, pos_cg, vel_cg, old_phi, old_omega, c_in, delta_time):
        new_omega = self.get_angular_velocity(pos_cg, vel_cg, old_phi, old_omega, c_in, delta_time)
        new_phi = old_phi + new_omega * delta_time
        return new_phi, new_omega

    ####################################################################################################################################################
    # Calculate forces

    def get_pin_friction(self, pos_cg, vel_cg, phi, c_in):
        """
        Purpose: Calculate friction force acting on pin from rail
        Formula: F_pin = mu_pin * L,    L = lateral force from rail on pin
        Returns:
            f2_vec -- ndarray containing the components of the pin friction force acting on the car (in x-, y- and z-direction)
        """

        l_vec = self.get_lateral_pin_force(pos_cg, vel_cg, phi, c_in)
        L = np.linalg.norm(l_vec)
        f2_vec_ = np.asarray([-self.mu_pin * L, 0, 0])

        if np.linalg.norm(vel_cg) < TOL:
            return np.zeros_like(f2_vec_)

        f2_vec = self.rotate(f2_vec_, self.get_gamma_angle(pos_cg, phi))

        return f2_vec

    def get_lateral_friction(self, pos_cg, vel_cg, phi, c_in):
        """
        Purpose: Calculate friction force acting on tires from track
        Formula: F_tire = min(mu_tire * N, m * v^2 / r),  N = normal force
        Returns:
            f3_vec -- ndarray containing the components of the tire friction force acting on the car (in x-, y- and z-direction)
        """

        f3_vec = np.zeros(3)

        if isinstance(self.rail, model.TurnRail):

            rho_w = np.linalg.norm((self.rho_front_axel + self.rho_rear_axel) / 2)

            pin_torque = self.get_pin_torque(pos_cg, vel_cg, phi, c_in)

            sigma_skid = 1
            if np.dot(pin_torque, np.asarray([0, 0, 1])) < 0:
                sigma_skid = -1

            f_hjul_magnitude = np.linalg.norm(pin_torque) / rho_w
            f_hjul = - f_hjul_magnitude * sigma_skid * np.asarray([0, 1, 0])

            n_vec = self.get_normal_force(pos_cg, vel_cg, phi, c_in)
            N = np.linalg.norm(n_vec)
            skid_limit = self.mu_tire * N

            f_kin = - self.mu_kin * N * sigma_skid * np.asarray([0,1,0])

            if f_hjul_magnitude <= skid_limit:
                f3_vec = -f_hjul
            else:
                f3_vec = -f_hjul - f_kin

        if np.linalg.norm(vel_cg) < TOL:
            return np.zeros_like(f3_vec)
        return f3_vec

    def get_magnet_force(self, pos_cg, vel_cg, phi, c_in):
        """
        Purpose: Calculate force from lane acting on the car's magnet
        Returns:
            m_vec -- ndarray containing the components of the magnetic force acting on the car (in x-, y- and z-direction)
        """

        # TODO: Make valid for skidding car (skid => magnet not above rail)

        m_vec = np.asarray([0, 0, -self.mag_coeff])

        return m_vec

    def get_drag_force(self, pos_cg, vel_cg, phi, c_in):
        """
        Purpose: Calculate drag force acting on tires from track
        Formula: D = .5 * rho * A * C_d * v^2
        Returns:
            d_vec -- ndarray containing the components of the drag force acting on the car (in x-, y- and z-direction)
        """

        RHO = 1.2  # density of air
        D = .5 * RHO * self.area * self.drag_coeff * np.dot(vel_cg, vel_cg)
        d_vec_ = np.asarray([-D, 0, 0])

        if np.linalg.norm(vel_cg) < TOL:
            return np.zeros_like(d_vec_)

        d_vec = self.rotate(d_vec_, self.get_gamma_angle(pos_cg, phi))

        return d_vec

    def get_lateral_pin_force(self, pos_cg, vel_cg, phi, c_in):
        """
        Purpose: Calculate lateral force from the track acting on the car's pin
        Formula: sum(F_centrifugal) = lateral friction + lateral pin force,
                 sum(F_centrifugal) = ma = m * v^2 / r
                 => lateral pin force = m * v^2 / r - lateral friction
        Returns:
            l_vec -- ndarray containing the components of the tire friction force acting on the car (in x-, y- and z-direction)
        """

        """
        rail_center_to_cg_vec = self.get_rail_center_to_cg()
        radial_unit_vector = rail_center_to_cg_vec / np.linalg.norm(rail_center_to_cg_vec)
        radial_speed = np.dot(self.vel_vec, radial_unit_vector)

        l_vec_magnitude = 1 / np.cos(self.ksi) * (self.mass * (radial_speed ** 2) / self.rail.get_lane_radius(self.lane) - np.dot(self.get_lateral_friction(), radial_unit_vector))

        l_vec_ = np.asarray([0, -l_vec_magnitude, 0])

        l_vec = self.rotate(l_vec_, self.get_gamma_angle())
        """

        centripetal_magnitude = self.mass * np.dot(vel_cg, vel_cg)

        other_forces = ( self.get_magnet_force(pos_cg, vel_cg, phi, c_in)
                       + self.get_gravity_force(pos_cg, vel_cg, phi, c_in)
                       + self.get_normal_force(pos_cg, vel_cg, phi, c_in)
                       + self.get_thrust_force(pos_cg, vel_cg, phi, c_in)
                       + self.get_axle_friction(pos_cg, vel_cg, phi, c_in)
                       + self.get_rolling_resistance(pos_cg, vel_cg, phi, c_in)
                       + self.get_motor_brake_force(pos_cg, vel_cg, phi, c_in)
                       + self.get_drag_force(pos_cg, vel_cg, phi, c_in) )
        other_forces_global = self.rotate(other_forces, -phi)

        # TODO: Other forces should include pin friction and wheel friction, but we have left them out to avoid coupled equations

        r_ccg = self.get_rail_center_to_cg(pos_cg)
        e_ccg = r_ccg / np.linalg.norm(r_ccg)

        l_vec_magnitude = (1/np.cos(self.get_beta_angle(pos_cg, phi))) * (centripetal_magnitude - np.dot(other_forces_global, e_ccg))

        r_cp = self.get_rail_center_to_pin(pos_cg, phi)
        global_unit_vec = r_cp / np.linalg.norm(r_cp)
        local_unit_vec = -self.rotate(global_unit_vec, phi)

        return l_vec_magnitude * local_unit_vec

    def get_centrifugal_force(self, pos_cg, vel_cg, phi, c_in):
        """
        Purpose: Calculate centrifugal force experienced by the car
        Formula: F = ma = mv^2/r
        Returns:
            cent_vec -- ndarray containing the components of the centrifugal force experienced by the car (in x-, y- and z-direction)
        """

        cent_vec = np.zeros(3)

        # TODO: Invalid radius, car can follow circular paths of different radii

        if isinstance(self.rail, model.TurnRail):  # Non-zero only if car is on a TurnRail
            cent_magnitude = np.dot(vel_cg, vel_cg) * self.mass / self.rail.get_lane_radius(self.lane)
            cent_vec = -self.rail.direction * np.asarray([0, cent_magnitude, 0])

        return cent_vec

    ####################################################################################################################
    # Calculate momenta

    def get_pin_torque(self, pos_cg, vel_cg, phi, c_in):
        return np.cross(self.rho_pin, (self.get_lateral_pin_force(pos_cg, vel_cg, phi, c_in) + self.get_pin_friction(pos_cg, vel_cg, phi, c_in)))

    def get_wheel_torque(self, pos_cg, vel_cg, phi, c_in):
        rho_wheel = (self.rho_front_axel + self.rho_rear_axel) / 2
        lat_fric_vec = self.get_lateral_friction(pos_cg, vel_cg, phi, c_in)
        return np.cross(rho_wheel, lat_fric_vec)

    def get_total_torque(self, pos_cg, vel_cg, phi, c_in):
        return np.zeros(3) #self.get_pin_torque(pos_cg, vel_cg, phi, c_in) + self.get_wheel_torque(pos_cg, vel_cg, phi, c_in)

    def get_angular_acceleration(self, pos_cg, vel_cg, phi, c_in):
        return np.dot(np.linalg.inv(self.inertia), self.get_total_torque(pos_cg, vel_cg, phi, c_in))

    def get_angular_velocity(self, pos_cg, vel_cg, phi, omega, c_in, delta_time):
        return omega + self.get_angular_acceleration(pos_cg, vel_cg, phi, c_in)[2] * delta_time

    ####################################################################################################################
    # Helper functions

    def rotate(self, vector, angle):
        rot_matrix = np.asarray([[np.cos(angle), np.sin(angle), 0], [-np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        rotated_vector = np.dot(rot_matrix, vector)
        return rotated_vector

    def get_rail_center_to_cg(self, pos_cg):
        rail_center = self.rail.get_rail_center()
        return pos_cg - rail_center

    def get_rail_center_to_pin(self, pos_cg, phi):
        #print("Rho pin:", self.rotate(self.rho_pin, -phi))
        #print("r_ccg:", self.get_rail_center_to_cg(pos_cg))
        #print("r_cp:", self.get_rail_center_to_cg(pos_cg) + self.rotate(self.rho_pin, -phi))
        return self.get_rail_center_to_cg(pos_cg) + self.rotate(self.rho_pin, -phi)

    def get_pin_position(self, pos_cg, phi):
        return pos_cg + self.rotate(self.rho_pin, -phi)

    def get_pin_velocity(self):
        pass

    def get_pin_acceleration(self):
        pass

    def get_gamma_angle(self, pos_cg, phi):
        # TODO: Update using arctan2?
        r_cp = self.get_rail_center_to_pin(pos_cg, phi)
        e_cp = r_cp / np.linalg.norm(r_cp)
        e_y = self.rotate(np.asarray([0, 1, 0]), -phi)
        dot_product = np.dot(e_cp, e_y)
        gamma_sign = 0
        if isinstance(self.rail, model.TurnRail):
            if abs(np.dot(self.get_rail_center_to_cg(pos_cg), r_cp)) > self.rail.get_lane_radius(self.lane):
                gamma_sign = 1
            else:
                gamma_sign = -1

            if self.rail.direction == model.TurnRail.Left:
                dot_product *= -1

        gamma = np.arccos(dot_product) * gamma_sign
        #print("e_cp:", e_cp)
        #print("e_y:", e_y)
        #print("dot product:", dot_product)
        #print(np.arccos(dot_product))
        return gamma

    def get_beta_angle(self, pos_cg, phi):
        # TODO: Update using arctan2?
        r_cp = self.get_rail_center_to_pin(pos_cg, phi)
        r_ccg = self.get_rail_center_to_cg(pos_cg)
        return np.arccos(np.dot(r_cp, r_ccg)/(np.linalg.norm(r_cp)*np.linalg.norm(r_ccg)))
