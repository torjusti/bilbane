import numpy as np
from model import model
from model.rk4 import rk4_step

G_ACC = 9.81
RHO   = 1.2  # density of air
TOL = 0.001



class PointMassCar:
    # Boolean to check if car is treated as point mass.
    is_point_mass = None
    # Boolean to check if car is locked to track.
    track_locked = None
    # Run-off criterion for point mass car.
    MAX_CENTRIFUGAL_FORCE = None

    # Crash state variables
    is_crashed = None  # Boolean to check if car has crashed.
    crash_time = None  # Time since the car crashed.

    # Physics state variables
    pos_vec = None  # Position of the car in the global coordinate system.
    vel_vec = None  # Velocity of the car in the global coordinate system.
    phi = None      # Yaw of the car in the global coordinate system.
    gamma = None    # Angle between x-axis and x'-axis

    # System variables
    lane = None  # Lane in which the car is driving.
    track = None  # Track on which the car is situated.
    rail = None  # Track section on which the car is situated.
    rail_progress = None # How far the car has driven along the current rail, normalized by the length of that rail.
    laps_completed = None # Number of laps the car has completed since previous crash.

    # Controller variables
    controller_input = None  # Value in interval [0,1].
    key_control = None  # True if car should be controlled by keyboard keys.
    controller = None  # Controller for letting AI control car

    # Car properties
    mass        = None  # kg
    area        = None  # m^2
    drag_coeff  = None  # dimensionless
    mag_coeff   = None  # N
    motor_eta   = None  # dimensionless
    mu_tire     = None  # dimensionless
    mu_pin      = None  # dimensionless
    mu_roll     = None  # dimensionless
    mu_axle     = None  # dimensionless
    motor_coeff = None  # N/(m/s)
    max_power   = None  # W

    def __init__(self, lane, track, key_control=False, track_locked=False):
        self.is_point_mass = True
        self.track_locked = track_locked
        self.MAX_CENTRIFUGAL_FORCE = 2

        self.is_crashed = False
        self.crash_time = 0

        self.pos_vec = np.asarray([0, lane * model.Rail.LANE_LANE_DIST / 2, 0])
        self.vel_vec = np.zeros(3)
        self.phi = 0.0
        self.gamma = 0.0

        self.lane  = lane
        self.track = track
        self.rail = track.rails[0]
        self.rail_progress = 0
        self.laps_completed = 0

        self.controller_input = 0
        self.key_control = key_control
        self.controller = None

        self.mass        = 0.08       # kg
        self.area        = 0.002268   # m^2
        self.drag_coeff  = 0.35       # dimensionless
        self.mag_coeff   = 1.0        # N
        self.motor_eta   = .95        # dimensionless
        self.mu_tire     = .9         # dimensionless
        self.mu_pin      = .04        # dimensionless
        self.mu_roll     = .01        # dimensionless
        self.mu_axle     = .1         # N
        self.motor_coeff = .1         # N/(m/s)
        self.max_power   = .5         # W


    def update_state(self, delta_time):
        """
        Purpose: Update car variables for new time step.
        Args:
            delta_time (float) -- size of time step (in seconds).
        Returns:
            void
        """

        # Crash check
        self.is_crashed = self.crash_check()
        if self.is_crashed:
            print("Ohhhh noooo")

        # Make velocity tangential to track
        #if self.track_locked:
        speed = np.linalg.norm(self.vel_vec)
        local_vel_vec = np.asarray([speed, 0, 0])
        self.vel_vec = self.rotate(local_vel_vec, -self.phi)

        # Update state given result of crash check
        if self.is_crashed and self.crash_time > 1:
            self.reset()
        elif self.is_crashed:
            self.crash_update(delta_time)
        else:
            self.physics_update(delta_time)

    def crash_check(self):
        return np.linalg.norm(self.get_centrifugal_force(self.pos_vec, self.vel_vec, self.phi, self.controller_input)) >= self.MAX_CENTRIFUGAL_FORCE

    def reset(self):
        """
        Purpose: Reset all car variables after a crash has completed.
        Args:
        Returns:
            void
        """

        self.is_crashed = False
        self.crash_time = 0

        self.pos_vec = np.asarray([0, self.lane * model.Rail.LANE_LANE_DIST / 2, 0])
        self.vel_vec = np.zeros(3)
        self.phi = 0.0

        self.rail = self.track.rails[0]
        self.rail_progress = 0
        self.laps_completed = 0

        self.controller_input = 0

    def crash_update(self, delta_time):
        """
        Purpose: Update position and orientation of car during crash animation.
        Args:
            delta_time (float) -- size of time step (in seconds).
        Returns:
            void
        """
        rail = self.rail
        self.crash_time += delta_time

        if isinstance(rail, model.StraightRail):
            crash_angle = rail.global_angle
            self.pos_vec[0] += np.linalg.norm(self.vel_vec) * delta_time * np.cos(crash_angle)
            self.pos_vec[1] += np.linalg.norm(self.vel_vec) * delta_time * np.sin(crash_angle)
            self.phi += 2 * np.linalg.norm(self.vel_vec) * delta_time
        elif isinstance(rail, model.TurnRail):
            crash_angle = rail.global_angle + rail.angle * self.rail_progress * rail.direction

            self.pos_vec[0] += np.linalg.norm(self.vel_vec) * delta_time * np.cos(crash_angle)
            self.pos_vec[1] += np.linalg.norm(self.vel_vec) * delta_time * np.sin(crash_angle)
            self.phi += 2 * rail.direction * np.linalg.norm(self.vel_vec) * delta_time

    def physics_update(self, delta_time):
        """
        Purpose: Update the car's position and orientation according to the physics model.
        Args:
            delta_time (float) -- size of time step (in seconds).
        Returns:
            void
        """
        new_pos_vec, new_vel_vec, new_phi = self.get_physics_state(delta_time)

        # Update rail progress based on new velocity from physics calculations
        self.rail_progress = self.get_rail_progress(new_pos_vec, new_vel_vec, delta_time)

        # Overwrite new_pos_vec and new_phi if locked to track
        if self.track_locked:
            rail = self.rail
            # Override new_pos_vec and new_phi based on updated rail progress
            if isinstance(rail, model.StraightRail):
                new_pos_vec[0] = rail.global_x + np.cos(rail.global_angle) * self.rail_progress * rail.length
                new_pos_vec[1] = rail.global_y + np.sin(rail.global_angle) * self.rail_progress * rail.length

                new_pos_vec[0] += np.cos(rail.global_angle + np.pi / 2 * self.lane) * model.Rail.LANE_LANE_DIST / 2
                new_pos_vec[1] += np.sin(rail.global_angle + np.pi / 2 * self.lane) * model.Rail.LANE_LANE_DIST / 2
            elif isinstance(rail, model.TurnRail):
                circle_x, circle_y, initial_angle = self.track._get_turn_circle(rail)

                angle = initial_angle + rail.angle * self.rail_progress * rail.direction

                new_pos_vec[0] = circle_x + rail.radius * np.cos(angle)
                new_pos_vec[1] = circle_y + rail.radius * np.sin(angle)

                new_phi = rail.global_angle + rail.angle * self.rail_progress * rail.direction

                new_pos_vec[0] += np.cos(new_phi + np.pi / 2 * self.lane) * model.Rail.LANE_LANE_DIST / 2
                new_pos_vec[1] += np.sin(new_phi + np.pi / 2 * self.lane) * model.Rail.LANE_LANE_DIST / 2

        # Move to next rail if reached end of current rail
        if self.rail_progress == 1:
            self.rail = self.rail.next_rail
            self.rail_progress = 0

        # Update state
        self.pos_vec = new_pos_vec
        self.vel_vec = new_vel_vec
        self.phi = new_phi

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
        old_c_in = self.controller_input

        # Calculate new state
        new_pos_vec, new_vel_vec = self.get_new_pos_and_vel(old_pos_vec, old_vel_vec, old_phi, old_c_in, delta_time)
        new_phi = self.get_phi(new_pos_vec)

        return new_pos_vec, new_vel_vec, new_phi

    def get_new_pos_and_vel(self, old_pos_vec, old_vel_vec, old_phi, old_c_in, delta_time):
        """
        Purpose: Use a numerical method to find new position and velocity, given the car's old state.
        Args:
            old_pos_vec (ndarray, shape=[3,]) -- old car position, (X,Y,Z) (in meters).
            old_vel_vec (ndarray, shape=[3,]) -- old car velocity (in meters per second).
            old_phi (float) -- old yaw of car (in radians).
            old_c_in (float) -- controller_input at current time step (dimensionless).
            delta_time (float) -- size of time step (in seconds).
        Returns:
            new_pos_vec (ndarray, shape=[3,]) -- new car position, (X, Y, Z) (in meters).
            new_vel_vec (ndarray, shape=[3,]) -- new car velocity (in meters per second).
        """

        old_y = np.concatenate((old_pos_vec, old_vel_vec), axis=None)
        #new_y = rk4_step(old_y, old_c_in, delta_time, self.dxdt, self.dvdt)
        new_y = self.newmark_beta_step(old_y, old_phi, old_c_in, delta_time)

        print("New position:", new_y[:3])
        print("New velocity:", new_y[3:])

        return new_y[:3], new_y[3:]

    def get_phi(self, pos_vec):
        """
        Purpose: Calculate yaw of the car (i.e. rotation relative to global coordinate system) given a car position.
        Args:
            pos_vec (ndarray, shape=[3,]) -- car position, (X,Y,Z (in meters).
        Returns:
            phi (float) -- calculated yaw
        """

        # TODO: Check if assumes car always on track

        # If on a turn
        if isinstance(self.rail, model.TurnRail):
            rail_center_vec = self.rail.get_rail_center()
            radial_vec = pos_vec - rail_center_vec
            tangent_vec = self.rotate(radial_vec, -self.rail.direction * np.pi / 2)
            phi = np.arctan2(tangent_vec[1], tangent_vec[0])

        # Else we are on a straight
        else:
            phi = self.rail.global_angle

        return phi

    def newmark_beta_step(self, old_y, old_phi, old_c_in, delta_time):
        """
        Purpose: Calculate new state vector using the newmark-beta method.
        Args:
            old_y (ndarray, shape=[6,]) -- old state vector containing position (first three components) and velocity (last three components)
            old_c_in (float) -- controller input at current time step
            delta_time (float) -- size of time step
        Returns:
            (ndarray, shape=[6,]) -- new state vector containing position (first three components) and velocity (last three components)
        """
        old_pos_vec = old_y[:3]
        old_vel_vec = old_y[3:]

        new_acc_vec_local = self.get_total_force(old_pos_vec, old_vel_vec, old_phi, old_c_in) / self.mass
        new_acc_vec_global = self.rotate(new_acc_vec_local, -old_phi)
        new_vel_vec_global = old_vel_vec + new_acc_vec_global * delta_time
        new_pos_vec_global = old_pos_vec + old_vel_vec * delta_time + .5 * new_acc_vec_global * (delta_time ** 2)

        return np.concatenate((new_pos_vec_global, new_vel_vec_global), axis=None)

    def dxdt(self, y):
        """
        Purpose: Calculate temporal derivative of position vector. (Helper for RK4.)
        Args:
            y (ndarray, shape=[6,]) -- state vector containing position (first three components) and velocity (last three components)
        Returns:
            (ndarray, shape=[3,]) -- temporal derivative of position vector
        """
        return y[3:]

    def dvdt(self, y, c_in):
        """
        Purpose: Calculate temporal derivative of velocity vector. (Helper for RK4.)
        Args:
            y (ndarray, shape=[6,]) -- state vector containing position (first three components) and velocity (last three components)
            c_in (float) -- controller input
        Returns:
            global_acc_vec (ndarray, shape=[3,]) -- temporal derivative of velocity vector
        """
        phi = self.get_phi(y[:3])
        local_acc_vec = self.get_total_force(y[:3], y[3:], phi, c_in) / self.mass
        global_acc_vec = self.rotate(local_acc_vec, -phi)
        return global_acc_vec

    def get_rail_progress(self, pos, vel, delta_time):
        """
        Purpose: Calculate the car's current rail progress given its current position and velocity.
        Args:
            pos (ndarray, shape=[3,] -- global car position (in meters)
            vel (ndarray, shape=[3,] -- global car velocity (in meters per second)
            delta_time (float) -- size of time step (in seconds)
        Returns:
            rail_progress (float) -- current rail progress (dimensionless number in range [0,1])
        """

        rail_progress = None

        if self.track_locked:
            rail_progress = self.rail_progress
            rail_progress += np.linalg.norm(vel) * delta_time / self.rail.get_length(self.lane)
        else:
            rail_start_vec = np.asarray([self.rail.global_x, self.rail.global_y, 0])
            if isinstance(self.rail, model.TurnRail):
                rail_center_vec = self.rail.get_rail_center()
                cent_to_start_vec = rail_start_vec - rail_center_vec
                cent_to_cg_vec = pos - rail_center_vec
                angle = np.arctan2(cent_to_cg_vec[1], cent_to_cg_vec[0]) - np.arctan2(cent_to_start_vec[1], cent_to_start_vec[0])
                if angle < 0:
                    angle = -angle
                rail_progress = angle * self.rail.get_lane_radius(self.lane) / self.rail.get_length(self.lane)
            elif isinstance(self.rail, model.StraightRail):
                unit_vec_along_track = self.rotate(np.asarray([1, 0, 0]), self.rail.global_angle)
                rail_start_to_cg_vec = pos - rail_start_vec
                rail_progress = np.linalg.norm(np.dot(unit_vec_along_track, rail_start_to_cg_vec)) / self.rail.get_length(self.lane)

        rail_progress = min(rail_progress, 1)
        return rail_progress

    # ---------------------------------------------------------------------------
    # Calculate forces

    def get_total_force(self, pos, vel, phi, c_in):
        """
        Purpose: Calculate total force on car.
        Args:
            pos (ndarray, shape=[3,]) -- car position (in meters)
            vel (ndarray, shape=[3,]) -- car velocity (in meters per second)
            phi (float) -- car's yaw relative to global coordinate system (in radians)
            c_in (float) -- controller input (dimensionless)
        Returns:
            total_force_vec (ndarray, shape=[3,]) -- force vector acting on the car (in Newton)
        """

        rolling_resistance = self.get_rolling_resistance(pos, vel, phi, c_in)
        motor_brake_force  = self.get_motor_brake_force(pos, vel, phi, c_in)
        axle_friction      = self.get_axle_friction(pos, vel, phi, c_in)
        pin_friction       = self.get_pin_friction(pos, vel, phi, c_in)
        lateral_friction   = self.get_lateral_friction(pos, vel, phi, c_in)
        magnet_force       = self.get_magnet_force(pos, vel, phi, c_in)
        gravity_force      = self.get_gravity_force(pos, vel, phi, c_in)
        normal_force       = self.get_normal_force(pos, vel, phi, c_in)
        thrust_force       = self.get_thrust_force(pos, vel, phi, c_in)
        drag_force         = self.get_drag_force(pos, vel, phi, c_in)
        lateral_pin_force  = self.get_lateral_pin_force(pos, vel, phi, c_in)


        total_force_vec = ( magnet_force
                          + gravity_force
                          + normal_force
                          + thrust_force
                          + lateral_pin_force
                          + lateral_friction
                          + pin_friction
                          + axle_friction
                          + rolling_resistance
                          + motor_brake_force
                          + drag_force )

        """
        if c_in != 0:
            print("Rolling resitance:", rolling_resistance, "\n",
                  "Motor brake force:", motor_brake_force, "\n",
                  "Axle friction    :", axle_friction, "\n",
                  "Pin friction     :", pin_friction, "\n",
                  "Lateral friction :", lateral_friction, "\n",
                  "Magnet force     :", magnet_force, "\n",
                  "Gravity force    :", gravity_force, "\n",
                  "Normal force     :", normal_force, "\n",
                  "Thrust force     :", thrust_force, "\n",
                  "Drag force       :", drag_force, "\n",
                  "Lateral pin force:", lateral_pin_force, "\n",
                  "Total force:", total_force_vec, "\n")
        """

        return total_force_vec

    def get_rolling_resistance(self, pos, vel, phi, c_in):
        """
        Purpose: Calculate rolling resistance acing on the car
        Formula: F_roll = mu_roll * N,   N = normal force
        Returns:
            f1_vec -- ndarray containing the components of the rolling resistance acting on the car (in x-, y- and z-direction)
        """

        n_vec  = self.get_normal_force(pos, vel, phi, c_in)
        N      = np.linalg.norm(n_vec)
        track_friction = self.mu_roll*N

        f1_vec = np.asarray([-track_friction, 0, 0])

        if np.linalg.norm(vel) < TOL:
            return np.zeros_like(f1_vec)

        return f1_vec

    def get_motor_brake_force(self, pos, vel, phi, c_in):
        """
        Purpose: Calculate motor brake force
        Formula: Braking torque proportional to speed
        Return:
            mbrake_vec -- ndarray containing the components of the motor brake force acting on the car (in x-, y- and z-direction)
        Source https://www.ormec.com/Portals/ormec/Library/Documents/Controllers/Orion/TechNotes/tn038.pdf
        """

        mbrake_vec = np.zeros(3)

        if c_in == 0:
            mbrake_vec[0] = -self.motor_coeff*np.linalg.norm(vel)

        if np.linalg.norm(vel) < TOL:
            mbrake_vec = np.zeros_like(mbrake_vec)

        return mbrake_vec

    def get_axle_friction(self, pos, vel, phi, c_in):
        """
        Purpose: Calculate axel friction force
        Return:
            axel_fric_vec -- ndarray containing the components of the axel friction force acting on the car (in x-, y- and z-direction)
        """

        axle_fric_vec = np.asarray([-self.mu_axle, 0, 0])

        if np.linalg.norm(vel) < TOL:
            axle_fric_vec =  np.zeros_like(axle_fric_vec)

        return axle_fric_vec


    def get_pin_friction(self, pos, vel, phi, c_in):
        """
        Purpose: Calculate friction force acting on pin from rail
        Formula: F_pin = mu_pin * L,    L = lateral force from rail on pin
        Returns:
            f2_vec -- ndarray containing the components of the pin friction force acting on the car (in x-, y- and z-direction)
        """


        l_vec  = self.get_lateral_pin_force(pos, vel, phi, c_in)
        L      = np.linalg.norm(l_vec)
        f2_vec = np.asarray([-self.mu_pin*L, 0, 0])

        if np.linalg.norm(vel) < TOL:
            f2_vec =  np.zeros_like(f2_vec)

        return f2_vec

    def get_lateral_friction(self, pos, vel, phi, c_in):
        """
        Purpose: Calculate friction force acting on tires from track
        Formula: This force is unphysical for a point mass, and is hence set to zero
            f3_vec -- ndarray containing the components of the tire friction force acting on the car (in x-, y- and z-direction)
        """

        f3_vec = np.zeros(3)

        return f3_vec

    def get_magnet_force(self, pos, vel, phi, c_in):
        """
        Purpose: Calculate force from lane acting on the car's magnet
        Returns:
            m_vec -- ndarray containing the components of the magnetic force acting on the car (in x-, y- and z-direction)
        """

        # TODO: Make valid for skidding car (skid => magnet not above rail)

        m_vec = np.asarray([0, 0, -self.mag_coeff])

        return m_vec

    def get_gravity_force(self, pos, vel, phi, c_in):
        """
        Purpose: Calculate gravitational force acting on the car
        Formula: G = mg
        Returns:
            g_vec -- ndarray containing the components of the gravitational force acting on the car (in x-, y- and z-direction)
        """

        g_vec = np.asarray([0, 0, -self.mass * G_ACC])

        return g_vec

    def get_normal_force(self, pos, vel, phi, c_in):
        """
        Purpose: Calculate friction force acting on tires from track
        Formula: sum(F_z) = 0 => N = - (G + m_vec)
        Returns:
            n_vec -- ndarray containing the components of the normal force acting on the car (in x-, y- and z-direction)
        """

        # TODO: Extend to 3D
        # In 3D, we do not necessarily have 0 net force in z-dir,
        # which the current implementation assumes.

        n_vec = - (self.get_magnet_force(pos, vel, phi, c_in) + self.get_gravity_force(pos, vel, phi, c_in))

        return n_vec

    def get_thrust_force(self, pos, vel, phi, c_in):
        """
        Purpose: Calculate thrust force acting on the car's tires from track, due to forced rotation of the tires (by the car's motor)
        Formula: T = C*P/v
        Returns:
            t_vec -- ndarray containing the components of the thrust force acting on the car (in x-, y- and z-direction)
        """

        T = c_in*self.max_power/max(0.12, np.linalg.norm(vel))
        t_vec = np.asarray([T, 0, 0])

        return t_vec

    def get_drag_force(self, pos, vel, phi, c_in):
        """
        Purpose: Calculate drag force acting on tires from track
        Formula: D = .5 * rho * A * C_d * v^2
        Returns:
            d_vec -- ndarray containing the components of the drag force acting on the car (in x-, y- and z-direction)
        """

        # TODO: Make valid for skidding car (skid => area and drag coefficient change)

        D     = .5 * RHO * self.area * self.drag_coeff * np.dot(vel, vel)
        d_vec = np.asarray([-D, 0, 0])

        if np.linalg.norm(vel) < TOL:
            d_vec = np.zeros_like(d_vec)

        return d_vec

    def get_lateral_pin_force(self, pos, vel, phi, c_in):
        """
        Purpose: Calculate lateral force from the track acting on the car's pin
        Formula: sum(F_centrifugal) = lateral friction + lateral pin force,
                 sum(F_centrifugal) = ma = m * v^2 / r
                 => lateral pin force = m * v^2 / r - lateral friction
        Returns:
            l_vec -- ndarray containing the components of the tire friction force acting on the car (in x-, y- and z-direction)
        """

        l_vec = np.zeros(3)

        if isinstance(self.rail, model.TurnRail): # Non-zero only if car is on a TurnRail
            cent_vec = self.get_centrifugal_force(pos, vel, phi, c_in)
            l_vec_magnitude = np.linalg.norm(cent_vec) - np.linalg.norm(self.get_lateral_friction(pos, vel, phi, c_in))
            l_vec[1] = self.rail.direction * l_vec_magnitude

        return l_vec

    def get_centrifugal_force(self, pos, vel, phi, c_in):
        """
        Purpose: Calculate centrifugal force experienced by the car
        Formula: F = ma = mv^2/r
        Returns:
            cent_vec -- ndarray containing the components of the centrifugal force experienced by the car (in x-, y- and z-direction)
        """

        cent_vec = np.zeros(3)

        local_vel_vec = self.rotate(vel, phi)
        tangential_vel = local_vel_vec[0]

        # TODO: Make radius not assume car is on track

        if isinstance(self.rail, model.TurnRail): # Non-zero only if car is on a TurnRail
            cent_magnitude = (tangential_vel**2) * self.mass / self.rail.get_lane_radius(self.lane)
            cent_vec = self.rail.direction * np.asarray([0, cent_magnitude, 0])

        return cent_vec

    # ---------------------------------------------------------------------------
    # Helper functions

    def rotate(self, vector, angle):
        """
        Purpose: Helper function for rotating a vector by a given angle.
        Args:
            vector (ndarray, shape=[3,]) -- vector to be rotated
            angle (float) -- angle with which to rotate the vector (in radians)
        Returns:
            rotated_vector (ndarray, shape=[3,]) -- the input vector rotated by the input angle
        """
        rot_matrix = np.asarray([[np.cos(angle), np.sin(angle), 0], [-np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        rotated_vector = np.dot(rot_matrix, vector)
        return rotated_vector

    def is_reversing(self, pos, vel):
        local_velocity = self.rotate(vel, self.phi)
        return local_velocity[0] < 0
