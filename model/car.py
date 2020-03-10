import numpy as np
from model import model

G_ACC = 9.81
RHO   = 1.2  # density of air
TOL = 0.001


class Car:
    # Run-off criterion for point mass car
    MAX_CENTRIFUGAL_FORCE = 2

    # Maximum voltage from track to car
    TRACK_VOLTAGE = 12

    # Boolean to check if car is treated as point mass
    is_point_mass = None

    # Boolean to check if car has crashed
    is_crashed = None

    # Time since the car crashed
    crash_time = 0

    # Position of the car in the global coordinate system.
    pos_vec = None
    vel_vec = None
    acc_vec = None

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

    # TODO: Use returned angle.
    phi = None

    # Track section on which the car is situated.
    rail = None

    # How far along the car is on the current rail.
    rail_progress = 0

    # Lane and track the car is on
    lane = None
    track = None

    # Input from the controller. Will be converted to a voltage which drives the car
    controller_input = None # Value in interval [0,1]

    def __init__(self, lane, track, key_control=False):
        self.lane  = lane
        self.track = track
        self.rail = track.rails[0]

        self.key_control = key_control

        self.controller_input = 0

        self.is_crashed   = False
        self.is_point_mass = True

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
        self.max_power   = .5        # W

        self.pos_vec = np.asarray([0, self.lane * model.Rail.LANE_LANE_DIST / 2, 0])
        self.vel_vec = np.zeros(3)
        self.acc_vec = np.zeros(3)
        self.phi = 0.0

    def get_new_state(self, delta_time):
        """
        Purpose: Get new state of the car
        Args:
            delta_time -- time step in seconds
        Returns:
            new_pos_vec -- ndarray containing new car position (x,y,z)
            new_vel_vec -- ndarray containing new car velocity (u, v, w)
            new_angle_vec -- ndarray containing new rotation of car relative to global coordinate system (roll, pitch, yaw)
        """

        new_pos_vec   = None
        new_angle_vec = None
        new_vel_vec   = None

        if not self.is_crashed:
            new_pos_vec   = self.get_new_pos(delta_time)
            new_vel_vec   = self.get_new_vel(delta_time)
            new_angle_vec = self.get_new_angles(new_pos_vec, self.rail)

        return new_pos_vec, new_vel_vec, new_angle_vec

    def get_new_pos(self, delta_time):
        """
        Purpose: Get new position of the car
        Args:
            delta_time -- time step in seconds
        Returns:
            new_pos_vec -- ndarray containing new car position (x,y,z)
        """

        new_acc_vec = self.get_total_force() / self.mass
        new_acc_vec_global = self.rotate(new_acc_vec, -self.phi)
        new_vel_vec_global = self.get_new_vel(delta_time)
        new_pos_vec = self.pos_vec + new_vel_vec_global * delta_time + .5*new_acc_vec_global*(delta_time**2)

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

        new_acc_vec_global = self.rotate(new_acc_vec, -self.phi)

        new_vel_vec_global = self.vel_vec + new_acc_vec_global*delta_time

        return new_vel_vec_global

    def get_new_angles(self, new_pos_vec, new_rail):
        """
        Purpose: Get new rotation of the car relative to global coordinate system
        Args:
            new_pos_vec -- ndarray containing new car position (x,y,z)
            new_rail -- instance of Rail, the rail on which the car is located when it has moved to new_pos_vec
        Returns:
            new_angle_vec -- ndarray containing new rotation of car relative to global coordinate system (roll, pitch, yaw)
        """

        if self.is_point_mass:
            angle_vec = self.get_new_point_angles(new_pos_vec, new_rail)
        else:
            angle_vec = self.get_new_body_angles()

        return angle_vec

    def get_new_point_angles(self, new_pos_vec, new_rail):
        """
        Purpose: Get new rotation of the car relative to global coordinate system
        Assumptions:
            - Car is a point mass
            - 2D
        Args:
            new_pos_vec -- ndarray containing new car position (x,y,z)
            new_rail -- instance of Rail, the rail on which the car is located when it has moved to new_pos_vec
        Returns:
            new_angle_vec -- ndarray containing new rotation of car relative to global coordinate system (roll, pitch, phi)
        """

        # If on a turn
        if isinstance(new_rail, model.TurnRail):
            rail_center_vec = new_rail.get_rail_center()
            radial_vec = self.pos_vec - rail_center_vec
            rotation_matrix = np.asarray([[0, -1, 0], [1, 0, 0], [0, 0, 1]]) * self.rail.direction
            tangent_vec = np.dot(rotation_matrix, radial_vec)
            phi = np.arctan2(tangent_vec[1], tangent_vec[0])

        # Else we are on a straight
        else:
            phi = new_rail.global_angle

        new_angle_vec = np.asarray([0, 0, phi])
        return new_angle_vec

    def get_new_body_angles(self):
        return 0

    # ---------------------------------------------------------------------------
    # Calculate forces

    def get_total_force(self):
        """
        Purpose: Calculate total force on car, and check if it exceeds given force limit
        Returns:
            total_force_vec -- ndarray containing force acting on the car (in x-, y- and z-direction)
        """

        rolling_resistance = self.get_rolling_resistance()
        motor_brake_force  = self.get_motor_brake_force()
        axle_friction      = self.get_axle_friction()
        pin_friction       = self.get_pin_friction()
        lateral_friction   = self.get_lateral_friction()
        magnet_force       = self.get_magnet_force()
        gravity_force      = self.get_gravity_force()
        normal_force       = self.get_normal_force()
        thrust_force       = self.get_thrust_force()
        drag_force         = self.get_drag_force()
        lateral_pin_force  = self.get_lateral_pin_force()

        #print(motor_brake_force)

        total_force_vec = ( rolling_resistance
                          + motor_brake_force
                          + axle_friction
                          + pin_friction
                          + lateral_friction
                          + magnet_force
                          + gravity_force
                          + normal_force
                          + thrust_force
                          + drag_force
                          + lateral_pin_force )

        """
        print("Rolling resitance:", self.get_rolling_resistance(), "\n",
              "Motor brake force:", self.get_motor_brake_force(), "\n",
              "Axle friction    :", self.get_axle_friction(), "\n",
              "Pin friction     :", self.get_pin_friction(), "\n",
              "Lateral friction :", self.get_lateral_friction(), "\n",
              "Magnet force     :", self.get_magnet_force(), "\n",
              "Gravity force    :", self.get_gravity_force(), "\n",
              "Normal force     :", self.get_normal_force(), "\n",
              "Thrust force     :", self.get_thrust_force(), "\n",
              "Drag force       :", self.get_drag_force(), "\n",
              "Lateral pin force:", self.get_lateral_pin_force(), "\n",
              "Total force:", total_force_vec, "\n")
        """
        # Crash check
        if np.linalg.norm(self.get_centrifugal_force()) >= self.MAX_CENTRIFUGAL_FORCE:
            self.is_crashed = True

        return total_force_vec

    def get_rolling_resistance(self):
        """
        Purpose: Calculate rolling resistance acing on the car
        Formula: F_roll = mu_roll * N,   N = normal force
        Returns:
            f1_vec -- ndarray containing the components of the rolling resistance acting on the car (in x-, y- and z-direction)
        """

        # TODO: Make valid for skidding car

        n_vec  = self.get_normal_force()
        N      = np.linalg.norm(n_vec)
        track_friction = self.mu_roll*N

        f1_vec = np.asarray([-track_friction, 0, 0])

        if np.linalg.norm(self.vel_vec) < TOL or self.is_reversing():
            return np.zeros_like(f1_vec)

        return f1_vec

    def get_motor_brake_force(self):
        """
        Purpose: Calculate motor brake force
        Formula: Braking torque proportional to speed
        Return:
            mbrake_vec -- ndarray containing the components of the motor brake force acting on the car (in x-, y- and z-direction)
        # TODO: Update according to this: https://www.ormec.com/Portals/ormec/Library/Documents/Controllers/Orion/TechNotes/tn038.pdf
        """

        mbrake_vec = np.asarray([-self.motor_coeff*np.linalg.norm(self.vel_vec), 0, 0])

        if not self.controller_input == 0:
            mbrake_vec = np.zeros_like(mbrake_vec)

        if np.linalg.norm(self.vel_vec) < TOL or self.is_reversing():
            mbrake_vec = np.zeros_like(mbrake_vec)

        return mbrake_vec

    def get_axle_friction(self):
        """
        Purpose: Calculate axel friction force
        Return:
            axel_fric_vec -- ndarray containing the components of the axel friction force acting on the car (in x-, y- and z-direction)
        """

        axle_fric_vec = np.asarray([-self.mu_axle, 0, 0])

        if np.linalg.norm(self.vel_vec) < TOL or self.is_reversing():
            return np.zeros_like(axle_fric_vec)

        return axle_fric_vec


    def get_pin_friction(self):
        """
        Purpose: Calculate friction force acting on pin from rail
        Formula: F_pin = mu_pin * L,    L = lateral force from rail on pin
        Returns:
            f2_vec -- ndarray containing the components of the pin friction force acting on the car (in x-, y- and z-direction)
        """

        l_vec  = self.get_lateral_pin_force()
        L      = np.linalg.norm(l_vec)
        f2_vec = np.asarray([-self.mu_pin*L, 0, 0])

        if np.linalg.norm(self.vel_vec) < TOL or self.is_reversing():
            return np.zeros_like(f2_vec)

        return f2_vec

    def get_lateral_friction(self):
        """
        Purpose: Calculate friction force acting on tires from track
        Formula: F_tire = min(mu_tire * N, m * v^2 / r),  N = normal force
        Returns:
            f3_vec -- ndarray containing the components of the tire friction force acting on the car (in x-, y- and z-direction)
        """

        f3_vec = np.zeros(3)

        # TODO: Try to find a more meaningful expression for this force

        if isinstance(self.rail, model.TurnRail):
            n_vec     = self.get_normal_force()
            N         = np.linalg.norm(n_vec)
            centripetal_force = np.linalg.norm(self.get_centrifugal_force())
            f3_vec[1] = self.rail.direction * np.minimum(self.mu_tire * N, centripetal_force) # Friction cannot exceed centripetal force

        if np.linalg.norm(self.vel_vec) < TOL:
            return np.zeros_like(f3_vec)

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

    def get_gravity_force(self):
        """
        Purpose: Calculate gravitational force acting on the car
        Formula: G = mg
        Returns:
            g_vec -- ndarray containing the components of the gravitational force acting on the car (in x-, y- and z-direction)
        """

        g_vec = np.asarray([0, 0, -self.mass * G_ACC])

        return g_vec

    def get_normal_force(self):
        """
        Purpose: Calculate friction force acting on tires from track
        Formula: sum(F_z) = 0 => N = - (G + m_vec)
        Returns:
            n_vec -- ndarray containing the components of the normal force acting on the car (in x-, y- and z-direction)
        """

        # TODO: Extend to 3D
        # In 3D, we do not necessarily have 0 net force in z-dir,
        # which the current implementation assumes.

        n_vec = - (self.get_magnet_force() + self.get_gravity_force())

        return n_vec

    def get_thrust_force(self):
        """
        Purpose: Calculate thrust force acting on the car's tires from track, due to forced rotation of the tires (by the car's motor)
        Formula: T = C*P/v
        Returns:
            t_vec -- ndarray containing the components of the thrust force acting on the car (in x-, y- and z-direction)
        """

        T = self.controller_input*self.max_power/max(0.12, np.linalg.norm(self.vel_vec))
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
        D     = .5 * RHO * self.area * self.drag_coeff * np.dot(self.vel_vec, self.vel_vec)
        d_vec = np.asarray([-D, 0, 0])

        if np.linalg.norm(self.vel_vec) < TOL:
            return np.zeros_like(d_vec)

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

        if isinstance(self.rail, model.TurnRail): # Non-zero only if car is on a TurnRail
            #cent_vec = self.get_centrifugal_force()
            cent_vec = self.get_centrifugal_force()
            l_vec_magnitude = np.linalg.norm(cent_vec) - np.linalg.norm(self.get_lateral_friction())
            l_vec[1] = self.rail.direction * l_vec_magnitude

        return l_vec

    def get_centrifugal_force(self):
        """
        Purpose: Calculate centrifugal force experienced by the car
        Formula: F = ma = mv^2/r
        In principle one should use the tangential velocity only, but calculating the tangential component is in
        practice not any more accurate simply assuming that the velocity is solely tangential
        (presumably because the angles are not exact).
        Returns:
            cent_vec -- ndarray containing the components of the centrifugal force experienced by the car (in x-, y- and z-direction)
        """

        cent_vec = np.zeros(3)

        if isinstance(self.rail, model.TurnRail): # Non-zero only if car is on a TurnRail
            cent_magnitude = np.dot(self.vel_vec, self.vel_vec) * self.mass / self.rail.get_lane_radius(self.lane)
            cent_vec = self.rail.direction * np.asarray([0, cent_magnitude, 0])

        return cent_vec

    def get_car_track_angle(self):
        # TODO: Implement for car that is not a point mass
        return 0

    def rotate(self, vector, angle):
        rot_matrix = np.asarray([[np.cos(angle), np.sin(angle), 0], [-np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        rotated_vector = np.dot(rot_matrix, vector)
        return rotated_vector

    def is_reversing(self):
        local_velocity = self.rotate(self.vel_vec, self.phi)
        return local_velocity[0] < 0