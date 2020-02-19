import numpy as np
import model

G_ACC = 9.81

class Car:
    # Run-off criterion for point mass car
    MAX_LATERAL_FORCE = 100

    # Maximum voltage from track to car
    MAX_VOLTAGE = 16

    # Boolean to check if car is treated as point mass
    is_point_mass = None

    # Boolean to check if car has crashed
    is_chrashed = None

    # Position of the car in the global coordinate system.
    pos_vec = None
    vel_vec = None
    acc_vec = None

    mass       = None  # kg
    area       = None  # m^2
    drag_coeff = None  # dimensionless
    mag_coeff  = None  # N
    motor_eta  = None  # dimensionless
    mu_tire    = None  # dimensionless
    mu_pin     = None  # dimensionless
    mu_roll    = None  # dimensionless

    # Track section on which the car is situated.
    rail = None # TODO: When is this gonna be updated?

    # How far along the car is on the current rail.
    rail_progress = 0 # TODO: What is this here for?

    # Lane and track the car is on
    lane = None
    track = None

    # Input from the controller. Will be converted to a voltage which drives the car
    controller_input = None # Value in interval [0,1]

    def __init__(self, lane, track):
        self.lane  = lane
        self.track = track
        self.rail = track.rails[0]

        self.controller_input = 0

        self.is_chrashed   = False
        self.is_point_mass = True

        self.mass       = 0.08       # kg
        self.area       = 0.002268   # m^2
        self.drag_coeff = 0.35       # dimensionless
        self.mag_coeff  = 1.0        # N
        self.motor_eta  = .95        # dimensionless
        self.mu_tire    = .9         # dimensionless
        self.mu_pin     = .04        # dimensionless
        self.mu_roll    = .01        # dimensionless

        self.pos_vec = np.zeros(3)
        self.vel_vec = np.zeros(3)
        self.acc_vec = np.zeros(3)
    
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

        if (not self.is_chrashed):
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
        new_vel_vec = self.vel_vec + new_acc_vec*delta_time
        new_pos_vec = new_vel_vec*delta_time + .5*new_acc_vec*delta_time

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
        new_vel_vec = self.vel_vec + new_acc_vec*delta_time

        return new_vel_vec

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
            angle_vec = self.get_new_body_angles(new_pos_vec, new_rail)

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
            new_angle_vec -- ndarray containing new rotation of car relative to global coordinate system (roll, pitch, yaw)
        """

        # If on a turn
        if isinstance(new_rail, model.TurnRail):
            pos_vec_COR = new_rail.get_rail_center()
            lane_radius = new_rail.get_lane_radius(self.lane)
            left_turn   = (new_rail.direction == model.TurnRail.Left)
            
            pos_vec_rel    = new_pos_vec - pos_vec_COR # Vector from centre of rail to car
            pos_vec_up     = np.asarray([0, self.rail.direction * lane_radius, 0]) # Vector from centre of rail in positive y(local) coord
            dot_product    = np.dot(pos_vec_rel, pos_vec_up) # Dotproduct between the two
            relative_angle = np.arccos(np.clip(dot_product/(lane_radius*lane_radius), -1.0, 1.0)) # Angle between Y(global) and y(local) axis

            if pos_vec_rel[0] < 0: # If relative position vector points to the right
                yaw = relative_angle # Angle between X(global) and x(local) axis
            else:
                yaw = 2*np.pi - relative_angle # Angle between X(global) and x(local) axis
            
            if left_turn:
                yaw = yaw + np.pi # Left turn offsets the calculations above with 180 degrees because of flipped x(local) axis
            
            if (yaw >= 2*np.pi):
                yaw = yaw - 2*np.pi
            if (yaw < 0):
                yaw = yaw + 2*np.pi
        
        # Else we are on a straight
        else:
            yaw = new_rail.global_angle

        new_angle_vec = np.asarray([0, 0, yaw])
        return new_angle_vec

    def get_new_body_angles():
        pass

    # ---------------------------------------------------------------------------
    # Calculate forces

    def get_total_force(self):
        """
        Purpose: Calculate total force on car, and check if it exceeds given force limit
        Returns: 
            total_force_vec -- ndarray containing force acting on the car (in x-, y- and z-direction)
        """

        total_force_vec = ( self.get_rolling_resistance()
                          + self.get_pin_friction()
                          + self.get_lateral_friction()
                          + self.get_magnet_force()
                          + self.get_gravity_force()
                          + self.get_normal_force()
                          + self.get_thrust_force()
                          + self.get_drag_force()
                          + self.get_lateral_pin_force() )
        
        # Crash check
        if (total_force_vec[1] >= self.MAX_LATERAL_FORCE):
            self.is_chrashed = True

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
        f1_vec = np.asarray([-self.mu_roll*N, 0, 0])

        return f1_vec

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
            n_vec     = self.get_normal_force()
            N         = np.linalg.norm(n_vec)
            centripetal_force = self.mass*np.dot(self.vel_vec, self.vel_vec)/self.rail.get_lane_radius(self.lane)
            f3_vec[1] = self.rail.direction * np.minimum(self.mu_tire * N, centripetal_force) # Friction cannot exceed centripetal force

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

        T = self.motor_eta*(U**2)/R
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

        RHO   = 1.2 # density of air
        D     = .5 * RHO * self.area * self.drag_coeff * np.dot(self.vel_vec, self.vel_vec)
        d_vec = np.asarray([-D, 0, 0])

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
            cent_f = np.dot(self.vel_vec, self.vel_vec) * self.mass / self.rail.get_lane_radius(self.lane)
            l_vec = self.rail.direction * np.asarray([0, cent_f, 0]) - self.get_lateral_friction()

        return l_vec

    def get_car_track_angle(self):
        # TODO: Implement for car that is not a point mass
        return 0


if __name__ == "__main__":
    # Doing some testing here

    # ----------------------------------------------
    # Test set-up

    # 1st track

    # Define rails of 1st test track (starts with straight)
    test_rails = [
        model.StraightRail(200),
        model.TurnRail(100, np.pi, model.TurnRail.Left),
        model.TurnRail(25, 2 * np.pi, model.TurnRail.Right),
        model.StraightRail(200),
        model.TurnRail(100, np.pi, model.TurnRail.Left)
    ]

    # Define test track, without cars
    test_track = model.Track(test_rails, None)

    # Define test cars
    test_cars = [
        Car(model.Rail.Lane1, test_track),
        Car(model.Rail.Lane2, test_track)
    ]

    # Put test cars on test track
    test_track.cars = test_cars

    # 2nd track

    # Define rails of test track
    test_rails_2 = [
        model.TurnRail(2, 2 * np.pi, model.TurnRail.Right)
    ]

    # Define test track, without cars
    test_track_2 = model.Track(test_rails_2, None)

    # Define test cars
    test_cars_2 = [
        Car(model.Rail.Lane1, test_track_2),
        Car(model.Rail.Lane2, test_track_2)
    ]

    # Put test cars on test track
    test_track_2.cars = test_cars_2

    # 3nd track

    # Define rails of test track
    test_rails_3 = [
        model.TurnRail(2, 2 * np.pi, model.TurnRail.Left)
    ]

    # Define test track, without cars
    test_track_3 = model.Track(test_rails_3, None)

    # Define test cars
    test_cars_3 = [
        Car(model.Rail.Lane1, test_track_3),
        Car(model.Rail.Lane2, test_track_3)
    ]

    # Put test cars on test track
    test_track_3.cars = test_cars_3

    # ----------------------------------------------------------
    # Testing force calculations

    print("PRINTING PROPERTIES OF CAR 1")
    print("Mass:   ", test_track.cars[0].mass)
    print("Area:   ", test_track.cars[0].area)
    print("C_drag: ", test_track.cars[0].drag_coeff)
    print("C_mag:  ", test_track.cars[0].mag_coeff)
    print("eta:    ", test_track.cars[0].motor_eta)
    print("mu_tire:", test_track.cars[0].mu_tire)
    print("mu_pin: ", test_track.cars[0].mu_pin)
    print("mu_roll:", test_track.cars[0].mu_roll)
    print("")

    print("RESISTANCE TEST")
    print("Resistances should be roughly [39.1 39.1]")
    print(test_track.resistances)
    print("Okay!\n")

    print("MAGNET FORCE TEST")
    print("Magnet force should be roughly [0 0 -1]")
    print(test_track.cars[0].get_magnet_force())
    print("Okay!\n")

    print("GRAVITATIONAL FORCE TEST")
    print("Gravitational force should be roughly [0 0 -.7848]")
    print(test_track.cars[0].get_gravity_force())
    print("Okay!\n")

    print("NORMAL FORCE TEST")
    print("Normal force should be roughly [0 0 1.7848]")
    print(test_track.cars[0].get_normal_force())
    print("Okay!\n")

    print("ROLLING RESISTANCE TEST")
    print("Rolling resistance should be roughly [-0.017848 0 0]")
    print(test_track.cars[0].get_rolling_resistance())
    print("Okay!\n")

    print("LATERAL FRICTION TEST 1 -- STRAIGHT")
    print("Lateral friction force on a straight should be [0 0 0]")
    print(test_track.cars[0].get_lateral_friction())
    print("Okay!\n")

    print("LATERAL FRICTION TEST 2 -- RIGHT TURN")
    print("Lateral friction force on a straight should be [0 0 0]")
    print(test_track_2.cars[0].get_lateral_friction())
    print("Okay!\n")

    print("LATERAL FRICTION TEST 3 -- LEFT TURN")
    print("Lateral friction force on a straight should be [0 0 0]")
    print(test_track_3.cars[0].get_lateral_friction())
    print("Okay!\n")

    test_track_2.cars[0].vel_vec = np.asarray([10, 2, 3])
    test_track_3.cars[0].vel_vec = np.asarray([10, 2, 3])

    print("LATERAL FRICTION TEST 4 -- RIGHT TURN")
    print("Lateral friction force on a straight should be [0 -1.606 0]")
    print(test_track_2.cars[0].get_lateral_friction())
    print("Okay!\n")

    print("LATERAL FRICTION TEST 5 -- LEFT TURN")
    print("Lateral friction force on a straight should be [0 1.606 0]")
    print(test_track_3.cars[0].get_lateral_friction())
    print("Okay!\n")

    print("THRUST FORCE TEST")
    test_track.cars[0].controller_input = 0.25
    print("Thrust force should be roughly [0.39 0 0]")
    print(test_track.cars[0].get_thrust_force())
    print("Okay!\n")

    print("DRAG FORCE TEST")
    test_track.cars[0].vel_vec = np.asarray([0.2, -.5, .7])
    print("Drag force should be roughly [-0.00037 0 0]")
    print(test_track.cars[0].get_drag_force())
    print("Okay!\n")

    print("LATERAL PIN FORCE TEST 1 -- STRAIGHT")
    print("Lateral pin force should be [0 0 0]")
    print(test_track.cars[0].get_lateral_pin_force())
    print("Okay!\n")

    test_track_2.cars[0].vel_vec = np.asarray([1, 2, 3])
    test_track_3.cars[0].vel_vec = np.asarray([1, 2, 3])

    print("LATERAL PIN FORCE TEST 2 -- RIGHT TURN")
    print("Lateral pin force should be [0 0 0]")
    print(test_track_2.cars[0].get_lateral_pin_force())
    print("Okay!\n")

    print("LATERAL PIN FORCE TEST 3 -- LEFT TURN")
    print("Lateral pin force should be [0 0 0]")
    print(test_track_3.cars[0].get_lateral_pin_force())
    print("Okay!\n")

    test_track_2.cars[0].vel_vec = np.asarray([10, 2, 3])
    test_track_3.cars[0].vel_vec = np.asarray([10, 2, 3])

    print("LATERAL PIN FORCE TEST 4 -- RIGHT TURN")
    print("Lateral pin force should be roughly [0 -2.83 0]")
    print(test_track_2.cars[0].get_lateral_pin_force())
    print("Okay!\n")

    print("LATERAL PIN FORCE TEST 5 -- LEFT TURN")
    print("Lateral pin force should be roughly [0 3.00 0]")
    print(test_track_3.cars[0].get_lateral_pin_force())
    print("Okay!\n")

    print("PIN FRICTION FORCE TEST")
    print("Pin friction force should be roughly [-0.12 0 0]")
    print(test_track_3.cars[0].get_pin_friction())
    print("Okay\n")

    print("TOTAL FORCE TEST")
    print("Total force should be roughly [-0.19 4.6 0]")
    #print("Rolling resistance:", test_track_3.cars[0].get_rolling_resistance())
    #print("Pin friction:",       test_track_3.cars[0].get_pin_friction())
    #print("Lateral friction:",   test_track_3.cars[0].get_lateral_friction())
    #print("Magnet force:",       test_track_3.cars[0].get_magnet_force())
    #print("Gravity force:",      test_track_3.cars[0].get_gravity_force())
    #print("Normal force:",       test_track_3.cars[0].get_normal_force())
    #print("Thrust force:",       test_track_3.cars[0].get_thrust_force())
    #print("Drag force:",         test_track_3.cars[0].get_drag_force())
    #print("Lateral pin force:",  test_track_3.cars[0].get_lateral_pin_force())
    #print("Total force:",        test_track_3.cars[0].get_total_force())
    print("Okay\n")

    # ----------------------------------------------------------------------------
    # Testing state update functions

    test_track_3.cars[0].vel_vec = np.asarray([10, 0, 0])
    print(test_track_3.cars[0].pos_vec)
    for i in range(1001):
        test_track_3.cars[0].pos_vec, test_track_3.cars[0].vel_vec, angle_vec= test_track_3.cars[0].get_new_state(0.001)
        if (i%100 == 0):
            print("i=",i)
            print("New position: ", test_track_3.cars[0].pos_vec)
            print("New velocity: ", test_track_3.cars[0].vel_vec)