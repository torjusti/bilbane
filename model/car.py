import numpy as np
import model

class Car:
    # Run-off criterion for point mass car
    MAX_SPEED = 500
    MAX_LATERAL_FORCE = 1
    MAX_VOLTAGE = 16

    # Boolean to check if car is treated as point mass
    is_point_mass = True

    # Boolean to check if car has crashed
    is_chrashed = True

    # Position of the car in the global coordinate system.
    pos_vec = np.zeros(3)
    vel_vec = np.zeros(3)
    acc_vec = np.zeros(3)

    mass       = 0.08 #kg
    area       = 0.002268 #m^2
    drag_coeff = 0.35
    mag_coeff  = 1 #N
    motor_eta  = None

    # Track section on which the car is situated.
    rail = None
    # How far along the car is on the current rail.
    rail_progress = 0

    lane = None
    track = None

    controller_input = None # Value in interval [0,1]

    def __init__(self, lane, speed, track,
                 mass, area, drag_coeff, mag_coeff, motor_eta):
        self.lane = lane
        self.speed = speed
        self.track = track
        #TODO: Initialize all variables

    def get_new_state(self, delta_time):
        new_pos_vec   = None
        new_angle_vec = None
        if (not self.is_chrashed):
            new_pos_vec   = get_new_pos()
            new_angle_vec = get_new_angles()
        return new_pos_vec, new_angle_vec

    def get_new_pos():
        return new_pos_vec

    def get_new_angles():
        if is_point_mass:
            angle_vec = get_new_point_angles()
        else:
            angle_vec = get_new_body_angles()
        return angle_vec

    def get_new_point_angles(pos_vec,Rail current_rail):
            #TODO: Fix stuff between -------------
        if current_rail.isturn: #needs a member function to check if rail is a turning rail
            #--------------------------------------------------------------------------------------------------------
            pos_vec_COR=current_rail.centre #needs a member function to find centre of rail
            lane_radius=current_rail.radius #needs a member function radius of rail to lane
            left_turn=current_rail.isleftturn #needs a boolean value to know whether or not it is in a left turn
            #------------------------------------------------------------------------------------------------------
            pos_vec_rel=pos_vec-pos_vec_COR #vector from centre of rail to car
            pos_vec_up=np.array([0,lane_radius]) #vector from centre of rail in positive y(local) coord
            dot_product=np.dot(pos_vec_rel,pos_vec_up) #dotproduct between the two
            relative_angle=np.arccos(dot_product/(lane_radius*lane_radius)) #angle between Y(global) and y(local) axis
            if pos_vec_rel[0]<0: #if relative position vector points to the right
                yaw=relative_angle #angle between X(global) and x(local) axis
            else:
                yaw=360-relative_angle #angle between X(global) and x(local) axis
            if left_turn:
                yaw=yaw+180 #left turn offsets the calculations above with 180 degrees because of flipped x(local) axis
        else:
            yaw=current_rail.angle #needs angle of rail #TODO: Also this

        new_angle_vec = np.asarray([0, 0, yaw])
        return new_angle_vec

    def get_new_body_angles():
        return angle_vec

    # Calculate forces

    def get_total_force():
        total_force_vec = get_rolling_resistance() + 
                          get_pin_friction() +
                          get_lateral_friction() +
                          get_magnet_force() +
                          get_gravity_force() +
                          get_normal_force() +
                          get_thrust_force() +
                          get_drag_force() +
                          get_lateral_pin_force()
        # Crash check
        if (total_force_vec[1] >= MAX_LATERAL_FORCE):
            is_chrashed = True
        return total_force_vec

    def get_rolling_resistance():
        # TODO: Make valid for skidding car
        n_vec = get_normal_force(self)
        N = np.linalg.norm(n_vec)
        f1_vec = np.asarray([-C*N, 0, 0])
        return f1_vec

    def get_pin_friction():
        l_vec = get_lateral_pin_force(self)
        L = np.linalg.norm(l_vec)
        f2_vec = np.asarray([-mu_pin*L, 0, 0])
        return f2_vec

    def get_lateral_friction():
        n_vec = get_normal_force(self)
        N = np.linalg.norm(n_vec)
        f3_vec = np.asarray([0, -mu_tire*N, 0])
        return f3_vec

    def get_magnet_force():
        # TODO: Make valid for skidding car (skid => magnet not above rail)
        m_vec = [0, 0, -M]
        return m_vec

    def get_gravity_force():
        # This function is always valid
        g_vec = np.asarray([0, 0, -self.mass*G_ACC])
        return g_vec

    def get_normal_force(self):
        # TODO: Extend to 3D
        # In 3D, we do not necessarily have 0 net force in z-dir,
        # which the current implementation assumes.
        n_vec = - (get_magnet_force(self) + get_gravity_force(self))
        return n_vec

    def get_thrust_force():
        U = controller_input * MAX_VOLTAGE

        if (self.lane == Rail.Lane1):
            R = track
        elif (self.lane == Rail.Lane2):
            R = track
        else:
            raise ValueError("Invalid lane value")
        T = eta*(U**2)/
        t_vec = np.asarray([0, 0, T])
        return t_vec

    def get_drag_force():
        # TODO: Make valid for skidding car (skid => area and drag coefficient change)
        RHO = 1.21 # density of air
        d_vec = .5*RHO*A*drag_coeff*np.dot(self.vel_vec, self.vel_vec)
        return d_vec

    def get_lateral_pin_force():
        l_vec = np.zeros(3)
        if isinstance(self.rail, TurnRail):
            cent_f = np.dot(self.vel_vec, self.vel_vec)*self.mass/self.rail.radius
            # TODO: Check if right or left turn
            l_vec = -np.asarray([0, -cent_f, 0]) - get_lateral_friction()
        return l_vec

    def get_car_track_angle():
        # TODO: Implement for car that is not a point mass
        return 0