import numpy as np
import model

G_ACC = 9.81

class Car:
    # Run-off criterion for point mass car
    MAX_LATERAL_FORCE = 1

    # Maximum voltage from track to car
    MAX_VOLTAGE = 16

    # Boolean to check if car is treated as point mass
    is_point_mass = None

    # Boolean to check if car has crashed
    is_chrashed = None

    # Position of the car in the global coordinate system.
    pos_vec = np.zeros(3)
    vel_vec = np.zeros(3)
    acc_vec = np.zeros(3)

    mass       = None       #kg
    area       = None   #m^2
    drag_coeff = None
    mag_coeff  = None          #N
    motor_eta  = None
    mu_tire    = None
    mu_pin     = None
    mu_roll    = None

    # Track section on which the car is situated.
    rail = None # TODO: When is this gonna be updated?
    # How far along the car is on the current rail.
    rail_progress = 0 # TODO: What is this here for?

    # Lane and track the car is on
    lane = None
    track = None

    # Input from the controller, will be converted to a voltage which drives the car
    controller_input = None # Value in interval [0,1]

    def __init__(self, lane, speed, track):
        self.lane  = lane
        self.speed = speed
        self.track = track
        self.controller_input = 0

        is_chrashed   = False
        is_point_mass = True

        mass       = 0.08       #kg
        area       = 0.002268   #m^2
        drag_coeff = 0.35
        mag_coeff  = 1          #N
        motor_eta  = 1
        mu_tire    = 1
        mu_pin     = 1
        mu_roll    = 1
    
    def get_new_state(self, delta_time):
        """
        Purpose: Get new state of the car
        Args:
            delta_time -- time step in seconds
        Returns: 
            new_pos_vec -- ndarray containing car position
            new_angle_vec -- ndarray containing roll, pitch and yaw of car relative to global coordinate system
        """
        new_pos_vec   = None
        new_angle_vec = None

        if (not self.is_chrashed):
            new_pos_vec   = self.get_new_pos(delta_time)
            new_angle_vec = self.get_new_angles(new_pos_vec, self.rail)

        self.set_new_pos(new_pos_vec)
        return new_pos_vec, new_angle_vec

    def get_new_pos(self, delta_time):
        new_acc_vec = self.get_total_force() / self.mass
        new_vel_vec = vel_vec + new_acc_vec*delta_time
        new_pos_vec = new_vel_vec*delta_time + .5*new_acc_vec*delta_time

        return new_pos_vec

    def set_new_pos(self, new_pos_vec):
        self.pos_vec = new_pos_vec

    def get_new_angles(self, new_pos_vec, new_rail):
        if is_point_mass:
            angle_vec = self.get_new_point_angles(new_pos_vec, new_rail)
        else:
            angle_vec = self.get_new_body_angles(new_pos_vec, new_rail)

        return angle_vec

    def get_new_point_angles(self, new_pos_vec, new_rail):
        # Assumptions: 2D

        # If on a turn
        if isinstance(new_rail, TurnRail):
            pos_vec_COR = new_rail.get_rail_center
            lane_radius = new_rail.radius # TODO: Fix this
            left_turn   = (new_rail.direction == TurnRail.Left)
            
            pos_vec_rel    = new_pos_vec - pos_vec_COR # Vector from centre of rail to car
            pos_vec_up     = np.asarray([0, lane_radius]) # Vector from centre of rail in positive y(local) coord
            dot_product    = np.dot(pos_vec_rel, pos_vec_up) # Dotproduct between the two
            relative_angle = np.arccos(dot_product/(lane_radius*lane_radius)) # Angle between Y(global) and y(local) axis
            
            if pos_vec_rel[0] < 0: # If relative position vector points to the right
                yaw = relative_angle # Angle between X(global) and x(local) axis
            else:
                yaw = 360 - relative_angle # Angle between X(global) and x(local) axis
            
            if left_turn:
                yaw = yaw + 180 # Left turn offsets the calculations above with 180 degrees because of flipped x(local) axis
        
        # Else we are on a straight
        else:
            yaw = new_rail.global_angle

        new_angle_vec = np.asarray([0, 0, yaw])
        return new_angle_vec

    def get_new_body_angles():
        return angle_vec

    # ---------------------------------------------------------------------------
    # Calculate forces

    def get_total_force(self):
        total_force_vec = self.get_rolling_resistance()
                          + self.get_pin_friction()
                          + self.get_lateral_friction()
                          + self.get_magnet_force()
                          + self.get_gravity_force()
                          + self.get_normal_force()
                          + self.get_thrust_force()
                          + self.get_drag_force()
                          + self.get_lateral_pin_force()
        # Crash check
        if (total_force_vec[1] >= self.MAX_LATERAL_FORCE):
            self.is_chrashed = True
        return total_force_vec

    def get_rolling_resistance(self):
        # TODO: Make valid for skidding car
        n_vec  = get_normal_force(self)
        N      = np.linalg.norm(n_vec)
        f1_vec = np.asarray([-self.mu_roll*N, 0, 0])
        return f1_vec

    def get_pin_friction(self):
        l_vec  = get_lateral_pin_force(self)
        L      = np.linalg.norm(l_vec)
        f2_vec = np.asarray([-self.mu_pin*L, 0, 0])
        return f2_vec

    def get_lateral_friction(self):
        n_vec  = self.get_normal_force()
        N      = np.linalg.norm(n_vec)
        f3_vec = np.asarray([0, -self.mu_tire*N, 0])
        return f3_vec

    def get_magnet_force(self):
        # TODO: Make valid for skidding car (skid => magnet not above rail)
        m_vec = [0, 0, -self.mag_coeff]
        return m_vec

    def get_gravity_force(self):
        # This function is always valid
        g_vec = np.asarray([0, 0, -self.mass*G_ACC])
        return g_vec

    def get_normal_force(self):
        # TODO: Extend to 3D
        # In 3D, we do not necessarily have 0 net force in z-dir,
        # which the current implementation assumes.
        n_vec = - (self.get_magnet_force() + self.get_gravity_force())
        return n_vec

    def get_thrust_force(self):
        U = self.controller_input * self.MAX_VOLTAGE

        if (self.lane == Rail.Lane1):
            R = self.track.resistances[0]
        elif (self.lane == Rail.Lane2):
            R = self.track.resistances[1]
        else:
            raise ValueError("Invalid lane value")
        T = self.motor_eta*(U**2)/R
        t_vec = np.asarray([0, 0, T])
        return t_vec

    def get_drag_force(self):
        # TODO: Make valid for skidding car (skid => area and drag coefficient change)
        RHO   = 1.21 # density of air
        d_vec = .5*RHO*self.area*self.drag_coeff*np.dot(self.vel_vec, self.vel_vec)
        return d_vec

    def get_lateral_pin_force(self):
        l_vec = np.zeros(3)
        if isinstance(self.rail, TurnRail):
            cent_f = np.dot(self.vel_vec, self.vel_vec)*self.mass/self.rail.radius
            # TODO: Check if right or left turn
            l_vec = -np.asarray([0, -cent_f, 0]) - get_lateral_friction()
        return l_vec

    def get_car_track_angle(self):
        # TODO: Implement for car that is not a point mass
        return 0


if __name__ == "__main__":
    # TODO: Do some testing here