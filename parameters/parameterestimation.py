import math
import numpy as np
import random

class DataGathering:
    def __init__(self, track, filename):
        self.track = track
        ''' 
            0: mu_roll  
            1: mu_gears  
            2: max_power
            3: mu_prop_v 
            '''
        self.voltage_upper_limit = 0.65
        self.voltage_lower_limit = 0.4
        self.voltage_inc = 0.05

        self.upper_limits = [1, 1, 5, 1]
        self.lower_limits = [ 0.05, 0.05, 1, 0.05 ]
        self.update_inc =  [ 0.095, 0.095, 0.4 , 0.095]#10 steps på hver
        self.filename = filename
        
        self.car_pars = [ 0.05, 0.05, 1, 0.05 ]
        self.const_var = 0
        self.update_var = 1

    
    def set_up(self):
        for car in self.track.cars:
            car.controller_input = 0.4
            car.mu_roll = self.car_pars[0]
            car.mu_gears = self.car_pars[1]
            car.max_power = self.car_pars[2]
            car.mu_prop_v = self.car_pars[3]
            

    def update_voltage(self):
        for car in self.track.cars:
            prev_voltage = int(car.controller_input*100)/100 
            if int(car.controller_input*100)/100 == self.voltage_upper_limit:
                car.controller_input = self.voltage_lower_limit
            else:
                car.controller_input += self.voltage_inc
            return prev_voltage

    def write_to_file(self, ls):
        f = open(self.filename, "a+")
        for item in ls:
            f.write(str(item) +',')
        f.write(str(self.car_pars[0])+','+ str(self.car_pars[1])+','+ str(self.car_pars[2])+ ','+ str(self.car_pars[3])+"\n")
        f.close()

    def update_parameters(self):
        if self.car_pars[self.update_var] >= self.upper_limits[self.update_var]:
            if (self.const_var == len(self.car_pars)-1 and self.update_var == len(self.car_pars)-2 )\
                 or self.update_var == len(self.car_pars)-1:
                self.car_pars[self.const_var] += self.update_inc[self.const_var]
                for i in range(len(self.car_pars)):
                    if i != self.const_var:
                        self.car_pars[i] = self.lower_limits[i]
                if self.car_pars[self.const_var] >= self.upper_limits[self.const_var]:
                    self.const_var +=1
                    if self.const_var == len(self.car_pars):
                        return False
                    self.car_pars[self.const_var] = self.lower_limits[self.const_var]    
            self.car_pars[self.update_var] = self.lower_limits[self.update_var]
        self.car_pars[self.update_var] += self.update_inc[self.update_var]
       
        if(self.update_var + 1 != self.const_var and self.update_var + 1 != len(self.car_pars)):
            self.update_var += 1
        else:
            if (self.update_var + 1 == len(self.car_pars) and self.const_var != 0) or \
                (self.update_var + 1 == self.const_var and self.const_var == len(self.car_pars)-1) :
                self.update_var = 0
            elif self.update_var + 1 == len(self.car_pars) and self.const_var == 0:
                self.update_var = 1
            elif self.update_var + 1 == self.const_var and self.const_var != len(self.car_pars)-1:
                self.update_var += 2
        return True

    def run(self):
        self.set_up()
        while True:
            time_voltage = []
            for _ in range(6):
                time = self.run_simulation()
                voltage = self.update_voltage()
                time_voltage.append(time)
                time_voltage.append(voltage)
                print("Time: ", time)
            for i in range(len(self.car_pars)):
                print(i,self.car_pars[i])
            if(time != -1 and time < 2 and time_voltage[0]<3):   
                self.write_to_file(time_voltage)
            for car in self.track.cars:
                #print("vel: ", np.linalg.norm(car.vel_vec))
                car.reset()
                car.controller_input = self.voltage_lower_limit
            if not self.update_parameters():
                break
            self.set_car_pars()

    def run_simulation(self):
        global_time = 0
        laps = 0
        delta_time = 1/100 
        start_timer = True
        start_time_lap = 0
        previous_rail = None
        while True:
            global_time += delta_time
            self.track.step(delta_time)
            for car in self.track.cars:
                if previous_rail == self.track.rails[-1] and car.rail == self.track.rails[0]:
                    laps+=1
                if car.is_crashed:
                    lap_time = -1
                    break
                previous_rail = car.rail    
            if laps == 1 and start_timer:
                start_timer = False
                start_time_lap = global_time
            if (laps == 2 and not start_timer):
                lap_time = global_time - start_time_lap
                ''' print("Lap time", lap_time, " controller input: ", car.controller_input)
                print("vel: ", np.linalg.norm(car.vel_vec))
                print("Rolling resitance:", car.get_rolling_resistance(car.pos_vec, car.vel_vec), "\n",
                 
                   "Axle friction    :", car.get_axle_friction(car.pos_vec, car.vel_vec), "\n",
                  
                   "Magnet force     :", car.get_magnet_force(car.pos_vec, car.vel_vec), "\n",
                   "Gravity force    :", car.get_gravity_force(car.pos_vec, car.vel_vec), "\n",
                   "Normal force     :", car.get_normal_force(car.pos_vec, car.vel_vec), "\n",
                   "Thrust force     :", car.get_thrust_force(car.pos_vec, car.vel_vec, car.controller_input), "\n",
                   "Drag force       :", car.get_drag_force(car.pos_vec, car.vel_vec), "\n",
                  
                   "Total force:", car.get_total_force( car.pos_vec, car.vel_vec, car.phi, car.controller_input), "\n")
                '''
                break
            if global_time > 50:
                lap_time = -1
                break
        return lap_time
    
    def set_car_pars(self):
        for car in self.track.cars:
            car.mu_roll = self.car_pars[0]
            car.mu_gears = self.car_pars[1]
            car.max_power = self.car_pars[2]
            car.mu_prop_v = self.car_pars[3]
            

class ParameterEstimation:
    def __init__(self, filename):
        self.filename = filename
        self.measured_lap_times = [2.14, 1.75, 1.55, 1.35, 1.16, 1.07]
        self.sim_lap_times = []#float
        self.corresponding_parameters = []#str
        self.best_fit = None
        self.best_parameters = None
        self.deviation = np.inf
    
    def extract_lap_times(self):
        f = open(self.filename, "r")
        for line in f:
            lap_times = line.split(",")
            ls = []
            j = 0
            for i in range(0, 12, 2):
                if((float(lap_times[i]) - 0.1 ) <= self.measured_lap_times[j] <= (float(lap_times[i]) + 0.1 )):
                    ls.append(float(lap_times[i]))
                    if i == 10:
                        self.sim_lap_times.append(ls)
                        self.corresponding_parameters.append(lap_times[12:len(lap_times)])
                else:
                    break
                j += 1

    def sum_of_squares(self): 
        i = 0
        for times in self.sim_lap_times:
            sum = 0
            for j in range(len(times)): 
                sum += pow(times[j]-self.measured_lap_times[j],2)
            if sum < self.deviation:
                self.deviation = sum
                self.best_fit = times
                self.best_parameters = self.corresponding_parameters[i]
                print("Best parameters: ", self.best_parameters)
                print("Deviation: ", self.deviation)
                print("Best fit lap times: ", self.best_fit)
            i += 1
    def run(self):
        self.extract_lap_times()
        self.sum_of_squares()
        print("Best match")
        print("Best parameters: ", self.best_parameters)
        print("Deviation: ", self.deviation)
        print("Best fit lap times: ", self.best_fit)