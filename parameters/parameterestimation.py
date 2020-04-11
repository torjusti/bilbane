import math
import numpy as np
import random



class ParameterEstimation:
    def __init__(self, track, filename):
        self.track = track
        ''' # 0:mag_coeff
         1: mu_pin 
        2: mu_roll  
        3: mu_axle  
        4: motor_coeff 
        5: max_power '''
        
        self.update_var = 1
        self.voltage_upper_limit = 0.65
        self.voltage_lower_limit = 0.4
        self.voltage_inc = 0.05
        self.upper_limits = [1.5, 0.05, 0.05, 0.5, 0.5, 1]
        self.lower_limits = [0.5, 0.01, 0.01, 0.05, 0.05, 0.1]
        self.update_inc = [0.1, 0.005, 0.005, 0.05, 0.05, 0.1]
        self.filename = filename
        
        self.car_pars = [0.5, 0.01, 0.01, 0.05, 0.05, 0.1]
        self.const_var = 0
       

    
    def set_up(self):
        for car in self.track.cars:
            car.controller_input = 0.4
            car.mag_coeff = self.car_pars[0]
            car.mu_pin = self.car_pars[1]
            car.mu_roll = self.car_pars[2]
            car.mu_axle = self.car_pars[3]
            car.motor_coeff = self.car_pars[4]
            car.max_power = self.car_pars[5]

    def update_voltage(self):
        for car in self.track.cars:
            prev_voltage = int(car.controller_input*100)/100 
            if int(car.controller_input*100)/100 == self.voltage_upper_limit:
                car.controller_input = self.voltage_lower_limit
            else:
                car.controller_input += self.voltage_inc
            return prev_voltage

    def write_to_file(self, ls):
        f = open(self.filename+".txt", "a+")
        for item in ls:
            f.write(str(item) +',')
        f.write(str(self.car_pars[0])+','+ str(self.car_pars[1])+','+ str(self.car_pars[2])
        +','+ str(self.car_pars[3])+
        ','+str(self.car_pars[4])+','+str(self.car_pars[5])+"\n")
        f.close()

    def update_parameters(self):
        if self.car_pars[self.update_var] >= self.upper_limits[self.update_var]:
            if (self.const_var == 5 and self.update_var == 4 ) or self.update_var == 5:
                self.car_pars[self.const_var] += self.update_inc[self.const_var]
                for i in range(len(self.car_pars)-1):
                    if i != self.const_var:
                        self.car_pars[i] = self.lower_limits[i]
                if self.car_pars[self.const_var] >= self.upper_limits[self.const_var]:
                    self.const_var +=1
                    if self.const_var == 6:
                        return False
                    self.car_pars[self.const_var] = self.lower_limits[self.const_var]
                        
                    
            self.car_pars[self.update_var] = self.lower_limits[self.update_var]
        self.car_pars[self.update_var] += self.update_inc[self.update_var]
        self.update_var += 1
        if self.update_var == self.const_var:
            self.update_var +=1
        if self.update_var == 7:
            self.update_var = 0
        if self.update_var == 6:
            if self.const_var != 0:
                self.update_var = 0
            else:
                self.update_var = 1
        
        
        
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
            if(time != -1):   
                self.write_to_file(time_voltage)
            
            for car in self.track.cars:
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
                    
                    previous_rail = car.rail
                if car.is_crashed:
                    
                    lap_time = -1
                    break
                previous_rail = car.rail    
            if laps == 1 and start_timer:
                start_timer = False
                start_time_lap = global_time
            if (laps == 2 and not start_timer):
                lap_time = global_time - start_time_lap
                break
            if global_time > 50:
                
                lap_time = -1
                break
        return lap_time
    
    def set_car_pars(self):
        for car in self.track.cars:
            car.mag_coeff = self.car_pars[0]
            car.mu_pin = self.car_pars[1]
            car.mu_roll = self.car_pars[2]
            car.mu_axle = self.car_pars[3]
            car.motor_coeff = self.car_pars[4]
            car.max_power = self.car_pars[5]