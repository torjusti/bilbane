import pyfirmata
import time

class ExternalInput:
    def __init__(self, port, pin):
        self.board = pyfirmata.Arduino(port)
        it = pyfirmata.util.Iterator(self.board)
        it.start()
        self.analog_input = self.board.get_pin('a:' + pin + '0:i')
        
    def read(self):
        self.value = self.analog_input.read()


class ControlTrack:
    def __init__(self, port, pwm_pin):
        self.scaling_factor = 1
        self.input = []
        self.board = pyfirmata.Arduino(port)
        it = pyfirmata.util.Iterator(self.board)
        it.start()
        self.pwm_pin = self.board.get_pin('d:' + pwm_pin + ':p') #definer pinnen som PWM(p), d(digital)
        #self.round_pin = self.board.get_pin('a:' + round_pin + '0:i')
        #self.photo_value = None
        #self.start_time_last_lap = None
        #self.lap_times = []
        #self.photo_max = None

    def read_input(self, file):
        f = open(file, 'r')
        for line in f:
            self.input.append(self.scaling_factor*float(line))
        f.close()
    '''
    def lap(self):
        new_photo_value = self.round_pin.read()
        new_lap = False
        #print(new_photo_value)
        if( self.photo_value - new_photo_value) > 0.02 and self.photo_value > self.photo_max: #fikse verdien her kommer ann på lysstyrke tid på dagen ect.
             new_lap = True
        self.photo_value = new_photo_value
        return new_lap


    def initilize_photoresistor(self):
        new_photo_value = self.round_pin.read()
        while new_photo_value == None or self.photo_value == None:
            self.photo_value = new_photo_value
            new_photo_value = self.round_pin.read()
        time.sleep(2)
        self.photo_value = new_photo_value
        self.photo_max = int(new_photo_value*100)/100
        
        
    
    def lap_time(self):
        finish_time_last_lap = time.time()
        self.lap_times.append(float(finish_time_last_lap-self.start_time_last_lap))
        self.start_time_last_lap = finish_time_last_lap
    
    def print_lap_times(self):
        for time in self.lap_times:
            print(time)
'''
    def run(self):
        i = 0
        for i in self.input:
            self.pwm_pin.write(i)
            time.sleep(1/60)
        self.pwm_pin.write(0)
    def test(self):
        self.pwm_pin.write(0.64)
        time.sleep(20)
        self.pwm_pin.write(0)
        time.sleep(5)
                

                    


def main():
    output = ControlTrack('com3','9')
    #output.test()
    output.read_input("final_test_3_laps.txt")
    output.run()
    
main()