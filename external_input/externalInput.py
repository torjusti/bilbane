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
    def __init__(self, port, pwm_pin, round_pin, scaling_factor):
        self.scaling_factor = scaling_factor
        self.input = []
        self.board = pyfirmata.Arduino(port)
        it = pyfirmata.util.Iterator(self.board)
        it.start()
        self.pwm_pin = self.board.get_pin('d:' + pwm_pin + ':p') #definer pinnen som PWM(p), d(digital)
        self.round_pin = self.board.get_pin('a:' + round_pin + '0:i')
        self.photo_value = None
        self.start_time_last_lap = None
        self.lap_times = []
        self.photo_max = None

    def read_input(self, file):
        f = open(file, 'r')
        for line in f:
            self.input.append(self.scaling_factor*float(line))
        f.close()
    
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

    def run(self):
        puls_width = 0.5
        i = 0
        laps = 0
        self.start_time_last_lap = time.time()
        stop = True
        while True:
            self.pwm_pin.write(puls_width)
            if(self.lap()):
                self.lap_time()
                laps +=1
            if (laps+1)% 5==0 and stop:
                self.pwm_pin.write(0)
                time.sleep(2)
                stop = False
            
            if laps == 10:
                self.pwm_pin.write(0)
                break
            #puls_width = self.input[i]
            #i+= 1
            
                

                    


def main():
    output = ControlTrack('com3','9','0',0.9)
    output.initilize_photoresistor()
    #output.read_input('input_sequences\\4_rounds.txt')
    output.run()
    output.print_lap_times()
    #output.read_photoresistor()

main()