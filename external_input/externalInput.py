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
    def __init__(self, port, pin, scaling_factor):
        self.scaling_factor = scaling_factor
        self.input = []
        self.board = pyfirmata.Arduino(port)
        it = pyfirmata.util.Iterator(self.board)
        it.start()
        self.pwm_pin = self.board.get_pin('d:' + pin + ':p') #definer pinnen som PWM(p), d(digital)

    def read_input(self, file):
        f = open(file, 'r')
        for line in f:
            self.input.append(self.scaling_factor*float(line))
        f.close()

    def run(self):
        puls_width = 0
        i = 0
        while True:
            self.pwm_pin.write(puls_width)#får noe mellom 0-1, 0 duty cycle 0 s, 1 duty cycle = TS T(periode)
            time.sleep(1/60) # må sleepe like lenge som tidssteget til AI?
            #puls_width += 0.01# "opplevd spenning" duty-cycle/T(periode)*maxV
            #finn max spenning, den kjører av i en sving: et sted mellom 0.7 og 0.8, eller litt lavere
            #finn min spenning for at den skal kjøre, 0.3 så kjører den så vidt
            puls_width = self.input[i]
            i+= 1
            #print(puls_width)
            if i == len(self.input):
                self.pwm_pin.write(0)
                break
                
                


def main():
    output = ControlTrack('com3','9',0.9)
    output.read_input('input_sequences\\4_rounds.txt')
    output.run()

main()