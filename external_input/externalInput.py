import pyfirmata
import time
input_list = [1.0,
1.0,
1.0,
1.0,
1.0,
1.0,
1.0,
1.0,
1.0,
1.0,
1.0,
1.0,
1.0,
1.0,
1.0,
1.0,
1.0,
1.0,
1.0,
0.8327631950378418,
1.0,
0.954216718673706,
0.7152539491653442,
0.40410691499710083,
0.14987421035766602,
0.0,
0.0,
0.0,
0.0,
0.0,
0.0,
0.0,
0.0,
0.0,
0.0,
0.0,
0.0,
0.4997319281101227,
0.4123758375644684,
0.2414570152759552,
0.11693926155567169,
0.04826507344841957,
0.06074143946170807,
0.06984943896532059,
0.06859058886766434,
0.10975507646799088,
0.14489440619945526,
0.09361784160137177,
0.06859414279460907,
1.0,
1.0,
1.0,
0.8977769613265991,
0.6185762882232666,
0.4889664947986603,
0.37680965662002563,
0.20841845870018005,
1.0,
0.9557011127471924,
0.8471686840057373,
0.7313284277915955,
0.7822330594062805,
0.907218337059021,
0.8237181305885315,
0.677240788936615,
0.385707825422287,
0.17696413397789001,
1.0,
1.0,
1.0,
1.0,
0.8933021426200867,
0.825316309928894,
0.7972418069839478,
0.9299527406692505,
1.0,
1.0,
1.0,
1.0,
1.0,
1.0,
1.0,
1.0,
1.0,
1.0]
input_list = [x * 0.85 for x in input_list]
class ExternalInput:
    def __init__(self, port, pin):
        self.board = pyfirmata.Arduino(port)
        it = pyfirmata.util.Iterator(self.board)
        it.start()
        self.analog_input = self.board.get_pin('a:' + pin + '0:i')
        
    def read(self):
        self.value = self.analog_input.read()

class ControlTrack:
    def __init__(self, port, pin):
        self.board = pyfirmata.Arduino(port)
        it = pyfirmata.util.Iterator(self.board)
        it.start()
        self.pwm_pin = self.board.get_pin('d:' + pin + ':p') #definer pinnen som PWM(p), d(digital)
    def test(self):
        puls_width = 0#max spenning 0.3*15.6v = 4.68V
        i = 0
        while True:
            self.pwm_pin.write(puls_width)#får noe mellom 0-1, 0 duty cycle 0 s, 1 duty cycle = TS T(periode)
            time.sleep(1/60) # må sleepe like lenge som tidssteget til AI?
            #puls_width += 0.01# "opplevd spenning" duty-cycle/T(periode)*maxV
            #finn max spenning, den kjører av i en sving: et sted mellom 0.7 og 0.8, eller litt lavere
            #finn min spenning for at den skal kjøre, 0.3 så kjører den så vidt
            puls_width = input_list[i]
            i+= 1

            #print(puls_width)
            if i == len(input_list):
                self.pwm_pin.write(0)
                break
                
                


def main():
    output = ControlTrack('com3','9')
    output.test()

main()