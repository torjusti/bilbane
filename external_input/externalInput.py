import pyfirmata


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
        self.pwm_pin = self.board.get_pin('d:' + pin + ':p')
    def test(self):
        while True:
           self.pwm_pin.write(0.4)


def main():
    output = ControlTrack('com3','9')
    output.test()

main()