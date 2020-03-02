import pyfirmata


class ExternalInput:
    def __init__(self, port, pin):
        self.board = pyfirmata.Arduino(port)
        it = pyfirmata.util.Iterator(self.board)
        it.start()
        self.analog_input = self.board.get_pin('a:' + pin + '0:i')
    
    def read(self):
        self.value = self.analog_input.read()




    