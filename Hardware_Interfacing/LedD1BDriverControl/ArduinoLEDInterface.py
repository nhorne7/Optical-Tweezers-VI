import pyfirmata2

class ArduinoLEDController:
    def __init__(self, pin_number):
        print("ArduinoLEDController Object created")
        self.pin_number = pin_number
        self.board = None
        self.pwm_out = None

    def initialize(self):
        self.PORT = pyfirmata2.Arduino.AUTODETECT
        self.board = pyfirmata2.Arduino(self.PORT)
        if self.pin_number in [3,5,6,9,10,11]:
            self.pwm_out = self.board.get_pin(f'd:{self.pin_number}:p')
        else:
            print(f'Pin number {self.pin_number} is not capable of PWM output on the Arduino Nano.')
            raise ValueError
        self.board.digital[13].write(True) # Turn onboard led on to show that the arduino is working properly.

    def setLEDDutyCycle(self, duty_cycle):
        if self.pwm_out:
            self.pwm_out.write(duty_cycle)
        if duty_cycle <= 1 and duty_cycle > 0:
            self.board.digital[13].write(1) # For debugging
        elif duty_cycle == 0:
            self.board.digital[13].write(0)
            
    def release(self):
        if self.pwm_out:
            self.pwm_out.write(0.0)
        self.board.digital[13].write(False) # For debugging without actual led :)
        self.board.exit()


