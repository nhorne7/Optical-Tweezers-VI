import ArduinoLEDInterface
import time


board = ArduinoLEDInterface.ArduinoLEDController(3)

board.initialize()

try:
    while True:
        board.setLEDDutyCycle(1)
        time.sleep(1)
        board.setLEDDutyCycle(0)
        time.sleep(1)
except KeyboardInterrupt:
    board.release()