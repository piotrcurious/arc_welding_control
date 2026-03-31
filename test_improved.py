
import sys
import os
# Relative path to simulation library
sys.path.append(os.path.join(os.path.dirname(__file__), 'sim'))
from mig_simulator import MIGSimulator, MockArduino, map_arduino, constrain

class Controller:
    def __init__(self, arduino):
        self.arduino = arduino
    def loop(self): pass

class ImprovedController(Controller):
    """
    Python model of the logic implemented in MIG_improved.ino.
    Used for verifying the control strategy before C++ deployment.
    """
    def __init__(self, arduino):
        super().__init__(arduino)
        self.state = 'FIND_CONTACT'
        self.timer = 0
        self.arc_count = 0

    def loop(self):
        v = map_arduino(self.arduino.analogRead(0), 0, 1023, 0, 102.3)
        i = map_arduino(self.arduino.analogRead(1), 0, 1023, 0, 204.6)

        if self.state == 'FIND_CONTACT':
            self.arduino.analogWrite(9, 50)
            self.arduino.digitalWrite(11, True)
            self.arduino.digitalWrite(10, True)
            self.arduino.tick(0.1)
            self.arduino.digitalWrite(10, False)
            self.arduino.tick(0.4)
            if v < 1.0:
                self.state = 'RETRACT_INIT'
                self.timer = self.arduino.millis
        elif self.state == 'RETRACT_INIT':
            self.arduino.digitalWrite(11, False)
            for _ in range(5):
                self.arduino.digitalWrite(10, True)
                self.arduino.tick(0.1)
                self.arduino.digitalWrite(10, False)
                self.arduino.tick(0.1)
            self.state = 'PULSE'
            self.timer = self.arduino.millis
        elif self.state == 'PULSE':
            self.arduino.analogWrite(9, 255)
            self.arduino.tick(10)
            self.state = 'SUSTAIN'
            self.timer = self.arduino.millis
        elif self.state == 'SUSTAIN':
            self.arduino.analogWrite(9, 120)
            self.arduino.digitalWrite(11, True)
            self.arduino.digitalWrite(10, True)
            self.arduino.tick(0.1)
            self.arduino.digitalWrite(10, False)
            self.arduino.tick(1.9)
            if self.arduino.millis - self.timer > 100:
                self.state = 'PULSE'
                self.timer = self.arduino.millis
                self.arc_count += 1
                if self.arc_count > 10:
                    self.state = 'FIND_CONTACT'
                    self.arc_count = 0

def run_test(ControllerClass, filename):
    sim = MIGSimulator(dt=0.0001)
    arduino = MockArduino(sim)
    controller = ControllerClass(arduino)
    print(f"Running test for {ControllerClass.__name__}...")
    for _ in range(3000):
        controller.loop()
        if sim.time > 3.0: break
    sim.plot(filename)
    print(f"Plot saved to {filename}")

if __name__ == "__main__":
    run_test(ImprovedController, 'improved_controller.png')
