
import sys
sys.path.append('/home/jules/self_created_tools')
from mig_simulator import MIGSimulator, MockArduino, map_arduino, constrain

class Controller:
    def __init__(self, arduino):
        self.arduino = arduino
    def loop(self): pass

class MagicMIGController(Controller):
    def __init__(self, arduino):
        super().__init__(arduino)
        self.WIRE_BURN_PULSE_FUDGE_FACTOR = 0.8
        self.PULSE_DURATION = 0.01
        self.PULSE_CURRENT_RATIO = 5
        self.TARGET_WIRE_FEED_RATE = 100
        self.wire_burn_pulse_amount = 0.5 # initial estimate
        self.wire_burn_pulse_error = 0
        self.wire_feed_step_delay = 1000 # us
        self.state = 'NORMAL'

    def loop(self):
        v = map_arduino(self.arduino.analogRead(0), 0, 1023, 0, 40)
        i = map_arduino(self.arduino.analogRead(1), 0, 1023, 0, 200)

        # Check contact (simplified)
        if v < 5.0 and i > 5.0:
            # Retract
            self.arduino.digitalWrite(11, False)
            self.arduino.digitalWrite(10, True)
            self.arduino.tick(0.1)
            self.arduino.digitalWrite(10, False)

            # Pulse
            self.arduino.analogWrite(9, 200) # High current
            self.arduino.tick(self.PULSE_DURATION * 1000)

            # Low current sustain
            self.arduino.analogWrite(9, 50)

            # Feed back some
            self.arduino.digitalWrite(11, True)
            for _ in range(50): # Feed 0.25mm
                self.arduino.digitalWrite(10, True)
                self.arduino.tick(0.1)
                self.arduino.digitalWrite(10, False)
                self.arduino.tick(0.9)
        else:
            # Feed forward looking for contact
            self.arduino.digitalWrite(11, True)
            self.arduino.digitalWrite(10, True)
            self.arduino.tick(0.1)
            self.arduino.digitalWrite(10, False)
            self.arduino.tick(5.0) # Slow feed
            self.arduino.analogWrite(9, 60) # Small sensing current

class StateMachineController(Controller):
    def __init__(self, arduino):
        super().__init__(arduino)
        self.state = 'FEED'
        self.timer = 0

    def loop(self):
        v = map_arduino(self.arduino.analogRead(0), 0, 1023, 0, 40)
        i = map_arduino(self.arduino.analogRead(1), 0, 1023, 0, 200)

        if self.state == 'FEED':
            self.arduino.analogWrite(9, 60)
            self.arduino.digitalWrite(11, True)
            self.arduino.digitalWrite(10, True)
            self.arduino.tick(0.1)
            self.arduino.digitalWrite(10, False)
            self.arduino.tick(1.0)
            if v < 2.0:
                self.state = 'PULSE'
                self.timer = self.arduino.millis
        elif self.state == 'PULSE':
            self.arduino.analogWrite(9, 250)
            self.arduino.digitalWrite(11, False)
            self.arduino.digitalWrite(10, True)
            self.arduino.tick(0.1)
            self.arduino.digitalWrite(10, False)
            if self.arduino.millis - self.timer > 10:
                self.state = 'SUSTAIN'
                self.timer = self.arduino.millis
        elif self.state == 'SUSTAIN':
            self.arduino.analogWrite(9, 80)
            self.arduino.digitalWrite(11, True)
            if self.arduino.millis - self.timer > 50:
                self.state = 'FEED'

def run_test(ControllerClass, filename):
    sim = MIGSimulator()
    arduino = MockArduino(sim)
    controller = ControllerClass(arduino)
    print(f"Running test for {ControllerClass.__name__}...")
    for _ in range(500): # 500 loop iterations
        controller.loop()
    sim.plot(filename)
    print(f"Plot saved to {filename}")

if __name__ == "__main__":
    run_test(MagicMIGController, 'magic_controller.png')
    run_test(StateMachineController, 'statemachine_controller.png')
