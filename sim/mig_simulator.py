
import math
import matplotlib.pyplot as plt

class MIGSimulator:
    def __init__(self, dt=0.0001):  # 0.1ms time step
        self.dt = dt
        # Physics Constants
        self.V_bus = 50.0
        self.L = 0.0001
        self.C = 0.001
        self.R_internal = 0.05
        self.R_short = 0.02
        self.V_ion = 15.0
        self.E_field = 0.5 # V/mm
        self.k_melt = 0.3  # mm / (A * s)

        # State Variables
        self.v_cap = 0.0
        self.i_ind = 0.0
        self.gap = 5.0    # mm (initial gap)
        self.arc_active = False

        # History
        self.history = {'time': [], 'v_out': [], 'i_out': [], 'gap': [], 'arc': [], 'duty': [], 'steps': []}
        self.time = 0.0

    def step(self, duty, step_pin, dir_pin):
        # Move wire
        if step_pin:
            move = 0.005 # 5um per step
            if dir_pin: self.gap -= move
            else: self.gap += move

        # Clip gap
        if self.gap < 0: self.gap = 0

        # Buck Electricals
        V_pwm = self.V_bus * (duty / 255.0)

        if self.gap <= 0:
            R_load = self.R_short
            self.arc_active = False
        elif self.arc_active or (self.gap < 0.2 and self.v_cap > 25): # Breakdown at 25V/0.2mm
            self.arc_active = True
            V_arc_ideal = self.V_ion + self.gap * self.E_field
            # V_cap = I * R_load -> R_load = V_arc_ideal / I
            R_load = V_arc_ideal / max(0.1, self.i_ind) + 0.1
        else:
            self.arc_active = False
            R_load = 1e6

        # di/dt = (V_pwm - v_cap) / L
        self.i_ind += (V_pwm - self.v_cap) / self.L * self.dt
        if self.i_ind < 0: self.i_ind = 0

        # dv/dt = (i_ind - v_cap/R_load) / C
        self.v_cap += (self.i_ind - self.v_cap / R_load) / self.C * self.dt
        if self.v_cap < 0: self.v_cap = 0

        # Melting
        if self.arc_active:
            self.gap += self.k_melt * self.i_ind * self.dt

        # Extinction
        if self.gap > 8.0 or self.i_ind < 1.0:
            self.arc_active = False

        self.time += self.dt
        self.history['time'].append(self.time)
        self.history['v_out'].append(self.v_cap)
        self.history['i_out'].append(self.v_cap/R_load if R_load < 1e5 else 0)
        self.history['gap'].append(self.gap)
        self.history['arc'].append(1.0 if self.arc_active else 0.0)
        self.history['duty'].append(duty)
        self.history['steps'].append(1.0 if step_pin else 0.0)

        return self.v_cap, self.history['i_out'][-1]

    def plot(self, filename):
        fig, axs = plt.subplots(5, 1, figsize=(10, 15), sharex=True)
        axs[0].plot(self.history['time'], self.history['v_out'])
        axs[0].set_ylabel('Voltage (V)')
        axs[1].plot(self.history['time'], self.history['i_out'])
        axs[1].set_ylabel('Current (A)')
        axs[2].plot(self.history['time'], self.history['gap'])
        axs[2].set_ylabel('Gap (mm)')
        axs[3].plot(self.history['time'], self.history['duty'])
        axs[3].set_ylabel('Duty Cycle')
        axs[4].plot(self.history['time'], self.history['arc'])
        axs[4].set_ylabel('Arc Active')
        axs[4].set_xlabel('Time (s)')
        plt.tight_layout()
        plt.savefig(filename)
        plt.close()

class MockArduino:
    def __init__(self, simulator):
        self.sim = simulator
        self.pins = {i: False for i in range(20)}
        self.analog_pins = {i: 512 for i in range(8)}
        self.pwm_pins = {i: 0 for i in range(20)}
        self.millis = 0
        self.step_pin = 10
        self.dir_pin = 11
        self.pwm_pin = 9

    def digitalWrite(self, pin, value):
        self.pins[pin] = bool(value)

    def digitalRead(self, pin):
        return self.pins[pin]

    def analogWrite(self, pin, value):
        self.pwm_pins[pin] = value

    def analogRead(self, pin):
        v_fb = self.sim.v_cap
        i_fb = self.sim.history['i_out'][-1] if self.sim.history['i_out'] else 0
        if pin == 0: return int(constrain(v_fb * 10, 0, 1023))
        if pin == 1: return int(constrain(i_fb * 5, 0, 1023))
        return self.analog_pins.get(pin, 0)

    def tick(self, dt_ms):
        for _ in range(int(dt_ms * 0.001 / self.sim.dt)):
            self.sim.step(self.pwm_pins[self.pwm_pin], self.pins[self.step_pin], self.pins[self.dir_pin])
        self.millis += dt_ms

def constrain(x, a, b):
    return max(a, min(x, b))

def map_arduino(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
