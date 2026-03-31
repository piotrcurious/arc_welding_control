
import math
import matplotlib.pyplot as plt
import random

class MIGSimulator:
    def __init__(self, dt=0.0001):
        self.dt = dt
        # Physics Constants
        self.V_bus = 50.0
        self.L = 0.0001    # 100 uH
        self.C = 0.01      # 10 mF
        self.R_internal = 0.02
        self.R_short = 0.01
        self.V_ion = 14.0
        self.E_field = 0.5 # V/mm
        self.k_melt = 0.3  # mm / (A * s)

        # State Variables
        self.v_cap = 0.0
        self.i_ind = 0.0
        self.gap = 5.0
        self.arc_active = False
        self.hand_pos = 0.0

        # History
        self.history = {'time': [], 'v_out': [], 'i_out': [], 'gap': [], 'arc': [], 'duty': [], 'steps': []}
        self.time = 0.0

    def step(self, duty, step_pin, dir_pin):
        # Move wire
        if step_pin:
            move = 0.05
            if dir_pin: self.gap -= move
            else: self.gap += move

        # Limit gap
        self.gap = max(0.0, min(20.0, self.gap))

        # Sub-stepping for stability
        sub_steps = 10
        sub_dt = self.dt / sub_steps

        for _ in range(sub_steps):
            # Hand Jitter
            self.hand_pos += (math.sin(self.time * 2) * 0.002 + (random.random() - 0.5) * 0.001) * (sub_dt / self.dt)
            current_gap = self.gap + self.hand_pos
            if current_gap < 0: current_gap = 0

            # Load resistance and Arc Physics
            if current_gap <= 0.001: # Direct contact
                R_load = self.R_short
                self.arc_active = False
            elif self.arc_active or (current_gap < 0.8 and self.v_cap > 20.0):
                self.arc_active = True
                V_arc_ideal = self.V_ion + current_gap * self.E_field
                # Resistance of the arc depends on current
                R_load = V_arc_ideal / max(1.0, self.i_ind) + 0.05
            else:
                self.arc_active = False
                R_load = 500.0 # Open circuit leakage

            # Buck Electricals
            V_pwm = self.V_bus * (duty / 255.0)

            # Inductor Current
            di = (V_pwm - self.v_cap - self.i_ind * self.R_internal) / self.L * sub_dt
            self.i_ind += di
            if self.i_ind < 0: self.i_ind = 0

            # Capacitor Voltage
            dv = (self.i_ind - self.v_cap / R_load) / self.C * sub_dt
            self.v_cap += dv
            if self.v_cap < 0: self.v_cap = 0

            # Melting
            if self.arc_active:
                self.gap += self.k_melt * (self.v_cap / R_load) * sub_dt

            # Extinction conditions
            if current_gap > 10.0 or (self.arc_active and self.v_cap < self.V_ion * 0.8):
                self.arc_active = False

        self.history['time'].append(self.time)

        # Analog values for controller
        v_out = self.v_cap
        i_out = self.i_ind

        self.history['v_out'].append(v_out)
        self.history['i_out'].append(i_out)
        self.history['gap'].append(self.gap + self.hand_pos)
        self.history['arc'].append(1.0 if self.arc_active else 0.0)
        self.history['duty'].append(duty)
        self.history['steps'].append(1.0 if step_pin else 0.0)

        self.time += self.dt
        return v_out, i_out

    def plot(self, filename):
        if not self.history['time']: return
        fig, axs = plt.subplots(5, 1, figsize=(10, 15), sharex=True)
        axs[0].plot(self.history['time'], self.history['v_out'])
        axs[0].set_ylabel('Voltage (V)')
        axs[0].grid(True)
        axs[1].plot(self.history['time'], self.history['i_out'])
        axs[1].set_ylabel('Current (A)')
        axs[1].grid(True)
        axs[2].plot(self.history['time'], self.history['gap'])
        axs[2].set_ylabel('Gap (mm)')
        axs[2].grid(True)
        axs[3].plot(self.history['time'], self.history['duty'])
        axs[3].set_ylabel('Duty Cycle')
        axs[3].grid(True)
        axs[4].plot(self.history['time'], self.history['arc'])
        axs[4].set_ylabel('Arc Active')
        axs[4].set_xlabel('Time (s)')
        axs[4].grid(True)
        plt.tight_layout()
        plt.savefig(filename)
        plt.close()
