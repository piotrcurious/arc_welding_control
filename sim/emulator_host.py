
import subprocess
import os
import sys
import time
import re
from sim.mig_simulator import MIGSimulator

class ArduinoEmulator:
    def __init__(self, ino_path, sim):
        self.ino_path = ino_path
        self.sim = sim
        self.process = None
        self.binary = "emulator.bin"
        self.pin_map = {
            'PWM': 9,
            'STEP': 10,
            'DIR': 11,
            'VOLTAGE': 14, # A0
            'CURRENT': 15, # A1
            'FEED_RATE': 16, # A2
            'PULSE_FACTOR': 17 # A3
        }
        self.pwm_val = 0
        self.step_val = False
        self.dir_val = True

    def compile(self):
        cpp_file = "temp_emulator.cpp"
        with open(self.ino_path, "r") as ino:
            content = ino.read()

        funcs = re.findall(r'\n(\w+)\s+(\w+)\s*\([^)]*\)\s*\{', content)
        forward_decls = "".join([f"{ret} {name}(); " for ret, name in funcs if name not in ['setup', 'loop']])

        with open(cpp_file, "w") as f:
            f.write('#include "arduino_mock/include/Arduino.h"\n')
            f.write('void setup(); void loop(); ' + forward_decls + '\n')
            f.write(content)

        cmd = [
            "g++", "-Iarduino_mock/include",
            "arduino_mock/src/Arduino.cpp",
            cpp_file, "-o", self.binary
        ]
        print(f"Compiling {self.ino_path}...")
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode != 0:
            print("Compilation failed!")
            print(res.stderr)
            return False
        return True

    def run(self, max_sim_time=5.0):
        self.process = subprocess.Popen(
            ["./" + self.binary],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=sys.stderr,
            text=True,
            bufsize=0
        )

        print(f"Running emulator for {max_sim_time}s simulation time...")

        start_time = time.time()
        while self.sim.time < max_sim_time:
            if time.time() - start_time > 30:
                print("Simulation timeout!")
                break

            line = self.process.stdout.readline()
            if not line:
                if self.process.poll() is not None:
                    break
                continue

            line = line.strip()
            if line.startswith("LOG:"):
                # print(line[4:])
                continue

            if line.startswith("DW:"):
                parts = line.split(":")
                if len(parts) == 3:
                    pin, val = int(parts[1]), int(parts[2])
                    if pin == self.pin_map['STEP']: self.step_val = bool(val)
                    if pin == self.pin_map['DIR']: self.dir_val = bool(val)

            elif line.startswith("AW:"):
                parts = line.split(":")
                if len(parts) == 3:
                    pin, val = int(parts[1]), int(parts[2])
                    if pin == self.pin_map['PWM']: self.pwm_val = val

            elif line.startswith("AR:"):
                parts = line.split(":")
                if len(parts) == 2:
                    pin = int(parts[1])
                    val = self.get_analog_val(pin)
                    self.process.stdin.write(f"{val}\n")
                    self.process.stdin.flush()

            elif line.startswith("MT"):
                self.process.stdin.write(f"{int(self.sim.time * 1000000)}\n")
                self.process.stdin.flush()

            elif line.startswith("DL:"):
                parts = line.split(":")
                if len(parts) == 2:
                    amt = float(parts[1])
                    self.tick_sim(amt / 1000.0)
            elif line.startswith("DM:"):
                parts = line.split(":")
                if len(parts) == 2:
                    amt = float(parts[1])
                    self.tick_sim(amt / 1000000.0)

            elif line == "LS":
                self.tick_sim(self.sim.dt)

    def get_analog_val(self, pin):
        v_fb = self.sim.v_cap
        if self.sim.gap <= 0:
            i_fb = self.sim.v_cap / 0.02
        elif self.sim.arc_active:
            i_fb = self.sim.i_ind
        else:
            i_fb = 0

        if pin == 14: return int(min(1023, v_fb * 10))
        if pin == 15: return int(min(1023, i_fb * 5))
        if pin == 16: return 512
        if pin == 17: return 512
        return 0

    def tick_sim(self, duration):
        steps = int(duration / self.sim.dt)
        if steps == 0 and duration > 0: steps = 1
        for _ in range(max(1, steps)):
            self.sim.step(self.pwm_val, self.step_val, self.dir_val)
        self.step_val = False

    def cleanup(self):
        if self.process:
            self.process.terminate()
