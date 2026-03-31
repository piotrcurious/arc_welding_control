
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'sim'))
from mig_simulator import MIGSimulator
from emulator_host import ArduinoEmulator

def test_sketch(ino_path, output_png):
    sim = MIGSimulator(dt=0.001)
    emulator = ArduinoEmulator(ino_path, sim)

    # Custom pin mapping for certain sketches if needed
    if "magic" in ino_path:
        emulator.pin_map['VOLTAGE'] = 14 # A0
        emulator.pin_map['CURRENT'] = 15 # A1
        emulator.pin_map['PWM'] = 9
        emulator.pin_map['STEP'] = 10
        emulator.pin_map['DIR'] = 11

    if emulator.compile():
        try:
            emulator.run(max_sim_time=2.0)
            sim.plot(output_png)
            print(f"Simulation of {ino_path} finished. Plot saved to {output_png}")
        finally:
            emulator.cleanup()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        sketch = sys.argv[1]
        out = sketch.replace(".ino", ".png")
        test_sketch(sketch, out)
    else:
        # Default test
        test_sketch("MIG_improved.ino", "MIG_improved_emu.png")
