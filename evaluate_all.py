
import sys
import os
import numpy as np
import json
sys.path.append(os.path.join(os.path.dirname(__file__), 'sim'))
from mig_simulator import MIGSimulator
from emulator_host import ArduinoEmulator

def evaluate_sketch(ino_path, sim_time=8.0):
    sim = MIGSimulator(dt=0.001)
    emulator = ArduinoEmulator(ino_path, sim)

    if not emulator.compile():
        print(f"Compilation failed for {ino_path}")
        return None

    try:
        print(f"Running simulation for {ino_path}...")
        emulator.run(max_sim_time=sim_time)

        # Calculate Metrics
        history = sim.history
        time = np.array(history['time'])
        arc_active = np.array(history['arc'])
        v_out = np.array(history['v_out'])
        i_out = np.array(history['i_out'])

        # Determine Target Voltage from pot settings in emulator (A2 = 512)
        # map(512, 0, 1023, 150, 300) / 10.0 = 22.5
        # If it's the old version it might be different, but we'll assume 22.5 for modern ones
        # or 18.0 as a generic target. Let's use 22.5 as it's the current MIG_improved target.
        target_v = 22.5

        start_idx = np.where(arc_active == 1.0)[0]
        if len(start_idx) > 0:
            first_arc_time = time[start_idx[0]]

            # Stable arc check
            stable_arc_time = None
            consecutive_count = 0
            needed = int(0.2 / sim.dt)
            for i in range(start_idx[0], len(arc_active)):
                if arc_active[i] == 1.0:
                    consecutive_count += 1
                else:
                    consecutive_count = 0
                if consecutive_count >= needed:
                    stable_arc_time = time[i - consecutive_count + 1]
                    break
        else:
            first_arc_time = sim_time
            stable_arc_time = sim_time

        if len(start_idx) > 0:
            # Only measure metrics in the last 50% of the simulation to allow for convergence
            mid_idx = len(time) // 2
            evaluation_mask = (arc_active == 1.0) & (np.arange(len(time)) > mid_idx)

            if not np.any(evaluation_mask): # Fallback to any active arc
                 evaluation_mask = arc_active == 1.0

            active_v = v_out[evaluation_mask]
            active_i = i_out[evaluation_mask]

            arc_stability = np.mean(arc_active[start_idx[0]:]) * 100

            if len(active_v) > 0:
                v_ripple = np.std(active_v)
                v_mean = np.mean(active_v)
                v_error = np.abs(v_mean - target_v)
            else:
                v_ripple = 99.9
                v_error = target_v

            if len(active_i) > 0:
                i_ripple = np.std(active_i)
                i_mean = np.mean(active_i)
            else:
                i_ripple = 999.9
                i_mean = 0.0

            if len(active_i) > 1:
                di = np.diff(active_i)
                spatter_index = np.mean(np.abs(di)) / sim.dt
            else:
                spatter_index = 999.9
        else:
            arc_stability = 0.0
            v_ripple = 99.9
            v_error = target_v
            i_ripple = 999.9
            spatter_index = 999.9
            first_arc_time = sim_time

        metrics = {
            'sketch': ino_path,
            'stability_pct': float(arc_stability),
            'v_ripple_v': float(v_ripple),
            'v_error_v': float(v_error),
            'i_ripple_a': float(i_ripple),
            'startup_time_s': float(first_arc_time),
            'spatter_index': float(spatter_index)
        }

        plot_name = ino_path.replace(".ino", "_eval.png")
        sim.plot(plot_name)

        return metrics
    except Exception as e:
        print(f"Error during evaluation: {e}")
        return None
    finally:
        emulator.cleanup()

if __name__ == "__main__":
    sketches = ["MIG_improved.ino"]
    if len(sys.argv) > 1:
        sketches = sys.argv[1:]

    for sketch in sketches:
        print(f"\n--- Evaluating {sketch} ---")
        res = evaluate_sketch(sketch)
        if res:
            print(f"Stability: {res['stability_pct']:.2f}% | V-Ripple: {res['v_ripple_v']:.2f}V | V-Error: {res['v_error_v']:.2f}V")
            print(f"Startup: {res['startup_time_s']:.2f}s | Spatter Index: {res['spatter_index']:.2f}")
        else:
            print(f"Failed to evaluate {sketch}")
