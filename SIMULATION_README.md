
# MIG Arc Welding Simulation Testing Framework

This repository contains a Python-based simulation environment for testing Arduino MIG welder controllers.

## Components

- `sim/mig_simulator.py`: The core physics engine. It simulates:
  - DC-DC Buck Converter (Electrical)
  - Arc Physics (Breakdown, Ionization, Voltage/Length relationship)
  - Wire Melting (Heat/Mass balance)
  - Stepper Motor (Mechanical wire feed)
- `test_improved.py`: A Python verification script that models the logic in `MIG_improved.ino`.

## How to Test

Run the simulation using:
```bash
python3 test_improved.py
```
This will generate `improved_controller.png` showing the performance of the controller.

## Controller Logic (MIG_improved.ino)

The improved controller uses a contact-triggered state machine:
1. **SENSE_CONTACT**: Feeds wire forward with low sensing current.
2. **RETRACT**: Retracts wire slightly upon contact.
3. **PULSE**: Initiates a high-power pulse to start the arc.
4. **SUSTAIN**: Maintains the arc with constant power/current while feeding wire forward.
5. **CYCLE**: Automatically repeats pulses to maintain a stable welding process.
