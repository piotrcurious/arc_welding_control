
// Define the pins for the PWM output, the step and dir signals for the stepper motor, and the analog inputs for the voltage and current feedback
#define PWM_PIN 9
#define STEP_PIN 10
#define DIR_PIN 11
#define VOLTAGE_PIN A0
#define CURRENT_PIN A1

// Define some constants for the welding parameters
#define TARGET_WIRE_FEED_RATE 100 // mm/s
#define WIRE_BURN_PULSE_FUDGE_FACTOR 0.8 // fraction of estimated wire burn amount to feed before each pulse
#define WIRE_DIAMETER 0.8 // mm
#define WIRE_DENSITY 7.8 // g/cm3
#define WIRE_MELTING_POINT 1538 // degrees C
#define WIRE_HEAT_CAPACITY 0.46 // J/gK
#define WIRE_RESISTIVITY 1.43e-6 // ohm-m
#define ARC_LENGTH 2 // mm
#define ARC_VOLTAGE_DROP 20 // V
#define ARC_EFFICIENCY 0.8 // fraction of arc power transferred to wire
#define MIN_CURRENT 10 // A
#define MAX_CURRENT 200 // A
#define MIN_VOLTAGE 10 // V
#define MAX_VOLTAGE 40 // V
#define MIN_PWM_DUTY_CYCLE 10 // %
#define MAX_PWM_DUTY_CYCLE 90 // %
#define PULSE_DURATION 0.01 // s
#define PULSE_CURRENT_RATIO 5 // ratio of pulse current to base current
#define STEPPER_STEPS_PER_REVOLUTION 200 // steps/rev
#define STEPPER_MICROSTEPS 16 // microsteps/step
#define STEPPER_GEAR_RATIO 10 // ratio of motor shaft to wire feed roller rotation
#define STEPPER_WIRE_FEED_ROLLER_DIAMETER 10 // mm

// Define some variables for the welding state and control logic
float wire_feed_rate; // mm/s
float wire_feed_step_delay; // us
float wire_feed_error; // mm
float wire_burn_amount; // mm
float wire_burn_pulse_amount; // mm
float wire_burn_pulse_error; // mm
float voltage; // V
float current; // A
float pwm_duty_cycle; // %
bool wire_contact; // true if wire is in contact with workpiece, false otherwise

// Define some helper functions for the welding logic

// Map a value from one range to another range linearly
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Clamp a value between a minimum and a maximum value
float clamp_float(float x, float min_val, float max_val) {
  return min(max(x, min_val), max_val);
}

// Convert a voltage reading from the analog input to a real voltage value in volts
float read_voltage() {
  return map_float(analogRead(VOLTAGE_PIN), 0, 1023, 0, MAX_VOLTAGE);
}

// Convert a current reading from the analog input to a real current value in amps
float read_current() {
  return map_float(analogRead(CURRENT_PIN), 0, 1023, 0, MAX_CURRENT);
}

// Set the PWM duty cycle for the buck converter output in percentage (0-100)
void set_pwm_duty_cycle(float duty_cycle) {
  duty_cycle = clamp_float(duty_cycle, MIN_PWM_DUTY_CYCLE, MAX_PWM_DUTY_CYCLE);
  analogWrite(PWM_PIN, map_float(duty_cycle, 0, 100, 0, 255));
}

// Set the direction of the stepper motor rotation (true for forward, false for backward)
void set_stepper_direction(bool direction) {
  digitalWrite(DIR_PIN, direction);
}

// Make one step of the stepper motor rotation (positive or negative depending on direction)
void make_stepper_step() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(STEP_PIN, LOW);
}

// Estimate the amount of wire burned by one pulse of current based on the arc parameters and wire properties
float estimate_wire_burn_amount(float current) {
  
  float arc_power = current * ARC_VOLTAGE_DROP * ARC_EFFICIENCY; // W
  float wire_mass = PI * sq(WIRE_DIAMETER / 2) * WIRE_DENSITY / 1000; // g/mm
  float wire_heat = wire_mass * (WIRE_MELTING_POINT - 20) * WIRE_HEAT_CAPACITY; // J/mm
  float wire_resistance = WIRE_RESISTIVITY / (PI * sq(WIRE_DIAMETER / 2)); // ohm/mm
  float wire_power = sq(current) * wire_resistance; // W/mm
  float wire_burn_amount = arc_power * PULSE_DURATION / (wire_heat + wire_power); // mm
  return wire_burn_amount;
}

// Check if the wire is in contact with the workpiece by measuring the voltage drop across the wire
bool check_wire_contact() {
  float wire_voltage_drop = current * WIRE_RESISTIVITY / (PI * sq(WIRE_DIAMETER / 2)); // V/mm
  float contact_threshold = wire_voltage_drop * ARC_LENGTH + ARC_VOLTAGE_DROP; // V
  return voltage < contact_threshold;
}


// Initialize the welding state and control logic
void setup() {
  
  // Set the PWM output pin as output and set its frequency to 31.25 kHz
  pinMode(PWM_PIN, OUTPUT);
  TCCR1B = TCCR1B & B11111000 | B00000001;
  
  // Set the step and dir pins as output and initialize them to low
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  
  // Set the initial values for the welding state and control logic variables
  wire_feed_rate = TARGET_WIRE_FEED_RATE;
  wire_feed_step_delay = 1000000 / (wire_feed_rate * STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS * STEPPER_GEAR_RATIO / (PI * STEPPER_WIRE_FEED_ROLLER_DIAMETER));
  wire_feed_error = 0;
  wire_burn_amount = estimate_wire_burn_amount(MAX_CURRENT);
  wire_burn_pulse_amount = wire_burn_amount * WIRE_BURN_PULSE_FUDGE_FACTOR;
  wire_burn_pulse_error = 0;
  voltage = read_voltage();
  current = read_current();
  pwm_duty_cycle = map_float(voltage, MIN_VOLTAGE, MAX_VOLTAGE, MIN_PWM_DUTY_CYCLE, MAX_PWM_DUTY_CYCLE);
  set_pwm_duty_cycle(pwm_duty_cycle);
  wire_contact = check_wire_contact();
}

// Update the welding state and control logic
void loop() {
  
  // Read the voltage and current feedback from the analog inputs
  voltage = read_voltage();
  current = read_current();
  
  // Check if the wire is in contact with the workpiece
  wire_contact = check_wire_contact();
  
  // If the wire is in contact, retract it slightly and increase the current to initiate a pulse
  if (wire_contact) {
    set_stepper_direction(false); // backward
    make_stepper_step(); // one step back
    pwm_duty_cycle = map_float(current * PULSE_CURRENT_RATIO, MIN_CURRENT, MAX_CURRENT, MIN_PWM_DUTY_CYCLE, MAX_PWM_DUTY_CYCLE); // increase current
    set_pwm_duty_cycle(pwm_duty_cycle); // set PWM duty cycle
    delay(PULSE_DURATION * 1000); // wait for pulse duration
    pwm_duty_cycle = map_float(current / PULSE_CURRENT_RATIO, MIN_CURRENT, MAX_CURRENT, MIN_PWM_DUTY_CYCLE, MAX_PWM_DUTY_CYCLE); // decrease current
    set_pwm_duty_cycle(pwm_duty_cycle); // set PWM duty cycle
    
    // Estimate the amount of wire burned by the pulse and update the error
    wire_burn_amount = estimate_wire_burn_amount(current * PULSE_CURRENT_RATIO);
    wire_burn_pulse_error = wire_burn_pulse_amount - wire_burn_amount;
    
    // Feed the wire by half of the estimated wire burn amount plus the error correction term
    set_stepper_direction(true); // forward
    float wire_feed_steps = (wire_burn_amount / 2 + wire_burn_pulse_error) * STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS * STEPPER_GEAR_RATIO / (PI * STEPPER_WIRE_FEED_ROLLER_DIAMETER); // steps to feed
    for (int i = 0; i < wire_feed_steps; i++) {
      make_stepper_step(); // one step forward
      delayMicroseconds(wire_feed_step_delay); // wait for step delay
    }
  }
  
  // If the wire is not in contact, feed it by the estimated wire burn amount before each pulse
  else {
    set_stepper_direction(true); // forward
    float wire_feed_steps = wire_burn_pulse_amount * STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS * STEPPER_GEAR_RATIO / (PI * STEPPER_WIRE_FEED_ROLLER_DIAMETER); // steps to feed
    for (int i = 0; i < wire_feed_steps; i++) {
      make_stepper_step(); // one step forward
      delayMicroseconds(wire_feed_step_delay); // wait for step delay
    }
  }
}
