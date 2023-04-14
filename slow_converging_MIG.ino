
// Arduino code for MIG arc welding controller

// Define pins for PWM output and step/dir signals
#define PWM_PIN 9
#define STEP_PIN 10
#define DIR_PIN 11

// Define analog inputs for target wire feed rate and wire burn pulse fudge factor
#define WIRE_FEED_RATE_PIN A0
#define WIRE_BURN_PULSE_FUDGE_PIN A1

// Define analog inputs for voltage and current feedback
#define VOLTAGE_FEEDBACK_PIN A2
#define CURRENT_FEEDBACK_PIN A3

// Define constants for PWM frequency and duty cycle range
#define PWM_FREQ 1000 // Hz
#define PWM_MIN 0 // %
#define PWM_MAX 100 // %

// Define constants for stepper motor steps per revolution and microstepping mode
#define STEPS_PER_REV 200
#define MICROSTEP_MODE 16

// Define constants for wire feed rate range and conversion factor
#define WIRE_FEED_RATE_MIN 0 // mm/s
#define WIRE_FEED_RATE_MAX 100 // mm/s
#define WIRE_FEED_RATE_FACTOR 0.01 // mm/step

// Define constants for wire burn pulse fudge factor range and conversion factor
#define WIRE_BURN_PULSE_FUDGE_MIN 0 // %
#define WIRE_BURN_PULSE_FUDGE_MAX 100 // %
#define WIRE_BURN_PULSE_FUDGE_FACTOR 0.01 // %

// Define constants for voltage and current feedback range and conversion factor
#define VOLTAGE_FEEDBACK_MIN 0 // V
#define VOLTAGE_FEEDBACK_MAX 50 // V
#define VOLTAGE_FEEDBACK_FACTOR 0.2 // V/unit
#define CURRENT_FEEDBACK_MIN 0 // A
#define CURRENT_FEEDBACK_MAX 200 // A
#define CURRENT_FEEDBACK_FACTOR 0.8 // A/unit

// Define constants for arc initiation current and pulse duration
#define ARC_INIT_CURRENT 10 // A
#define ARC_INIT_DURATION 10 // ms

// Define constants for arc pulse current and duration range and estimation factor
#define ARC_PULSE_CURRENT_MIN 20 // A
#define ARC_PULSE_CURRENT_MAX 200 // A
#define ARC_PULSE_DURATION_MIN 1 // ms
#define ARC_PULSE_DURATION_MAX 10 // ms
#define ARC_PULSE_ESTIMATION_FACTOR 0.1 // ms/A

// Define constants for arc sustain current and duration range and estimation factor
#define ARC_SUSTAIN_CURRENT_MIN 5 // A
#define ARC_SUSTAIN_CURRENT_MAX 50 // A
#define ARC_SUSTAIN_DURATION_MIN 10 // ms
#define ARC_SUSTAIN_DURATION_MAX 100 // ms
#define ARC_SUSTAIN_ESTIMATION_FACTOR 0.5 // ms/A

// Define constants for wire feed correction factor and tolerance range
#define WIRE_FEED_CORRECTION_FACTOR 0.5 // %
#define WIRE_FEED_TOLERANCE_MIN -5 // %
#define WIRE_FEED_TOLERANCE_MAX 5 // %

// Define constants for stepper motor acceleration and deceleration range and factor
#define STEPPER_ACCEL_MIN 10 // steps/s^2
#define STEPPER_ACCEL_MAX 1000 // steps/s^2
#define STEPPER_ACCEL_FACTOR 10 // steps/s^2/unit

// Define constants for stepper motor speed range and factor
#define STEPPER_SPEED_MIN 10 // steps/s
#define STEPPER_SPEED_MAX 1000 // steps/s
#define STEPPER_SPEED_FACTOR 10 // steps/s/unit

// Define variables for target wire feed rate and wire burn pulse fudge factor inputs
int wire_feed_rate_input;
int wire_burn_pulse_fudge_input;

// Define variables for voltage and current feedback inputs
int voltage_feedback_input;
int current_feedback_input;

// Define variables for PWM output and step/dir signals
int pwm_output;
int step_signal;
int dir_signal;

// Define variables for target wire feed rate and wire burn pulse fudge factor values
float wire_feed_rate;
float wire_burn_pulse_fudge;

// Define variables for voltage and current feedback values
float voltage_feedback;
float current_feedback;

// Define variables for arc initiation current and pulse duration values
float arc_init_current;
float arc_init_duration;

// Define variables for arc pulse current and duration values
float arc_pulse_current;
float arc_pulse_duration;

// Define variables for arc sustain current and duration values
float arc_sustain_current;
float arc_sustain_duration;

// Define variables for wire feed correction value and tolerance value
float wire_feed_correction;
float wire_feed_tolerance;

// Define variables for stepper motor acceleration and deceleration values
float stepper_accel;
float stepper_decel;

// Define variables for stepper motor speed and position values
float stepper_speed;
float stepper_position;

// Define variables for wire feed steps and error values
int wire_feed_steps;
int wire_feed_error;

// Define variables for arc voltage and wire burn estimation values
float arc_voltage;
float wire_burn_estimation;

// Define variables for state machine and timer values
int state;
unsigned long timer;

// Define states for state machine
#define STATE_INIT 0
#define STATE_FEED 1
#define STATE_PULSE 2
#define STATE_SUSTAIN 3
#define STATE_MEASURE 4
#define STATE_CORRECT 5

// Initialize the Arduino
void setup() {
  // Set PWM output and step/dir signals as outputs
  pinMode(PWM_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  // Set analog inputs as inputs
  pinMode(WIRE_FEED_RATE_PIN, INPUT);
  pinMode(WIRE_BURN_PULSE_FUDGE_PIN, INPUT);
  pinMode(VOLTAGE_FEEDBACK_PIN, INPUT);
  pinMode(CURRENT_FEEDBACK_PIN, INPUT);

  // Set PWM frequency and initial duty cycle
  analogWriteFrequency(PWM_PIN, PWM_FREQ);
  pwm_output = map(PWM_MIN, PWM_MIN, PWM_MAX, 0, 255);
  analogWrite(PWM_PIN, pwm_output);

  // Set initial step and dir signals
  step_signal = LOW;
  dir_signal = HIGH;
  digitalWrite(STEP_PIN, step_signal);
  digitalWrite(DIR_PIN, dir_signal);

  // Set initial state and timer
  state = STATE_INIT;
  timer = millis();
}

// Loop the Arduino
void loop() {
  // Read analog inputs and convert to values
  wire_feed_rate_input = analogRead(WIRE_FEED_RATE_PIN);
  wire_burn_pulse_fudge_input = analogRead(WIRE_BURN_PULSE_FUDGE_PIN);
  voltage_feedback_input = analogRead(VOLTAGE_FEEDBACK_PIN);
  current_feedback_input = analogRead(CURRENT_FEEDBACK_PIN);
 
  wire_feed_rate = map(wire_feed_rate_input, 0, 1023, WIRE_FEED_RATE_MIN, WIRE_FEED_RATE_MAX);
  wire_burn_pulse_fudge = map(wire_burn_pulse_fudge_input, 0, 1023, WIRE_BURN_PULSE_FUDGE_MIN, WIRE_BURN_PULSE_FUDGE_MAX) * WIRE_BURN_PULSE_FUDGE_FACTOR;
  voltage_feedback = map(voltage_feedback_input, 0, 1023, VOLTAGE_FEEDBACK_MIN, VOLTAGE_FEEDBACK_MAX) * VOLTAGE_FEEDBACK_FACTOR;
  current_feedback = map(current_feedback_input, 0, 1023, CURRENT_FEEDBACK_MIN, CURRENT_FEEDBACK_MAX) * CURRENT_FEEDBACK_FACTOR;

  // Switch between states based on state machine
  switch (state) {
    case STATE_INIT:
      // Initialize arc initiation current and pulse duration values
      arc_init_current = ARC_INIT_CURRENT;
      arc_init_duration = ARC_INIT_DURATION;

      // Initialize arc pulse current and duration values
      arc_pulse_current = ARC_PULSE_CURRENT_MIN;
      arc_pulse_duration = ARC_PULSE_DURATION_MIN + arc_pulse_current * ARC_PULSE_ESTIMATION_FACTOR;

      // Initialize arc sustain current and duration values
      arc_sustain_current = ARC_SUSTAIN_CURRENT_MIN;
      arc_sustain_duration = ARC_SUSTAIN_DURATION_MIN + arc_sustain_current * ARC_SUSTAIN_ESTIMATION_FACTOR;

      // Initialize wire feed correction value and tolerance value
      wire_feed_correction = 0;
      wire_feed_tolerance = 0;

      // Initialize stepper motor acceleration and deceleration values
      stepper_accel = map(wire_feed_rate_input, 0, 1023, STEPPER_ACCEL_MIN, STEPPER_ACCEL_MAX) * STEPPER_ACCEL_FACTOR;
      stepper_decel = stepper_accel;

      // Initialize stepper motor speed and position values
      stepper_speed = STEPPER_SPEED_MIN;
      stepper_position = 0;

      // Initialize wire feed steps and error values
      wire_feed_steps = 0;
      wire_feed_error = 0;

      // Initialize arc voltage and wire burn estimation values
      arc_voltage = 0;
      wire_burn_estimation = 0;

      // Transition to feed state
      state = STATE_FEED;
      break;

    case STATE_FEED:
      // Set PWM output to arc initiation current value
      pwm_output = map(arc_init_current, CURRENT_FEEDBACK_MIN, CURRENT_FEEDBACK_MAX, PWM_MIN, PWM_MAX);
      analogWrite(PWM_PIN, pwm_output);

      // Set dir signal to feed direction
      dir_signal = HIGH;
      digitalWrite(DIR_PIN, dir_signal);

      // Calculate wire feed steps based on wire feed rate value and wire feed correction value
      wire_feed_steps = (wire_feed_rate + wire_feed_correction) * WIRE_FEED_RATE_FACTOR * MICROSTEP_MODE;

      // Check if wire feed steps is positive
      if (wire_feed_steps > 0) {
        // Calculate stepper motor speed based on wire feed rate value and stepper speed factor
        stepper_speed = wire_feed_rate * STEPPER_SPEED_FACTOR * MICROSTEP_MODE;

        // Calculate stepper motor position based on wire feed steps and wire feed error
        stepper_position = stepper_position + wire_feed_steps - wire_feed_error;

        // Generate step signal based on stepper motor position and speed
        step_signal = (stepper_position % (STEPS_PER_REV * MICROSTEP_MODE)) < (stepper_speed / 2);
        digitalWrite(STEP_PIN, step_signal);

        // Update wire feed error based on step signal
        if (step_signal == HIGH) {
          wire_feed_error++;
        }
        else {
          wire_feed_error--;
        }
      }

      // Check if arc voltage feedback is zero
      if (voltage_feedback == 0) {
        // Transition to pulse state and reset timer
        state = STATE_PULSE;
        timer = millis();
      }
      break;

    case STATE_PULSE:
      // Set PWM output to arc pulse current value
      pwm_output = map(arc_pulse_current, CURRENT_FEEDBACK_MIN, CURRENT_FEEDBACK_MAX, PWM_MIN, PWM_MAX);
      analogWrite(PWM_PIN, pwm_output);

      // Set dir signal to retract direction
      dir_signal = LOW;
      digitalWrite(DIR_PIN, dir_signal);

      // Calculate stepper motor speed based on arc pulse current value and stepper speed factor
      stepper_speed = arc_pulse_current * STEPPER_SPEED_FACTOR * MICROSTEP_MODE;

      // Calculate stepper motor position based on arc pulse duration value and wire burn pulse fudge value
      stepper_position = stepper_position - arc_pulse_duration * wire_burn_pulse_fudge * MICROSTEP_MODE;

      // Generate step signal based on stepper motor position and speed
      step_signal = (stepper_position % (STEPS_PER_REV * MICROSTEP_MODE)) < (stepper_speed / 2);
      digitalWrite(STEP_PIN, step_signal);

      // Check if timer has reached arc pulse duration value
      if (millis() - timer >= arc_pulse_duration) {
        // Transition to sustain state and reset timer
        state = STATE_SUSTAIN;
        timer = millis();
      }
      break;


    case STATE_SUSTAIN:
      // Set PWM output to arc sustain current value
      pwm_output = map(arc_sustain_current, CURRENT_FEEDBACK_MIN, CURRENT_FEEDBACK_MAX, PWM_MIN, PWM_MAX);
      analogWrite(PWM_PIN, pwm_output);

      // Set dir signal to feed direction
      dir_signal = HIGH;
      digitalWrite(DIR_PIN, dir_signal);

      // Calculate stepper motor speed based on arc sustain current value and stepper speed factor
      stepper_speed = arc_sustain_current * STEPPER_SPEED_FACTOR * MICROSTEP_MODE;

      // Calculate stepper motor position based on arc sustain duration value and wire burn pulse fudge value
      stepper_position = stepper_position + arc_sustain_duration * wire_burn_pulse_fudge * MICROSTEP_MODE;

      // Generate step signal based on stepper motor position and speed
      step_signal = (stepper_position % (STEPS_PER_REV * MICROSTEP_MODE)) < (stepper_speed / 2);
      digitalWrite(STEP_PIN, step_signal);

      // Check if timer has reached arc sustain duration value
      if (millis() - timer >= arc_sustain_duration) {
        // Transition to measure state and reset timer
        state = STATE_MEASURE;
        timer = millis();
      }
      break;

    case STATE_MEASURE:
      // Set PWM output to zero
      pwm_output = 0;
      analogWrite(PWM_PIN, pwm_output);

      // Set step signal to low
      step_signal = LOW;
      digitalWrite(STEP_PIN, step_signal);

      // Read arc voltage feedback value
      arc_voltage = voltage_feedback;

      // Estimate wire burn amount based on arc voltage feedback value and wire burn estimation factor
      wire_burn_estimation = arc_voltage * WIRE_BURN_ESTIMATION_FACTOR;

      // Transition to correct state
      state = STATE_CORRECT;
      break;

    case STATE_CORRECT:
      // Calculate wire feed correction value based on wire burn estimation value and wire feed rate value
      wire_feed_correction = wire_burn_estimation - wire_feed_rate;

      // Calculate wire feed tolerance value based on wire feed correction value and wire feed rate value
      wire_feed_tolerance = wire_feed_correction / wire_feed_rate * 100;

      // Check if wire feed tolerance value is within tolerance range
      if (WIRE_FEED_TOLERANCE_MIN <= wire_feed_tolerance && wire_feed_tolerance <= WIRE_FEED_TOLERANCE_MAX) {
        // Adjust arc pulse current value based on wire feed tolerance value and arc pulse current range
        arc_pulse_current = constrain(arc_pulse_current + wire_feed_tolerance, ARC_PULSE_CURRENT_MIN, ARC_PULSE_CURRENT_MAX);

        // Adjust arc pulse duration value based on arc pulse current value and arc pulse duration range
        arc_pulse_duration = constrain(ARC_PULSE_DURATION_MIN + arc_pulse_current * ARC_PULSE_ESTIMATION_FACTOR, ARC_PULSE_DURATION_MIN, ARC_PULSE_DURATION_MAX);

        // Adjust arc sustain current value based on wire feed tolerance value and arc sustain current range
        arc_sustain_current = constrain(arc_sustain_current + wire_feed_tolerance, ARC_SUSTAIN_CURRENT_MIN, ARC_SUSTAIN_CURRENT_MAX);

        // Adjust arc sustain duration value based on arc sustain current value and arc sustain duration range
        arc_sustain_duration = constrain(ARC_SUSTAIN_DURATION_MIN + arc_sustain_current * ARC_SUSTAIN_ESTIMATION_FACTOR, ARC_SUSTAIN_DURATION_MIN, ARC_SUSTAIN_DURATION_MAX);
      }

      // Transition to feed state
      state = STATE_FEED;
      break;
  }
}
