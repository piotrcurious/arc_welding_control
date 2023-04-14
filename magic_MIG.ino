
// Arduino MIG arc welding controller with improved reliability
// This code is for demonstration purposes only and has not been tested or verified
// Use at your own risk

// Define pins for analog inputs, PWM output, stepper motor and relays
#define VOLTAGE_PIN A0 // voltage feedback from welding circuit
#define CURRENT_PIN A1 // current feedback from welding circuit
#define WIRE_FEED_PIN A2 // potentiometer for wire feed rate
#define BURN_PULSE_PIN A3 // potentiometer for wire burn pulse fudge factor
#define PWM_PIN 3 // PWM output for dc-dc buck converter
#define STEP_PIN 4 // step signal for stepper motor driver
#define DIR_PIN 5 // direction signal for stepper motor driver
#define GAS_RELAY_PIN 6 // relay for gas valve
#define POWER_RELAY_PIN 7 // relay for power contactor

// Define constants and variables
const float VOLTAGE_SCALE = 0.01; // scale factor for voltage feedback (V/V)
const float CURRENT_SCALE = 0.01; // scale factor for current feedback (A/V)
const int PWM_FREQ = 20000; // PWM frequency for dc-dc buck converter (Hz)
const int PWM_RES = 8; // PWM resolution for dc-dc buck converter (bits)
const int STEP_FREQ = 1000; // step frequency for stepper motor driver (Hz)
const int STEP_RES = 200; // step resolution for stepper motor driver (steps/rev)
const float WIRE_DIAMETER = 0.8; // wire diameter (mm)
const float WIRE_DENSITY = 7.8; // wire density (g/cm3)
const float PI = 3.14159; // pi constant

float voltage; // measured voltage (V)
float current; // measured current (A)
float wire_feed_rate; // target wire feed rate (m/min)
float burn_pulse_factor; // wire burn pulse fudge factor (0-1)
float burn_pulse_duration; // wire burn pulse duration (ms)
float burn_pulse_current; // wire burn pulse current (A)
float burn_pulse_voltage; // wire burn pulse voltage (V)
float wire_burn_amount; // estimated wire burn amount per pulse (mm)
float wire_feed_amount; // corrected wire feed amount per cycle (mm)
float arc_voltage; // measured arc voltage during pulse (V)
float arc_length; // estimated arc length during pulse (mm)

bool gas_on; // gas valve status
bool power_on; // power contactor status
bool wire_feed_on; // wire feed status
bool burn_pulse_on; // burn pulse status

unsigned long gas_time; // gas valve timer (ms)
unsigned long power_time; // power contactor timer (ms)
unsigned long wire_feed_time; // wire feed timer (ms)
unsigned long burn_pulse_time; // burn pulse timer (ms)

// Added variables for improved reliability
float wire_feed_velocity; // target wire feed velocity (mm/s)
float wire_feed_acceleration; // target wire feed acceleration (mm/s2)
float step_interval; // step interval for stepper motor driver (ms)
float step_error; // step error for stepper motor driver (ms)
int step_count; // step count for stepper motor driver
int step_dir; // step direction for stepper motor driver (+1 or -1)

float burn_pulse_frequency; // target burn pulse frequency (Hz)
float burn_pulse_error; // burn pulse error based on arc voltage feedback (%)

// Initialize the system
void setup() {
  pinMode(VOLTAGE_PIN, INPUT); 
  pinMode(CURRENT_PIN, INPUT); 
  pinMode(WIRE_FEED_PIN, INPUT); 
  pinMode(BURN_PULSE_PIN, INPUT); 
  pinMode(PWM_PIN, OUTPUT); 
  pinMode(STEP_PIN, OUTPUT); 
  pinMode(DIR_PIN, OUTPUT); 
  pinMode(GAS_RELAY_PIN, OUTPUT); 
  pinMode(POWER_RELAY_PIN, OUTPUT); 

  analogWriteResolution(PWM_RES); 
  analogWriteFrequency(PWM_PIN, PWM_FREQ); 

  gas_on = false;
  power_on = false;
  wire_feed_on = false;
  burn_pulse_on = false;

}

// Main loop
void loop() {
  
  read_inputs(); // read analog inputs and calculate parameters

  if (!gas_on) { // if gas is off
    if (digitalRead(GAS_RELAY_PIN) == HIGH) { // if gas button is pressed
      digitalWrite(GAS_RELAY_PIN, LOW); // turn on gas valve
      gas_on = true;
      gas_time = millis(); // start gas timer
    }
  }
  
  if (!power_on) { // if power is off
    if ((millis() - gas_time) > 500) { // if gas pre-flow time is over
      digitalWrite(POWER_RELAY_PIN, HIGH); // turn on power contactor
      power_on = true;
      power_time = millis(); // start power timer
    }
  }

  if (!wire_feed_on) { // if wire feed is off
    if ((millis() - power_time) > 100) { // if power on-delay time is over
      digitalWrite(DIR_PIN, HIGH); // set wire feed direction to forward
      wire_feed_on = true;
      wire_feed_time = millis(); // start wire feed timer
      step_count = 0; // reset step count
      step_dir = 1; // set step direction to positive
      step_interval = 1000 / STEP_FREQ; // calculate initial step interval based on step frequency
      step_error = 0; // reset step error
    }
  }

  if (!burn_pulse_on) { // if burn pulse is on
    if (current > 10) { // if arc is initiated
      digitalWrite(DIR_PIN, LOW); // set wire feed direction to reverse
      burn_pulse_on = true;
      burn_pulse_time = millis(); // start burn pulse timer
    }
  }

  control_pwm(); // control PWM output for dc-dc buck converter
  control_stepper(); // control step and dir signals for stepper motor driver

  check_timers(); // check timers and update statuses

}

// Read analog inputs and calculate parameters
void read_inputs() {
  voltage = analogRead(VOLTAGE_PIN) * VOLTAGE_SCALE; // read and scale voltage feedback
  current = analogRead(CURRENT_PIN) * CURRENT_SCALE; // read and scale current feedback
  wire_feed_rate = map(analogRead(WIRE_FEED_PIN), 0, 1023, 1, 20); // read and map wire feed rate potentiometer
  burn_pulse_factor = map(analogRead(BURN_PULSE_PIN), 0, 1023, 0.1, 1.0); // read and map burn pulse factor potentiometer

  burn_pulse_duration = wire_feed_rate * burn_pulse_factor; // calculate burn pulse duration based on wire feed rate and factor
  burn_pulse_current = current * 5; // calculate burn pulse current based on current feedback and multiplier
  burn_pulse_voltage = voltage * 2; // calculate burn pulse voltage based on voltage feedback and multiplier

  wire_burn_amount = PI * WIRE_DIAMETER * WIRE_DIAMETER * WIRE_DENSITY * burn_pulse_duration / (4 * burn_pulse_current * burn_pulse_voltage); // estimate wire burn amount per pulse based on wire diameter, density, pulse duration, current and voltage

  wire_feed_velocity = wire_feed_rate * (1000 / (60 * PI * WIRE_DIAMETER)); // calculate target wire feed velocity based on wire feed rate and diameter (mm/s)
  wire_feed_acceleration = wire_feed_velocity / 10; // calculate target wire feed acceleration based on wire feed velocity and constant (mm/s2)

  burn_pulse_frequency = 1 / burn_pulse_duration; // calculate target burn pulse frequency based on burn pulse duration (Hz)
  burn_pulse_error = (arc_voltage - burn_pulse_voltage) / burn_pulse_voltage; // calculate burn pulse error based on arc voltage and burn pulse voltage feedback (%)
}

// Control PWM output for dc-dc buck converter
void control_pwm() {
  if (burn_pulse_on) { // if burn pulse is on
    analogWrite(PWM_PIN, map(burn_pulse_voltage, 0, 50, 0, 255)); // set PWM duty cycle based on burn pulse voltage and PWM resolution
  }
  else { // if burn pulse is off
    analogWrite(PWM_PIN, map(voltage, 0, 50, 0, 255)); // set PWM duty cycle based on voltage feedback and PWM resolution
  }
}

// Control step and dir signals for stepper motor driver
void control_stepper() {
  if (wire_feed_on) { // if wire feed is on
    if ((millis() - wire_feed_time) > (step_interval + step_error)) { // if step interval plus error is over
      digitalWrite(STEP_PIN, !digitalRead(STEP_PIN)); // toggle step signal
      wire_feed_time = millis(); // update wire feed timer
      step_count += step_dir; // update step count based on step direction
      step_error = (wire_feed_amount - (step_count * PI * WIRE_DIAMETER / STEP_RES)) * 1000 / wire_feed_velocity; // calculate step error based on wire feed amount, step count and velocity (ms)
      step_interval = 1000 / (wire_feed_velocity + wire_feed_acceleration * step_count); // calculate step interval based on wire feed velocity, acceleration and step count (ms)
    }
  }
}

// Check timers and update statuses
void check_timers() {
  if (burn_pulse_on) { // if burn pulse is on
    if ((millis() - burn_pulse_time) > burn_pulse_duration) { // if burn pulse duration is over
      burn_pulse_on = false; // turn off burn pulse
      arc_voltage = voltage; // measure arc voltage during pulse
      arc_length = arc_voltage / 20; // estimate arc length based on arc voltage and constant
      wire_feed_amount = wire_burn_amount * (1 + burn_pulse_error); // calculate corrected wire feed amount based on wire burn amount and burn pulse error
    }
  }

  if (wire_feed_on) { // if wire feed is on
    if (abs(step_error) < 0.1) { // if step error is negligible
      wire_feed_on = false; // turn off wire feed
    }
  }

  if (power_on) { // if power is on
    if (digitalRead(POWER_RELAY_PIN) == LOW) { // if power button is pressed
      digitalWrite(POWER_RELAY_PIN, LOW); // turn off power contactor
      power_on = false;
    }
  }

  if (gas_on) { // if gas is on
    if (!power_on && (millis() - power_time) > 500) { // if power is off and gas post-flow time is over
      digitalWrite(GAS_RELAY_PIN, HIGH); // turn off gas valve
      gas_on = false;
    }
  }
}

//Source: Conversation with Bing, 4/15/2023(1) MIG welder settings explained - Amperage & Wire Speed - WeldingPros. https://weldingpros.net/mig-welder-settings-explained/ Accessed 4/15/2023.
//(2) MIG Welder Wire Feed Problems and How To Fix Them. https://migweldingexpert.com/mig-welder-wire-feed-problems/ Accessed 4/15/2023.
//(3) TITANIUM MIG 170 Professional Welder with 120/240 Volt Input for $499.99. https://go.harborfreight.com/coupons/2023/04/180404-64805/ Accessed 4/15/2023.
