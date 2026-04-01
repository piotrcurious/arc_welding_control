
// Arduino MIG arc welding controller with improved reliability
#include <Arduino.h>

// Define pins
#define VOLTAGE_PIN A0
#define CURRENT_PIN A1
#define WIRE_FEED_PIN A2
#define BURN_PULSE_PIN A3
#define PWM_PIN 9
#define STEP_PIN 10
#define DIR_PIN 11
#define GAS_RELAY_PIN 6
#define POWER_RELAY_PIN 7

// Define constants and variables
const float VOLTAGE_SCALE = 0.1;
const float CURRENT_SCALE = 0.2;
const float PI_VAL = 3.14159;

float voltage;
float current;
float wire_feed_rate = 150.0; // mm/s
float wire_feed_velocity;

bool gas_on;
bool power_on;
bool wire_feed_on;
bool burn_pulse_on;

unsigned long wire_feed_time;
unsigned long burn_pulse_time;
float burn_pulse_duration = 5.0; // ms

int step_dir = 1;
float step_interval;

void read_inputs() {
  voltage = analogRead(VOLTAGE_PIN) * VOLTAGE_SCALE;
  current = analogRead(CURRENT_PIN) * CURRENT_SCALE;
  wire_feed_velocity = wire_feed_rate;
  step_interval = 1000.0 / (wire_feed_velocity * 20.0); // 20 steps/mm
}

void control_pwm() {
  if (voltage < 5.0 || current > 200.0) { // Contact
      analogWrite(PWM_PIN, 255);
  } else {
      // Proportional control
      float error = 22.5 - voltage;
      static float duty = 120;
      duty += error * 0.1;
      duty = constrain(duty, 40, 200);
      analogWrite(PWM_PIN, (int)duty);
  }
}

void control_stepper() {
  static unsigned long lastStep = 0;
  unsigned long now = micros();
  if (now - lastStep > step_interval * 1000.0) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(STEP_PIN, LOW);
    lastStep = now;
  }
}

void setup() {
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(CURRENT_PIN, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(DIR_PIN, HIGH);
  wire_feed_time = micros();
  read_inputs();
}

void loop() {
  read_inputs();
  control_pwm();
  control_stepper();
}
