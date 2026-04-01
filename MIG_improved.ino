
// Optimized Arduino MIG welder controller with PID and adaptive feed
// Version 2.9 - Smoothed PID and Relaxed Adaptive Feed

#define VOLTAGE_PIN A0
#define CURRENT_PIN A1
#define TARGET_VOLT_PIN A2
#define TARGET_WFS_PIN A3
#define PWM_PIN 9
#define STEP_PIN 10
#define DIR_PIN 11
#define GAS_RELAY_PIN 6
#define POWER_RELAY_PIN 7

// PID Constants for CV Mode (1ms loop)
float Kp = 1.2;
float Ki = 35.0;
float Kd = 0.01;

float integral = 110.0;
float prevError = 0;
float currentDuty = 0;

enum State {
  GAS_PREFLOW,
  SENSE_CONTACT,
  RETRACT,
  BURN_PULSE,
  ARC_SUSTAIN
};

State currentState = GAS_PREFLOW;
unsigned long stateTimer = 0;
unsigned long lastLoopTime = 0;
unsigned long lastStepTime = 0;

void stepMotor() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(STEP_PIN, LOW);
}

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(GAS_RELAY_PIN, OUTPUT);
  pinMode(POWER_RELAY_PIN, OUTPUT);

  TCCR1B = (TCCR1B & 0xF8) | 0x01;

  Serial.begin(115200);
  Serial.println("Optimized Welder V2.9 initialized");

  currentState = GAS_PREFLOW;
  stateTimer = millis();
  lastLoopTime = micros();
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastLoopTime) / 1000000.0;
  if (dt < 0.001) return; // 1kHz Loop
  lastLoopTime = now;

  float v_fb = analogRead(VOLTAGE_PIN) / 10.0;
  float i_fb = analogRead(CURRENT_PIN) / 5.0;
  float targetV = map(analogRead(TARGET_VOLT_PIN), 0, 1023, 150, 300) / 10.0;
  float targetWFS = map(analogRead(TARGET_WFS_PIN), 0, 1023, 10, 100);

  float error = targetV - v_fb;

  // State Machine
  switch (currentState) {
    case GAS_PREFLOW:
      digitalWrite(GAS_RELAY_PIN, HIGH);
      currentDuty = 0;
      if (millis() - stateTimer > 200) {
        currentState = SENSE_CONTACT;
        digitalWrite(POWER_RELAY_PIN, HIGH);
      }
      break;

    case SENSE_CONTACT:
      digitalWrite(DIR_PIN, HIGH);
      currentDuty = 80;
      if (now - lastStepTime > 2000) {
          stepMotor();
          lastStepTime = now;
      }
      if (v_fb < 5.0 && i_fb > 2.0) {
        currentState = RETRACT;
        stateTimer = millis();
      }
      break;

    case RETRACT:
      digitalWrite(DIR_PIN, LOW);
      currentDuty = 0;
      for (int i = 0; i < 15; i++) {
        stepMotor();
        delayMicroseconds(500);
      }
      currentState = BURN_PULSE;
      stateTimer = millis();
      break;

    case BURN_PULSE:
      currentDuty = 255;
      if (millis() - stateTimer > 25) {
        currentState = ARC_SUSTAIN;
        integral = 110.0;
        prevError = error;
      }
      break;

    case ARC_SUSTAIN:
      // PID Calculation
      integral += error * Ki * dt;
      integral = constrain(integral, 20, 240);
      float derivative = (error - prevError) / dt;
      currentDuty = constrain(integral + Kp * error + Kd * derivative, 20, 255);

      digitalWrite(DIR_PIN, HIGH);

      // Relaxed Adaptive WFS
      float wfs_adj = 1.0 + (v_fb - targetV) * 0.1;
      wfs_adj = constrain(wfs_adj, 0.7, 1.5);

      float effectiveWFS = targetWFS * wfs_adj;
      unsigned long stepDelay = 1000000 / (effectiveWFS * 20);

      if (now - lastStepTime > stepDelay) {
          stepMotor();
          lastStepTime = now;
      }

      if (v_fb > 40.0 && i_fb < 0.2) { // Arc broken
        currentState = SENSE_CONTACT;
      }
      break;
  }

  prevError = error;
  analogWrite(PWM_PIN, (int)currentDuty);
}
