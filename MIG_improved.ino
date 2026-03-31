
// Advanced Arduino MIG arc welding controller with contact-triggered pulse
// Improved version based on simulation testing

#define VOLTAGE_PIN A0
#define CURRENT_PIN A1
#define WIRE_FEED_RATE_PIN A2
#define BURN_PULSE_FACTOR_PIN A3
#define PWM_PIN 9
#define STEP_PIN 10
#define DIR_PIN 11
#define GAS_RELAY_PIN 6
#define POWER_RELAY_PIN 7

// State definitions
enum State {
  WAIT_FOR_TRIGGER,
  GAS_PREFLOW,
  SENSE_CONTACT,
  RETRACT,
  BURN_PULSE,
  ARC_SUSTAIN,
  POST_FLOW
};

State currentState = WAIT_FOR_TRIGGER;
unsigned long stateTimer = 0;
int pulseCount = 0;

// Configurable parameters via pots
float targetWireFeedRate; // 0-100 mm/s
float pulseDurationFactor; // 0.5-2.0 multiplier

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(GAS_RELAY_PIN, OUTPUT);
  pinMode(POWER_RELAY_PIN, OUTPUT);

  // High frequency PWM for buck converter
  TCCR1B = (TCCR1B & 0xF8) | 0x01;

  Serial.begin(115200);
}

void loop() {
  int v_raw = analogRead(VOLTAGE_PIN);
  int i_raw = analogRead(CURRENT_PIN);

  // Read potentiometers
  targetWireFeedRate = map(analogRead(WIRE_FEED_RATE_PIN), 0, 1023, 10, 100);
  pulseDurationFactor = map(analogRead(BURN_PULSE_FACTOR_PIN), 0, 1023, 50, 200) / 100.0;

  switch (currentState) {
    case WAIT_FOR_TRIGGER:
      analogWrite(PWM_PIN, 0);
      digitalWrite(POWER_RELAY_PIN, LOW);
      digitalWrite(GAS_RELAY_PIN, LOW);
      // For simulation we auto-trigger
      currentState = GAS_PREFLOW;
      stateTimer = millis();
      break;

    case GAS_PREFLOW:
      digitalWrite(GAS_RELAY_PIN, HIGH);
      if (millis() - stateTimer > 500) {
        currentState = SENSE_CONTACT;
        digitalWrite(POWER_RELAY_PIN, HIGH);
      }
      break;

    case SENSE_CONTACT:
      analogWrite(PWM_PIN, 50); // Low sensing current
      digitalWrite(DIR_PIN, HIGH); // Forward
      stepMotor();
      if (v_raw < 50) { // Contact detected (V < 0.5V roughly)
        currentState = RETRACT;
        stateTimer = millis();
      }
      break;

    case RETRACT:
      digitalWrite(DIR_PIN, LOW); // Reverse
      for (int i = 0; i < 5; i++) {
        stepMotor();
        delayMicroseconds(500);
      }
      currentState = BURN_PULSE;
      stateTimer = millis();
      break;

    case BURN_PULSE:
      analogWrite(PWM_PIN, 255); // Maximum pulse
      if (millis() - stateTimer > (10 * pulseDurationFactor)) {
        currentState = ARC_SUSTAIN;
        stateTimer = millis();
      }
      break;

    case ARC_SUSTAIN:
      analogWrite(PWM_PIN, 120); // Maintain arc
      digitalWrite(DIR_PIN, HIGH);
      stepMotor();
      delayMicroseconds(1000 / (targetWireFeedRate / 10.0)); // proportional to target rate

      if (v_raw > 800) { // Arc lost
        currentState = SENSE_CONTACT;
      }

      if (millis() - stateTimer > 100) {
        currentState = BURN_PULSE;
        stateTimer = millis();
        pulseCount++;
        if (pulseCount > 20) { // Weld finished example
           pulseCount = 0;
           currentState = POST_FLOW;
           stateTimer = millis();
        }
      }
      break;

    case POST_FLOW:
      analogWrite(PWM_PIN, 0);
      digitalWrite(POWER_RELAY_PIN, LOW);
      if (millis() - stateTimer > 1000) {
        digitalWrite(GAS_RELAY_PIN, LOW);
        currentState = WAIT_FOR_TRIGGER;
      }
      break;
  }
}

void stepMotor() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(STEP_PIN, LOW);
}
