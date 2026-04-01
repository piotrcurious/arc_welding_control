
#include <Arduino.h>

#define PWM_PIN 9
#define STEP_PIN 10
#define DIR_PIN 11
#define VOLT_PIN A0
#define CURR_PIN A1

// Optimized PID for difficult conditions (high noise/jitter)
const float TARGET_V = 22.5;
float Kp = 1.2, Ki = 35.0, Kd = 0.01;
float integral = 125.0 / 35.0;
float lastError = 0;
unsigned long lastTime = 0;

// Filtering - critical for noise handling
float vFilt = 25.0, iFilt = 0;
const float alpha = 0.22; // 0.22 smoothing coefficient

// Adaptive WFS
float baseWfs = 145.0, currentWfs = 145.0;

enum State { STARTUP, CONTACT, ARCING };
State mode = STARTUP;

void stepWire(float mms) {
    static unsigned long lastStep = 0;
    if (mms <= 0) return;
    unsigned long interval = 1000000ULL / (mms * 20.0); // 20 steps/mm
    if (micros() - lastStep >= interval) {
        digitalWrite(STEP_PIN, HIGH); delayMicroseconds(2); digitalWrite(STEP_PIN, LOW);
        lastStep = micros();
    }
}

void setup() {
    pinMode(PWM_PIN, OUTPUT); pinMode(STEP_PIN, OUTPUT); pinMode(DIR_PIN, OUTPUT);
    digitalWrite(DIR_PIN, HIGH); analogWrite(PWM_PIN, 0);
    lastTime = micros();
}

void loop() {
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;
    if (dt <= 0) dt = 0.0001; lastTime = now;

    // Filtered Inputs
    vFilt = (alpha * (analogRead(VOLT_PIN) * 0.1)) + ((1.0 - alpha) * vFilt);
    iFilt = (alpha * (analogRead(CURR_PIN) * 0.2)) + ((1.0 - alpha) * iFilt);

    switch(mode) {
        case STARTUP:
            analogWrite(PWM_PIN, 100); digitalWrite(DIR_PIN, HIGH); stepWire(30.0);
            if (iFilt > 30.0 || vFilt < 5.0) mode = CONTACT;
            break;
        case CONTACT:
            analogWrite(PWM_PIN, 255); digitalWrite(DIR_PIN, LOW); stepWire(15.0);
            if (vFilt > 15.0) { mode = ARCING; integral = 130.0 / Ki; }
            break;
        case ARCING:
            float err = TARGET_V - vFilt;
            integral = constrain(integral + err * dt, 0, 255.0/Ki);
            float out = constrain(Kp*err + Ki*integral + Kd*(err-lastError)/dt, 0, 255);
            analogWrite(PWM_PIN, (int)out);

            // Adaptive WFS to handle hand drift
            currentWfs = constrain(baseWfs + (iFilt - 85.0)*0.75, 70, 240);
            digitalWrite(DIR_PIN, HIGH); stepWire(currentWfs);

            if (vFilt < 8.0 && iFilt > 160.0) mode = CONTACT;
            if (vFilt > 42.0) mode = STARTUP;
            lastError = err;
            break;
    }
}
