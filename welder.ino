
// Arduino uno code implementation of dc-dc buck converter with thermistor based thermal protection, hardware WDT, and constant current priority constant power output above 1V output voltage profile, constant voltage current limited profile in-between 1V with current increasing to twice the normal set value down to 0.5V range and short circuit protection below 0.5V
// Assume current being adjustable by A6 analog input pin
// Assume short circuit protection threshold being adjustable by A5 analog pin
// Assume constant - voltage to constant current threshold being adjustable by A4 analog pin

#include <avr/wdt.h> // include watchdog timer library

#define PWM_PIN 9 // define PWM output pin
#define VOUT_PIN A0 // define output voltage feedback pin
#define IOUT_PIN A1 // define output current feedback pin
#define THERM_PIN A2 // define thermistor input pin
#define LED_PIN 13 // define LED indicator pin

#define VREF 5.0 // define reference voltage for ADC
#define R1 10000.0 // define resistor value for voltage divider
#define R2 1000.0 // define resistor value for voltage divider
#define RSENSE 0.1 // define sense resistor value for current measurement
#define RTH 10000.0 // define nominal thermistor resistance at 25 C
#define BETA 3950.0 // define beta coefficient of thermistor
#define TMAX 80.0 // define maximum temperature limit in C

float vout; // declare output voltage variable
float iout; // declare output current variable
float temp; // declare temperature variable
float vset; // declare voltage setpoint variable
float iset; // declare current setpoint variable
float ishort; // declare short circuit threshold variable
float icc; // declare constant current threshold variable
int duty; // declare duty cycle variable

void setup() {
  pinMode(PWM_PIN, OUTPUT); // set PWM pin as output
  pinMode(LED_PIN, OUTPUT); // set LED pin as output
  analogReference(DEFAULT); // set ADC reference to default (5V)
  analogWrite(PWM_PIN, 0); // initialize PWM to zero
  Serial.begin(9600); // initialize serial communication at 9600 baud rate
  wdt_enable(WDTO_2S); // enable watchdog timer with 2 seconds timeout
}

void loop() {
  wdt_reset(); // reset watchdog timer
  
  vout = analogRead(VOUT_PIN) * VREF / 1024.0 * (R1 + R2) / R2; // read and calculate output voltage
  iout = analogRead(IOUT_PIN) * VREF / 1024.0 / RSENSE; // read and calculate output current
  
  vset = map(analogRead(A4), 0, 1023, 50, 100) / 10.0; // read and map analog input A4 to voltage setpoint from 0.5V to 10V
  iset = map(analogRead(A6), 0, 1023, 50, 500) / 100.0; // read and map analog input A6 to current setpoint from 0.5A to 5A
  ishort = map(analogRead(A5), 0, 1023, iset / 2.0, iset * 2.0); // read and map analog input A5 to short circuit threshold from half to twice the current setpoint 

  // read and map analog input A4 to constant current threshold from current setpoint to half the current setpoint
  icc = map(analogRead(A4), vset * R2 / (R1 + R2) * (1024 / VREF), vset * R2 / (R1 + R2) * (1024 / VREF) - (vset - 1.0) * (R1 + R2) / R2 * (1024 / VREF), iset * RSENSE * (1024 / VREF), iset * RSENSE * (1024 / VREF) - (iset - icc) * RSENSE * (1024 / VREF)) * VREF / 1024.0 / RSENSE; // read and map analog input A4 to constant current threshold from current setpoint to half the current setpoint
  
  temp = BETA / log(analogRead(THERM_PIN) * RTH / (1024.0 - analogRead(THERM_PIN))) - 273.15; // read and calculate temperature using thermistor equation
  
  if (temp > TMAX) { // check if temperature exceeds maximum limit
    duty = 0; // set duty cycle to zero
    digitalWrite(LED_PIN, HIGH); // turn on LED indicator
  }
  // invalid else if (iout > ishort) { // check if output current exceeds short circuit threshold
  else if (iout > ishort && vout < 0.5) { // check if output voltage is bellw 0.5V and output current exceeds short circuit threshold
    duty = 1; // set duty cycle to smallest current 
    digitalWrite(LED_PIN, HIGH); // turn on LED indicator
  }
  else if (vout > vset) { // check if output voltage exceeds voltage setpoint
    duty = duty - 1; // decrease duty cycle by one
    digitalWrite(LED_PIN, LOW); // turn off LED indicator
  }
  else if (vout < vset && iout < icc) { // check if output voltage is below voltage setpoint and output current is below constant current threshold
    duty = duty + 1; // increase duty cycle by one
    digitalWrite(LED_PIN, LOW); // turn off LED indicator
  }
  else if (vout < vset && iout > icc && vout > 1.0) { // check if output voltage is below voltage setpoint and output current is above constant current threshold and output voltage is above 1V
    duty = map(iout, icc, iset, map(vset, vset, 1.0, map(iset, icc, iset, 0, 255), map(icc, icc, iset, 0, 255)), map(iset, icc, iset, 0, 255)); // map output current to duty cycle using constant power equation
    digitalWrite(LED_PIN, LOW); // turn off LED indicator
  }
  else if (vout < vset && iout > icc && vout <= 1.0 && vout >= 0.5) { // check if output voltage is below voltage setpoint and output current is above constant current threshold and output voltage is between 1V and 0.5V
    duty = map(vout, vset, 0.5, map(iset, icc, iset * 2.0, 0, 255), map(ishort, icc, iset * 2.0, 0 ,255)); // map output voltage to duty cycle using constant voltage equation with increasing current limit
    digitalWrite(LED_PIN, LOW); // turn off LED indicator
  }
  
  duty = constrain(duty, 0 ,255); // constrain duty cycle between 0 and 255
  
  analogWrite(PWM_PIN, duty); // write duty cycle to PWM pin
  
  
}
