
#include "Arduino.h"
#include <iostream>
#include <string>
#include <sstream>

MockState mock;
SerialProxy Serial;

uint8_t mock_TCCR1B = 0;

void pinMode(uint8_t pin, uint8_t mode) {}

void digitalWrite(uint8_t pin, uint8_t val) {
    mock.digitalPins[pin] = val;
    std::cout << "DW:" << (int)pin << ":" << (int)val << std::endl;
}

int digitalRead(uint8_t pin) {
    return mock.digitalPins[pin];
}

int analogRead(uint8_t pin) {
    std::cout << "AR:" << (int)pin << std::endl;
    std::string line;
    if (std::getline(std::cin, line)) {
        if (line.empty()) exit(0);
        try {
            return std::stoi(line);
        } catch (...) {
            return 0;
        }
    }
    exit(0);
}

void analogWrite(uint8_t pin, int val) {
    mock.analogPins[pin] = val;
    std::cout << "AW:" << (int)pin << ":" << val << std::endl;
}

void analogReference(uint8_t mode) {}
void analogWriteResolution(int res) {}
void analogWriteFrequency(uint8_t pin, uint32_t freq) {}

unsigned long micros() {
    std::cout << "MT" << std::endl;
    std::string line;
    if (std::getline(std::cin, line)) {
        if (line.empty()) exit(0);
        try {
            return std::stoul(line);
        } catch (...) {
            return 0;
        }
    }
    exit(0);
}

unsigned long millis() { return micros() / 1000; }

void delay(unsigned long ms) {
    std::cout << "DL:" << ms << std::endl;
}

void delayMicroseconds(unsigned int us) {
    std::cout << "DM:" << us << std::endl;
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup();
void loop();

int main() {
    setup();
    while (true) {
        loop();
        std::cout << "LS" << std::endl;
    }
    return 0;
}
