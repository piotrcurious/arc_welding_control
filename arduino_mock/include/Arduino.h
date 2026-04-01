
#ifndef ARDUINO_H
#define ARDUINO_H

#include <stdint.h>
#include <iostream>
#include <map>
#include <string>
#include <math.h>
#include <algorithm>

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define A6 20
#define A7 21

#define DEFAULT 1

typedef bool boolean;
typedef uint8_t byte;

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogWrite(uint8_t pin, int val);
void analogReference(uint8_t mode);
void analogWriteResolution(int res);
void analogWriteFrequency(uint8_t pin, uint32_t freq);

unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);

// Math
#ifndef min
template<class T, class L>
auto min(const T& a, const L& b) -> decltype(a < b ? a : b) { return a < b ? a : b; }
#endif

#ifndef max
template<class T, class L>
auto max(const T& a, const L& b) -> decltype(a > b ? a : b) { return a > b ? a : b; }
#endif

#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
long map(long x, long in_min, long in_max, long out_min, long out_max);

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif
#define sq(x) ((x)*(x))

// Mock specific
struct MockState {
    std::map<int, int> digitalPins;
    std::map<int, int> analogPins;
    unsigned long currentTime;
};

extern MockState mock;

class SerialProxy {
public:
    void begin(unsigned long baud) {}
    void print(const std::string& s) { std::cout << "LOG:" << s; }
    void print(const char* s) { std::cout << "LOG:" << s; }
    void print(float f) { std::cout << "LOG:" << f; }
    void print(int i) { std::cout << "LOG:" << i; }
    void println(const std::string& s) { std::cout << "LOG:" << s << std::endl; }
    void println(const char* s) { std::cout << "LOG:" << s << std::endl; }
    void println(float f) { std::cout << "LOG:" << f << std::endl; }
    void println(int i) { std::cout << "LOG:" << i << std::endl; }
};

extern SerialProxy Serial;

// AVR specific defines
extern uint8_t mock_TCCR1B;
#define TCCR1B mock_TCCR1B
#define B11111000 0xF8
#define B00000001 0x01

#endif
