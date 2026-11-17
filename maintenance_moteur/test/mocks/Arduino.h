#pragma once
#include <cstdint>
#include <cstddef>
#include <cmath>
#include <string>
#include <sstream>
#include <deque>
#include <vector>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <algorithm>
#include <cstring>

// =============================================================================
// Arduino Fundamental Constants
// =============================================================================
#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define INPUT_PULLDOWN 0x3

#define CHANGE 1
#define FALLING 2
#define RISING 3

#define LED_BUILTIN 2

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif
#ifndef HALF_PI
#define HALF_PI 1.5707963267948966192313216916398
#endif
#ifndef TWO_PI
#define TWO_PI 6.2831853071795864769252867665590
#endif
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295769236907684886
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105
#endif
#ifndef EULER
#define EULER 2.718281828459045235360287471352
#endif

// Basic Arduino Types
using boolean = bool;
using byte = uint8_t;
using word = uint16_t;
using String = std::string;

// Standard Math Imports for Arduino compatibility
using std::isnan;
using std::isinf;
using std::abs;
using std::round;

// Arduino Math Helper Templates & Functions
template<typename A, typename B>
constexpr auto _arduino_min(A a, B b) { return (a < b) ? a : b; }

template<typename A, typename B>
constexpr auto _arduino_max(A a, B b) { return (a > b) ? a : b; }

#ifndef min
#define min(a,b) _arduino_min((a),(b))
#endif

#ifndef max
#define max(a,b) _arduino_max((a),(b))
#endif

template<typename T, typename L, typename H>
constexpr auto constrain(T amt, L low, H high) {
    return (amt < low) ? low : ((amt > high) ? high : amt);
}

template<typename T>
constexpr auto sq(T x) {
    return x * x;
}

template<typename T>
constexpr auto radians(T deg) {
    return deg * DEG_TO_RAD;
}

template<typename T>
constexpr auto degrees(T rad) {
    return rad * RAD_TO_DEG;
}

// Bit Manipulation Macros
#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
#define bit(b) (1UL << (b))

// -----------------------------------------------------------------------------
// Virtual Clock Management
// -----------------------------------------------------------------------------
class MockClock {
public:
    inline static uint64_t virtual_micros = 0;

    static void reset() {
        virtual_micros = 0;
    }
    static void advance_us(uint64_t us) {
        virtual_micros += us;
    }
    static void advance_ms(uint32_t ms) {
        virtual_micros += static_cast<uint64_t>(ms) * 1000ULL;
    }
    static void set_micros(uint64_t us) {
        virtual_micros = us;
    }
    static void set_millis(uint32_t ms) {
        virtual_micros = static_cast<uint64_t>(ms) * 1000ULL;
    }
    static uint64_t get_micros() {
        return virtual_micros;
    }
    static uint64_t get_millis() {
        return virtual_micros / 1000ULL;
    }
};

inline uint32_t millis() {
    return static_cast<uint32_t>(MockClock::virtual_micros / 1000ULL);
}

inline uint32_t micros() {
    return static_cast<uint32_t>(MockClock::virtual_micros);
}

inline void delay(uint32_t ms) {
    MockClock::advance_ms(ms);
}

inline void delayMicroseconds(uint32_t us) {
    MockClock::advance_us(us);
}

inline void yield() {}

// -----------------------------------------------------------------------------
// Mock GPIO Tracking System
// -----------------------------------------------------------------------------
class MockGPIO {
public:
    inline static uint8_t pin_mode[64] = {0};
    inline static uint8_t pin_state[64] = {0};
    inline static uint32_t pin_pwm[64] = {0};
    inline static uint32_t ledc_freq[16] = {0};
    inline static uint8_t ledc_res[16] = {0};
    inline static uint8_t ledc_pin_map[16] = {0};

    static void reset() {
        for (int i = 0; i < 64; ++i) {
            pin_mode[i] = INPUT;
            pin_state[i] = LOW;
            pin_pwm[i] = 0;
        }
        for (int i = 0; i < 16; ++i) {
            ledc_freq[i] = 0;
            ledc_res[i] = 0;
            ledc_pin_map[i] = 0;
        }
    }
};

inline void pinMode(uint8_t pin, uint8_t mode) {
    if (pin < 64) MockGPIO::pin_mode[pin] = mode;
}

inline void digitalWrite(uint8_t pin, uint8_t val) {
    if (pin < 64) MockGPIO::pin_state[pin] = val;
}

inline int digitalRead(uint8_t pin) {
    return (pin < 64) ? MockGPIO::pin_state[pin] : LOW;
}

inline void analogWrite(uint8_t pin, int val) {
    if (pin < 64) MockGPIO::pin_pwm[pin] = static_cast<uint32_t>(val);
}

inline int analogRead(uint8_t /*pin*/) {
    return 0;
}

// ESP32 LEDC PWM emulation
inline double ledcSetup(uint8_t channel, double freq, uint8_t resolution_bits) {
    if (channel < 16) {
        MockGPIO::ledc_freq[channel] = static_cast<uint32_t>(freq);
        MockGPIO::ledc_res[channel] = resolution_bits;
    }
    return freq;
}

inline void ledcAttachPin(uint8_t pin, uint8_t channel) {
    if (channel < 16) {
        MockGPIO::ledc_pin_map[channel] = pin;
    }
}

inline void ledcWrite(uint8_t channel, uint32_t duty) {
    if (channel < 16) {
        uint8_t pin = MockGPIO::ledc_pin_map[channel];
        if (pin < 64) MockGPIO::pin_pwm[pin] = duty;
    }
    if (channel < 64) {
        MockGPIO::pin_pwm[channel] = duty;
    }
}

inline void ledcDetachPin(uint8_t /*pin*/) {}

inline void tone(uint8_t pin, unsigned int /*frequency*/, unsigned long /*duration*/ = 0) {
    digitalWrite(pin, HIGH);
}

inline void noTone(uint8_t pin) {
    digitalWrite(pin, LOW);
}

// -----------------------------------------------------------------------------
// Mock Serial (HardwareSerial & Stream Emulation)
// -----------------------------------------------------------------------------
class MockSerial {
private:
    std::deque<char> rx_buffer;
    std::stringstream tx_stream;
    unsigned long baud_rate = 115200;
    bool is_active = true;

public:
    void begin(unsigned long baud) {
        baud_rate = baud;
        is_active = true;
        rx_buffer.clear();
        tx_stream.str("");
        tx_stream.clear();
    }

    void end() {
        is_active = false;
    }

    explicit operator bool() const {
        return is_active;
    }

    // RX injection APIs
    void inject_input(const std::string &data) {
        for (char c : data) {
            rx_buffer.push_back(c);
        }
    }

    void clear_input() {
        rx_buffer.clear();
    }

    int available() {
        return static_cast<int>(rx_buffer.size());
    }

    int peek() {
        if (rx_buffer.empty()) return -1;
        return static_cast<int>(static_cast<unsigned char>(rx_buffer.front()));
    }

    int read() {
        if (rx_buffer.empty()) return -1;
        char c = rx_buffer.front();
        rx_buffer.pop_front();
        return static_cast<int>(static_cast<unsigned char>(c));
    }

    size_t readBytes(char *buffer, size_t length) {
        size_t count = 0;
        while (count < length && !rx_buffer.empty()) {
            buffer[count++] = rx_buffer.front();
            rx_buffer.pop_front();
        }
        return count;
    }

    std::string readStringUntil(char terminator) {
        std::string result = "";
        while (!rx_buffer.empty()) {
            char c = rx_buffer.front();
            rx_buffer.pop_front();
            if (c == terminator) break;
            result += c;
        }
        return result;
    }

    std::string readString() {
        std::string result = "";
        while (!rx_buffer.empty()) {
            result += rx_buffer.front();
            rx_buffer.pop_front();
        }
        return result;
    }

    // TX capture APIs
    size_t write(uint8_t c) {
        tx_stream << static_cast<char>(c);
        return 1;
    }

    size_t write(const uint8_t *buffer, size_t size) {
        tx_stream.write(reinterpret_cast<const char*>(buffer), size);
        return size;
    }

    size_t write(const char *str) {
        if (!str) return 0;
        tx_stream << str;
        return std::strlen(str);
    }

    size_t print(const char *s) {
        if (s) tx_stream << s;
        return s ? std::strlen(s) : 0;
    }

    size_t print(const std::string &s) {
        tx_stream << s;
        return s.size();
    }

    size_t print(char c) {
        tx_stream << c;
        return 1;
    }

    size_t print(unsigned char b, int base = DEC) {
        return print(static_cast<unsigned long>(b), base);
    }

    size_t print(int n, int base = DEC) {
        return print(static_cast<long>(n), base);
    }

    size_t print(unsigned int n, int base = DEC) {
        return print(static_cast<unsigned long>(n), base);
    }

    size_t print(long n, int base = DEC) {
        if (base == DEC) {
            tx_stream << n;
        } else if (base == HEX) {
            tx_stream << std::hex << std::uppercase << n << std::dec;
        } else if (base == OCT) {
            tx_stream << std::oct << n << std::dec;
        } else if (base == BIN) {
            if (n == 0) {
                tx_stream << "0";
            } else {
                std::string s;
                unsigned long un = (n < 0) ? -n : n;
                while (un > 0) {
                    s = ((un & 1) ? "1" : "0") + s;
                    un >>= 1;
                }
                if (n < 0) s = "-" + s;
                tx_stream << s;
            }
        }
        return 0;
    }

    size_t print(unsigned long n, int base = DEC) {
        if (base == DEC) {
            tx_stream << n;
        } else if (base == HEX) {
            tx_stream << std::hex << std::uppercase << n << std::dec;
        } else if (base == OCT) {
            tx_stream << std::oct << n << std::dec;
        } else if (base == BIN) {
            if (n == 0) {
                tx_stream << "0";
            } else {
                std::string s;
                while (n > 0) {
                    s = ((n & 1) ? "1" : "0") + s;
                    n >>= 1;
                }
                tx_stream << s;
            }
        }
        return 0;
    }

    size_t print(double v, int prec = 2) {
        if (std::isnan(v)) {
            tx_stream << "nan";
            return 3;
        }
        if (std::isinf(v)) {
            tx_stream << "inf";
            return 3;
        }
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(prec) << v;
        tx_stream << ss.str();
        return ss.str().length();
    }

    size_t println(const char *s) {
        size_t n = print(s);
        tx_stream << "\n";
        return n + 1;
    }

    size_t println(const std::string &s) {
        size_t n = print(s);
        tx_stream << "\n";
        return n + 1;
    }

    size_t println(char c) {
        size_t n = print(c);
        tx_stream << "\n";
        return n + 1;
    }

    size_t println(unsigned char b, int base = DEC) {
        size_t written = print(b, base);
        tx_stream << "\n";
        return written + 1;
    }

    size_t println(int n, int base = DEC) {
        size_t written = print(n, base);
        tx_stream << "\n";
        return written + 1;
    }

    size_t println(unsigned int n, int base = DEC) {
        size_t written = print(n, base);
        tx_stream << "\n";
        return written + 1;
    }

    size_t println(long n, int base = DEC) {
        size_t written = print(n, base);
        tx_stream << "\n";
        return written + 1;
    }

    size_t println(unsigned long n, int base = DEC) {
        size_t written = print(n, base);
        tx_stream << "\n";
        return written + 1;
    }

    size_t println(double v, int prec = 2) {
        size_t written = print(v, prec);
        tx_stream << "\n";
        return written + 1;
    }

    size_t println() {
        tx_stream << "\n";
        return 1;
    }

    void flush() {}

    std::string get_output() const {
        return tx_stream.str();
    }

    void clear_output() {
        tx_stream.str("");
        tx_stream.clear();
    }

    void reset() {
        rx_buffer.clear();
        tx_stream.str("");
        tx_stream.clear();
        baud_rate = 115200;
        is_active = true;
    }
};

inline MockSerial Serial;
