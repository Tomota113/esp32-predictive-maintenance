#pragma once
#include <cstdint>
#include <cmath>
#include "Arduino.h"

// =============================================================================
// Mock DHT Sensor Driver with Settable Temp/Humidity and Fault Injection
// =============================================================================

#define DHT11 11
#define DHT22 22
#define DHT21 21
#define AM2301 21

class DHT {
private:
    uint8_t pin = 0;
    uint8_t type = DHT11;
    static inline float temperature = 28.5f;
    static inline float humidity = 45.0f;
    static inline bool force_nan = false;
    static inline bool is_begun = false;

public:
    DHT(uint8_t p, uint8_t t, uint8_t count = 6)
        : pin(p), type(t) {
        (void)count;
    }

    void begin(uint8_t usec = 55) {
        (void)usec;
        is_begun = true;
    }

    void set_temperature(float t) {
        temperature = t;
        force_nan = false;
    }

    void set_humidity(float h) {
        humidity = h;
    }

    void set_nan_failure(bool fail) {
        force_nan = fail;
    }

    float get_temperature() const {
        return temperature;
    }

    float get_humidity() const {
        return humidity;
    }

    bool is_nan_failure() const {
        return force_nan;
    }

    void reset() {
        temperature = 28.5f;
        humidity = 45.0f;
        force_nan = false;
        is_begun = true;
    }

    float readTemperature(bool isFahrenheit = false, bool force = false) {
        (void)force;
        if (force_nan) return NAN;
        return isFahrenheit ? (temperature * 1.8f + 32.0f) : temperature;
    }

    float readHumidity(bool force = false) {
        (void)force;
        if (force_nan) return NAN;
        return humidity;
    }

    bool read(bool force = false) {
        (void)force;
        return !force_nan;
    }

    float computeHeatIndex(float temp, float percentHumidity, bool isFahrenheit = false) {
        float f = isFahrenheit ? temp : (temp * 1.8f + 32.0f);
        float hi = 0.5f * (f + 61.0f + ((f - 68.0f) * 1.2f) + (percentHumidity * 0.094f));
        return isFahrenheit ? hi : ((hi - 32.0f) * 0.5555555555555556f);
    }
};

inline DHT mock_dht(0, DHT11);
