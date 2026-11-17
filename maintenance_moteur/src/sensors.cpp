#include "sensors.h"
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ==========================================
// Static Sensor Driver Instances & State
// ==========================================
static Adafruit_MPU6050 s_mpu;
static DHT s_dht(PIN_DHT_DATA, DHT_TYPE);

static float s_baseline_offset_z = 9.81f;       // Earth gravity baseline (m/s^2)
static float s_cached_temperature = 25.0f;     // Cached valid temperature (°C)
static float s_last_raw_z = 9.81f;             // Last valid raw Z reading
static uint32_t s_last_dht_poll_ms = 0;        // Timestamp of last DHT polling
static uint32_t s_vib_sample_counter = 0;      // Counter for deterministic fault synthesis

static bool s_mpu_healthy = false;
static bool s_dht_healthy = false;
static bool s_vib_injection_active = false;
static bool s_temp_injection_active = false;

// ==========================================
// Sensor Subsystem Implementations
// ==========================================

void Sensors_Init() {
#if defined(ESP32) && !defined(NATIVE_BUILD)
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setTimeOut(25);
#else
    Wire.begin();
#endif

    // Try primary I2C address 0x68 (AD0 -> GND), then alternate 0x69 (AD0 -> 3V3 / Float)
    s_mpu_healthy = s_mpu.begin(0x68);
    if (!s_mpu_healthy) {
        s_mpu_healthy = s_mpu.begin(0x69);
    }

    if (s_mpu_healthy) {
        s_mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
        s_mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
    }

    // Initialize DHT11
    s_dht.begin();
    float init_temp = s_dht.readTemperature();
    if (!std::isnan(init_temp)) {
        s_cached_temperature = init_temp;
        s_dht_healthy = true;
    } else {
        s_cached_temperature = 25.0f;
        s_dht_healthy = false;
    }

    s_last_dht_poll_ms = millis();
    s_vib_injection_active = false;
    s_temp_injection_active = false;
    s_vib_sample_counter = 0;
}

void Sensors_Calibrate(uint16_t sample_count) {
    if (sample_count == 0 || !s_mpu_healthy) {
        s_baseline_offset_z = 9.81f;
        return;
    }

    float sum = 0.0f;
    uint16_t valid_count = 0;

    for (uint16_t i = 0; i < sample_count; ++i) {
        sensors_event_t a, g, temp;
        if (s_mpu.getEvent(&a, &g, &temp)) {
            sum += a.acceleration.z;
            valid_count++;
        }
        delayMicroseconds(5000); // 5 ms interval
    }

    if (valid_count > 0) {
        s_baseline_offset_z = sum / static_cast<float>(valid_count);
    } else {
        s_baseline_offset_z = 9.81f;
    }
}

bool Sensors_SampleMPU(float &z_sample) {
    sensors_event_t a, g, temp;
    bool ok = s_mpu.getEvent(&a, &g, &temp);

    float raw_z = 0.0f;
    if (ok) {
        s_mpu_healthy = true;
        raw_z = a.acceleration.z;
        s_last_raw_z = raw_z;
    } else {
        s_mpu_healthy = false;
        raw_z = s_last_raw_z;
    }

    // Baseline detrending: remove constant gravitational DC offset
    float detrended = raw_z - s_baseline_offset_z;

    // Simulated Fault Injection Hook (INJECT_VIB)
    if (s_vib_injection_active) {
        // High-frequency (65 Hz) & high-amplitude (3.5 m/s^2) vibration anomaly
        float t = static_cast<float>(s_vib_sample_counter) / SAMPLING_FREQ_HZ;
        s_vib_sample_counter++;
        float fault_signal = 3.5f * sinf(2.0f * static_cast<float>(M_PI) * 65.0f * t);
        z_sample = detrended + fault_signal;
    } else {
        z_sample = detrended;
        s_vib_sample_counter = 0;
    }

    return ok;
}

bool Sensors_SampleDHT(float &temperature) {
    uint32_t now = millis();

    // Non-blocking rate-limited polling (DHT_INTERVAL_MS = 2000 ms)
    if (s_last_dht_poll_ms == 0 || (now - s_last_dht_poll_ms) >= DHT_INTERVAL_MS) {
        s_last_dht_poll_ms = now;
        float read_temp = s_dht.readTemperature();
        if (!std::isnan(read_temp)) {
            s_cached_temperature = read_temp;
            s_dht_healthy = true;
        } else {
            s_dht_healthy = false;
        }
    }

    // Simulated Fault Injection Hook (INJECT_TEMP)
    if (s_temp_injection_active) {
        // Critical thermal runaway anomaly (80.0 °C)
        temperature = 80.0f;
    } else {
        temperature = s_cached_temperature;
    }

    return s_dht_healthy;
}

void Sensors_SetVibrationInjection(bool active) {
    s_vib_injection_active = active;
    if (!active) {
        s_vib_sample_counter = 0;
    }
}

void Sensors_SetTemperatureInjection(bool active) {
    s_temp_injection_active = active;
}

bool Sensors_IsVibrationInjected() {
    return s_vib_injection_active;
}

bool Sensors_IsTemperatureInjected() {
    return s_temp_injection_active;
}

float Sensors_GetCalibrationOffset() {
    return s_baseline_offset_z;
}

void Sensors_SetCalibrationOffset(float offset) {
    s_baseline_offset_z = offset;
}

bool Sensors_IsMPUHealthy() {
    return s_mpu_healthy;
}

bool Sensors_IsDHTHealthy() {
    return s_dht_healthy;
}
