#pragma once
#include <cstdint>
#include <cmath>
#include <vector>
#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_Sensor.h"

// =============================================================================
// Mock Adafruit MPU6050 Driver with Synthetic Signal Generator
// =============================================================================

#define MPU6050_I2CADDR_DEFAULT 0x68
#define MPU6050_DEVICE_ID 0x68

typedef enum {
    MPU6050_RANGE_2_G = 0b00,
    MPU6050_RANGE_4_G = 0b01,
    MPU6050_RANGE_8_G = 0b10,
    MPU6050_RANGE_16_G = 0b11,
} mpu6050_accel_range_t;

typedef enum {
    MPU6050_RANGE_250_DEG = 0b00,
    MPU6050_RANGE_500_DEG = 0b01,
    MPU6050_RANGE_1000_DEG = 0b10,
    MPU6050_RANGE_2000_DEG = 0b11,
} mpu6050_gyro_range_t;

#define MPU6050_SCALE_250_DEG MPU6050_RANGE_250_DEG
#define MPU6050_SCALE_500_DEG MPU6050_RANGE_500_DEG
#define MPU6050_SCALE_1000_DEG MPU6050_RANGE_1000_DEG
#define MPU6050_SCALE_2000_DEG MPU6050_RANGE_2000_DEG

typedef enum {
    MPU6050_BAND_260_HZ = 0,
    MPU6050_BAND_184_HZ = 1,
    MPU6050_BAND_94_HZ  = 2,
    MPU6050_BAND_44_HZ  = 3,
    MPU6050_BAND_21_HZ  = 4,
    MPU6050_BAND_10_HZ  = 5,
    MPU6050_BAND_5_HZ   = 6,
} mpu6050_bandwidth_t;

typedef enum {
    MPU6050_HIGHPASS_DISABLE = 0,
    MPU6050_HIGHPASS_5_HZ = 1,
    MPU6050_HIGHPASS_2_5_HZ = 2,
    MPU6050_HIGHPASS_1_25_HZ = 3,
    MPU6050_HIGHPASS_0_63_HZ = 4,
    MPU6050_HIGHPASS_HOLD = 7,
} mpu6050_highpass_t;

typedef enum {
    MPU6050_CYCLE_1_25_HZ = 0,
    MPU6050_CYCLE_5_HZ = 1,
    MPU6050_CYCLE_20_HZ = 2,
    MPU6050_CYCLE_40_HZ = 3,
} mpu6050_cycle_rate_t;

typedef enum {
    MPU6050_INTR_ACTIVE_HIGH = 0,
    MPU6050_INTR_ACTIVE_LOW = 1,
} mpu6050_intr_active_t;

#define MPU6050_INTR_PIN_ACTIVE_HIGH MPU6050_INTR_ACTIVE_HIGH
#define MPU6050_INTR_PIN_ACTIVE_LOW MPU6050_INTR_ACTIVE_LOW

enum class MockMPU6050Mode {
    CONSTANT,
    SINE_WAVE,
    PLAYBACK_VECTOR,
    FAULT_SPIKE
};

class Adafruit_MPU6050 {
private:
    static inline MockMPU6050Mode mode = MockMPU6050Mode::SINE_WAVE;
    static inline float constant_x = 0.0f;
    static inline float constant_y = 0.0f;
    static inline float constant_z = 9.81f;
    static inline float sine_freq = 25.0f;
    static inline float sine_amp = 0.85f;
    static inline float dc_offset = 9.81f;
    static inline float noise_amplitude = 0.05f;
    static inline std::vector<float> playback_samples;
    static inline size_t playback_index = 0;
    static inline float fault_spike_value = 15.0f;
    static inline size_t fault_spike_duration = 0;
    static inline size_t fault_spike_count = 0;
    static inline bool init_success = true;
    static inline mpu6050_accel_range_t accel_range = MPU6050_RANGE_8_G;
    static inline mpu6050_gyro_range_t gyro_range = MPU6050_RANGE_500_DEG;
    static inline mpu6050_bandwidth_t filter_bandwidth = MPU6050_BAND_21_HZ;
    static inline mpu6050_highpass_t highpass_filter = MPU6050_HIGHPASS_DISABLE;

public:
    Adafruit_MPU6050() = default;

    bool begin(uint8_t i2c_addr = MPU6050_I2CADDR_DEFAULT, TwoWire *wire = &Wire, int32_t sensor_id = 0) {
        (void)i2c_addr;
        (void)wire;
        (void)sensor_id;
        return init_success;
    }

    void setAccelerometerRange(mpu6050_accel_range_t new_range) {
        accel_range = new_range;
    }

    void setAccelerometerRange(uint8_t new_range) {
        accel_range = static_cast<mpu6050_accel_range_t>(new_range);
    }

    mpu6050_accel_range_t getAccelerometerRange() const {
        return accel_range;
    }

    void setGyroRange(mpu6050_gyro_range_t new_range) {
        gyro_range = new_range;
    }

    void setGyroRange(uint8_t new_range) {
        gyro_range = static_cast<mpu6050_gyro_range_t>(new_range);
    }

    mpu6050_gyro_range_t getGyroRange() const {
        return gyro_range;
    }

    void setFilterBandwidth(mpu6050_bandwidth_t new_bandwidth) {
        filter_bandwidth = new_bandwidth;
    }

    void setFilterBandwidth(uint8_t new_bandwidth) {
        filter_bandwidth = static_cast<mpu6050_bandwidth_t>(new_bandwidth);
    }

    mpu6050_bandwidth_t getFilterBandwidth() const {
        return filter_bandwidth;
    }

    void setHighPassFilter(mpu6050_highpass_t filter) {
        highpass_filter = filter;
    }

    mpu6050_highpass_t getHighPassFilter() const {
        return highpass_filter;
    }

    void setMotionDetectionThreshold(uint8_t) {}
    void setMotionDetectionDuration(uint8_t) {}
    void setInterruptPinLatch(bool) {}
    void setInterruptPinPolarity(bool) {}
    void setMotionInterrupt(bool) {}

    // Mock Control & Injection Methods
    void set_init_success(bool success) {
        init_success = success;
    }

    bool get_init_success() const {
        return init_success;
    }

    void set_constant(float z, float x = 0.0f, float y = 0.0f) {
        mode = MockMPU6050Mode::CONSTANT;
        constant_z = z;
        constant_x = x;
        constant_y = y;
    }

    void set_sine(float freq, float amp, float offset = 9.81f, float noise = 0.05f) {
        mode = MockMPU6050Mode::SINE_WAVE;
        sine_freq = freq;
        sine_amp = amp;
        dc_offset = offset;
        noise_amplitude = noise;
    }

    void set_playback(const std::vector<float> &samples) {
        mode = MockMPU6050Mode::PLAYBACK_VECTOR;
        playback_samples = samples;
        playback_index = 0;
    }

    void set_fault_spike(float spike_val, size_t duration_samples = 1) {
        mode = MockMPU6050Mode::FAULT_SPIKE;
        fault_spike_value = spike_val;
        fault_spike_duration = duration_samples;
        fault_spike_count = 0;
    }

    MockMPU6050Mode get_mode() const {
        return mode;
    }

    void reset() {
        mode = MockMPU6050Mode::SINE_WAVE;
        constant_x = 0.0f;
        constant_y = 0.0f;
        constant_z = 9.81f;
        sine_freq = 25.0f;
        sine_amp = 0.85f;
        dc_offset = 9.81f;
        noise_amplitude = 0.05f;
        playback_samples.clear();
        playback_index = 0;
        fault_spike_duration = 0;
        fault_spike_count = 0;
        init_success = true;
        accel_range = MPU6050_RANGE_8_G;
        gyro_range = MPU6050_RANGE_500_DEG;
        filter_bandwidth = MPU6050_BAND_21_HZ;
        highpass_filter = MPU6050_HIGHPASS_DISABLE;
    }

    bool getEvent(sensors_event_t *accel, sensors_event_t *gyro = nullptr, sensors_event_t *temp = nullptr) {
        if (!init_success) return false;

        float z_val = dc_offset;
        float x_val = constant_x;
        float y_val = constant_y;

        if (mode == MockMPU6050Mode::CONSTANT) {
            z_val = constant_z;
        } else if (mode == MockMPU6050Mode::SINE_WAVE) {
            float t = static_cast<float>(micros()) / 1000000.0f;
            z_val = dc_offset + sine_amp * std::sin(2.0f * 3.14159265358979323846f * sine_freq * t);
            if (noise_amplitude > 0.0f) {
                float pseudo_noise = (((micros() % 1000) / 500.0f) - 1.0f) * noise_amplitude;
                z_val += pseudo_noise;
            }
        } else if (mode == MockMPU6050Mode::PLAYBACK_VECTOR) {
            if (!playback_samples.empty()) {
                z_val = playback_samples[playback_index % playback_samples.size()];
                playback_index++;
            }
        } else if (mode == MockMPU6050Mode::FAULT_SPIKE) {
            if (fault_spike_count < fault_spike_duration) {
                z_val = fault_spike_value;
                fault_spike_count++;
            } else {
                float t = static_cast<float>(micros()) / 1000000.0f;
                z_val = dc_offset + sine_amp * std::sin(2.0f * 3.14159265358979323846f * sine_freq * t);
            }
        }

        if (accel) {
            accel->version = sizeof(sensors_event_t);
            accel->sensor_id = 1;
            accel->type = SENSOR_TYPE_ACCELEROMETER;
            accel->timestamp = millis();
            accel->acceleration.x = x_val;
            accel->acceleration.y = y_val;
            accel->acceleration.z = z_val;
            accel->acceleration.status = 0;
        }
        if (gyro) {
            gyro->version = sizeof(sensors_event_t);
            gyro->sensor_id = 2;
            gyro->type = SENSOR_TYPE_GYROSCOPE;
            gyro->timestamp = millis();
            gyro->gyro.x = 0.0f;
            gyro->gyro.y = 0.0f;
            gyro->gyro.z = 0.0f;
            gyro->gyro.status = 0;
        }
        if (temp) {
            temp->version = sizeof(sensors_event_t);
            temp->sensor_id = 3;
            temp->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
            temp->timestamp = millis();
            temp->temperature = 25.0f;
        }
        return true;
    }
};

inline Adafruit_MPU6050 mock_mpu6050;
