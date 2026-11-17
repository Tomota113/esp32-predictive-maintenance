#include "test_framework.h"
#include "mocks/Arduino.h"
#include "mocks/Wire.h"
#include "mocks/Adafruit_Sensor.h"
#include "mocks/Adafruit_MPU6050.h"
#include "mocks/DHT.h"
#include "mocks/arduinoFFT.h"
#include "config.h"
#include "sensors.h"
#include "dsp_pipeline.h"
#include "actuator_control.h"
#include "model.h"
#include <cmath>
#include <vector>
#include <string>

// =============================================================================
// TIER 3: CROSS-FEATURE COMBINATIONS & RACE CONDITION TEST SUITE (>=5 TESTS)
// =============================================================================

TEST(Tier3_Combos, T3_01_Simultaneous_VibrationAndThermalFault) {
    MockClock::reset();
    MockGPIO::reset();
    Sensors_Init();
    Actuator_Init();
    Actuator_Rearm();

    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());

    // Simultaneously inject both vibration fault and thermal drift
    Sensors_SetVibrationInjection(true);
    Sensors_SetTemperatureInjection(true);

    DSPPipeline dsp;

    // Simulate 3 full DSP windows (64 samples each)
    for (int frame = 0; frame < 3; ++frame) {
        dsp.reset();
        for (int s = 0; s < 64; ++s) {
            float z = 0.0f;
            Sensors_SampleMPU(z);
            dsp.pushSample(z);
        }

        float current_temp = 0.0f;
        Sensors_SampleDHT(current_temp);

        DSPFeatures feat = dsp.computeFeatures(current_temp);
        float feat_arr[4];
        feat.toFeatureArray(feat_arr);
        int pred = model_predict(feat_arr);

        ASSERT_EQ(1, pred); // Must classify as anomaly
        Actuator_UpdateFSM(pred);
    }

    // 3 consecutive anomaly frames -> STATE_ALARM
    ASSERT_EQ(STATE_ALARM, Actuator_GetState());
    ASSERT_EQ(0u, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_IN1]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_IN2]);
    ASSERT_TRUE(Actuator_IsBuzzerActive());
    ASSERT_TRUE(Actuator_IsWarningAlarmLedActive());
}

TEST(Tier3_Combos, T3_02_RaceInterleaving_InjectAndReset) {
    MockClock::reset();
    MockGPIO::reset();
    Sensors_Init();
    Actuator_Init();
    Actuator_Rearm();

    for (int cycle = 0; cycle < 20; ++cycle) {
        Serial.inject_input("INJECT_VIB\n");
        Actuator_ProcessSerial();
        ASSERT_TRUE(Sensors_IsVibrationInjected());

        Actuator_UpdateFSM(1);

        Serial.inject_input("RESET\n");
        Actuator_ProcessSerial();
        ASSERT_FALSE(Sensors_IsVibrationInjected());
        ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
        ASSERT_EQ(0u, Actuator_GetDebounceCount());
    }

    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());
}

TEST(Tier3_Combos, T3_03_SamplingJitter_BurstArrival) {
    MockClock::reset();
    Sensors_Init();
    DSPPipeline dsp;

    // Simulate irregular hardware timer jitter (intervals: 4ms, 6ms, 5ms, 7ms, 3ms)
    uint32_t jitter_intervals_us[] = {4000, 6000, 5000, 7000, 3000};

    for (int i = 0; i < 64; ++i) {
        MockClock::advance_us(jitter_intervals_us[i % 5]);
        float z = 0.0f;
        bool ok = Sensors_SampleMPU(z);
        ASSERT_TRUE(ok);
        dsp.pushSample(z);
    }

    ASSERT_TRUE(dsp.isWindowReady());
    DSPFeatures f = dsp.computeFeatures(25.0f);
    ASSERT_FALSE(std::isnan(f.rms));
    ASSERT_FALSE(std::isnan(f.dominant_frequency));
}

TEST(Tier3_Combos, T3_04_AlternatingAnomaly_DebounceReset) {
    MockGPIO::reset();
    Actuator_Init();
    Actuator_Rearm();

    // Sequence: 1, 0, 1, 0, 1, 0, 1, 0 ... (anomaly never persists 3 consecutive frames)
    for (int i = 0; i < 20; ++i) {
        // Anomaly frame
        Actuator_UpdateFSM(1);
        ASSERT_EQ(STATE_WARNING, Actuator_GetState());
        ASSERT_EQ(1u, Actuator_GetDebounceCount());
        ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM()); // Motor must NOT stop

        // Normal frame
        Actuator_UpdateFSM(0);
        ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
        ASSERT_EQ(0u, Actuator_GetDebounceCount());
        ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());
    }

    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
}

TEST(Tier3_Combos, T3_05_SensorI2CGlitch_SafeFallback) {
    MockClock::reset();
    Sensors_Init();
    Sensors_Calibrate(20);

    float z_val = 0.0f;
    bool ok_before = Sensors_SampleMPU(z_val);
    ASSERT_TRUE(ok_before);
    ASSERT_TRUE(Sensors_IsMPUHealthy());

    // Inject temporary I2C bus disconnection
    mock_mpu6050.set_init_success(false);

    float z_glitch = 0.0f;
    bool ok_glitch = Sensors_SampleMPU(z_glitch);
    ASSERT_FALSE(ok_glitch);
    ASSERT_FALSE(Sensors_IsMPUHealthy());
    // Safe fallback: last raw value returned without crashing
    ASSERT_FALSE(std::isnan(z_glitch));

    // I2C bus reconnects
    mock_mpu6050.set_init_success(true);
    float z_recovered = 0.0f;
    bool ok_recovered = Sensors_SampleMPU(z_recovered);
    ASSERT_TRUE(ok_recovered);
    ASSERT_TRUE(Sensors_IsMPUHealthy());
}

TEST(Tier3_Combos, T3_06_DebounceInterleaving_TwoAnomaliesThenNormal) {
    MockGPIO::reset();
    Actuator_Init();
    Actuator_Rearm();

    for (int round = 0; round < 5; ++round) {
        // 2 anomaly frames -> reaches warning, count = 2
        Actuator_UpdateFSM(1);
        ASSERT_EQ(STATE_WARNING, Actuator_GetState());
        ASSERT_EQ(1u, Actuator_GetDebounceCount());

        Actuator_UpdateFSM(1);
        ASSERT_EQ(STATE_WARNING, Actuator_GetState());
        ASSERT_EQ(2u, Actuator_GetDebounceCount());

        // 1 normal frame -> resets count = 0, state returns to NORMAL
        Actuator_UpdateFSM(0);
        ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
        ASSERT_EQ(0u, Actuator_GetDebounceCount());
        ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());
    }

    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
}
