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
#include <chrono>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =============================================================================
// TIER 4: REAL-WORLD INDUSTRIAL WORKLOAD & LIFECYCLE TEST SUITE (>=5 TESTS)
// =============================================================================

TEST(Tier4_Workloads, T4_01_Continuous_1000Cycle_IndustrialStability) {
    MockClock::reset();
    MockGPIO::reset();
    Sensors_Init();
    Actuator_Init();
    Actuator_Rearm();

    mock_mpu6050.set_sine(25.0f, 0.85f, 9.81f, 0.02f);
    mock_dht.set_temperature(28.5f);

    DSPPipeline dsp;

    auto start_time = std::chrono::high_resolution_clock::now();

    for (int cycle = 0; cycle < 1000; ++cycle) {
        dsp.reset();
        for (int s = 0; s < 64; ++s) {
            MockClock::advance_us(5000);
            float z = 0.0f;
            Sensors_SampleMPU(z);
            dsp.pushSample(z);
        }

        float temp = 0.0f;
        Sensors_SampleDHT(temp);

        DSPFeatures feat = dsp.computeFeatures(temp);
        float feat_arr[4];
        feat.toFeatureArray(feat_arr);
        int pred = model_predict(feat_arr);

        ASSERT_EQ(0, pred);
        Actuator_UpdateFSM(pred);

        ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
        ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    // 1000 cycles on host should complete in < 2000 ms
    ASSERT_TRUE(total_ms < 2000);
}

TEST(Tier4_Workloads, T4_02_Full_7Phase_Industrial_Lifecycle) {
    MockClock::reset();
    MockGPIO::reset();
    Serial.reset();

    // -------------------------------------------------------------------------
    // Phase 1: Boot & Initialization (Cycles 0 - 50)
    // -------------------------------------------------------------------------
    Sensors_Init();
    Actuator_Init();
    ASSERT_EQ(STATE_INIT, Actuator_GetState());
    ASSERT_EQ(0u, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);

    // Operator starts system
    Actuator_Rearm();
    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);

    DSPPipeline dsp;

    // -------------------------------------------------------------------------
    // Phase 2: Steady Nominal Operation (Cycles 51 - 300)
    // -------------------------------------------------------------------------
    mock_mpu6050.set_sine(25.0f, 0.85f, 9.81f, 0.02f);
    mock_dht.set_temperature(28.5f);

    for (int c = 51; c <= 300; ++c) {
        dsp.reset();
        for (int s = 0; s < 64; ++s) {
            MockClock::advance_us(5000);
            float z = 0.0f;
            Sensors_SampleMPU(z);
            dsp.pushSample(z);
        }

        float temp = 0.0f;
        Sensors_SampleDHT(temp);
        DSPFeatures feat = dsp.computeFeatures(temp);

        float feat_arr[4];
        feat.toFeatureArray(feat_arr);
        int pred = model_predict(feat_arr);

        ASSERT_EQ(0, pred);
        Actuator_UpdateFSM(pred);

        ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
        ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_LED_WARN_ALARM]);
        ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_BUZZER]);
        ASSERT_EQ(MOTOR_NOMINAL_PWM, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
    }

    // -------------------------------------------------------------------------
    // Phase 3: Progressive Bearing Wear Degradation (Cycles 301 - 500)
    // -------------------------------------------------------------------------
    for (int c = 301; c <= 500; ++c) {
        float wear = static_cast<float>(c - 300) / 200.0f; // 0.0 to 1.0
        float amp = 0.85f + wear * 0.30f; // moderate rise
        mock_mpu6050.set_sine(25.0f, amp, 9.81f, 0.05f);
        mock_dht.set_temperature(28.5f + wear * 3.0f); // slight temperature drift

        dsp.reset();
        for (int s = 0; s < 64; ++s) {
            MockClock::advance_us(5000);
            float z = 0.0f;
            Sensors_SampleMPU(z);
            dsp.pushSample(z);
        }

        float temp = 0.0f;
        Sensors_SampleDHT(temp);
        DSPFeatures feat = dsp.computeFeatures(temp);

        float feat_arr[4];
        feat.toFeatureArray(feat_arr);
        int pred = model_predict(feat_arr);
        Actuator_UpdateFSM(pred);

        // State remains safe
        ASSERT_TRUE(Actuator_GetState() == STATE_NORMAL || Actuator_GetState() == STATE_WARNING);
    }

    // -------------------------------------------------------------------------
    // Phase 4: Critical Failure Anomaly Trigger (Cycles 501 - 503)
    // -------------------------------------------------------------------------
    // Critical vibration fault (65 Hz, 3.5 m/s^2) & critical temperature (80.0 °C)
    mock_mpu6050.set_sine(65.0f, 3.5f, 9.81f, 0.1f);
    mock_dht.set_temperature(80.0f);
    MockClock::advance_ms(2500);

    // Frame 501: Debounce count = 1 -> WARNING
    {
        dsp.reset();
        for (int s = 0; s < 64; ++s) {
            MockClock::advance_us(5000);
            float z = 0.0f;
            Sensors_SampleMPU(z);
            dsp.pushSample(z);
        }
        float temp = 0.0f;
        Sensors_SampleDHT(temp);
        DSPFeatures feat = dsp.computeFeatures(temp);
        float feat_arr[4];
        feat.toFeatureArray(feat_arr);
        int pred = model_predict(feat_arr);
        ASSERT_EQ(1, pred);
        Actuator_UpdateFSM(pred);
        ASSERT_EQ(STATE_WARNING, Actuator_GetState());
        ASSERT_EQ(1u, Actuator_GetDebounceCount());
    }

    // Frame 502: Debounce count = 2 -> WARNING
    {
        dsp.reset();
        for (int s = 0; s < 64; ++s) {
            MockClock::advance_us(5000);
            float z = 0.0f;
            Sensors_SampleMPU(z);
            dsp.pushSample(z);
        }
        float temp = 0.0f;
        Sensors_SampleDHT(temp);
        DSPFeatures feat = dsp.computeFeatures(temp);
        float feat_arr[4];
        feat.toFeatureArray(feat_arr);
        int pred = model_predict(feat_arr);
        ASSERT_EQ(1, pred);
        Actuator_UpdateFSM(pred);
        ASSERT_EQ(STATE_WARNING, Actuator_GetState());
        ASSERT_EQ(2u, Actuator_GetDebounceCount());
    }

    // Frame 503: Debounce count = 3 -> TRIP TO ALARM!
    {
        dsp.reset();
        for (int s = 0; s < 64; ++s) {
            MockClock::advance_us(5000);
            float z = 0.0f;
            Sensors_SampleMPU(z);
            dsp.pushSample(z);
        }
        float temp = 0.0f;
        Sensors_SampleDHT(temp);
        DSPFeatures feat = dsp.computeFeatures(temp);
        float feat_arr[4];
        feat.toFeatureArray(feat_arr);
        int pred = model_predict(feat_arr);
        ASSERT_EQ(1, pred);
        Actuator_UpdateFSM(pred);
        ASSERT_EQ(STATE_ALARM, Actuator_GetState());
    }

    // -------------------------------------------------------------------------
    // Phase 5: Emergency Safety Cut Verification (Cycles 504 - 600)
    // -------------------------------------------------------------------------
    ASSERT_EQ(0u, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_IN1]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_IN2]);
    ASSERT_EQ(HIGH, MockGPIO::pin_state[PIN_LED_WARN_ALARM]);
    ASSERT_EQ(HIGH, MockGPIO::pin_state[PIN_BUZZER]);

    for (int c = 504; c <= 600; ++c) {
        Actuator_UpdateFSM(1);
        ASSERT_EQ(STATE_ALARM, Actuator_GetState());
        ASSERT_EQ(0u, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
    }

    // -------------------------------------------------------------------------
    // Phase 6: Post-Trip Idle / Inspection (Cycles 601 - 700)
    // -------------------------------------------------------------------------
    // Sensor physically returns to normal baseline, but ALARM must stay latched!
    mock_mpu6050.set_sine(25.0f, 0.85f, 9.81f, 0.02f);
    mock_dht.set_temperature(25.0f);
    MockClock::advance_ms(2500);

    for (int c = 601; c <= 700; ++c) {
        dsp.reset();
        for (int s = 0; s < 64; ++s) {
            MockClock::advance_us(5000);
            float z = 0.0f;
            Sensors_SampleMPU(z);
            dsp.pushSample(z);
        }
        float temp = 0.0f;
        Sensors_SampleDHT(temp);
        DSPFeatures feat = dsp.computeFeatures(temp);
        float feat_arr[4];
        feat.toFeatureArray(feat_arr);
        int pred = model_predict(feat_arr);
        // Even if prediction is 0 (normal), ALARM remains latched
        Actuator_UpdateFSM(pred);

        ASSERT_EQ(STATE_ALARM, Actuator_GetState());
        ASSERT_EQ(0u, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
    }

    // -------------------------------------------------------------------------
    // Phase 7: Operator Manual RESET & Healthy Recovery (Cycles 701 - 1000)
    // -------------------------------------------------------------------------
    Serial.inject_input("RESET\n");
    Actuator_ProcessSerial();

    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_LED_WARN_ALARM]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_BUZZER]);
    ASSERT_EQ(HIGH, MockGPIO::pin_state[PIN_LED_NORMAL]);

    for (int c = 701; c <= 1000; ++c) {
        dsp.reset();
        for (int s = 0; s < 64; ++s) {
            MockClock::advance_us(5000);
            float z = 0.0f;
            Sensors_SampleMPU(z);
            dsp.pushSample(z);
        }
        float temp = 0.0f;
        Sensors_SampleDHT(temp);
        DSPFeatures feat = dsp.computeFeatures(temp);
        float feat_arr[4];
        feat.toFeatureArray(feat_arr);
        int pred = model_predict(feat_arr);
        Actuator_UpdateFSM(pred);

        ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
        ASSERT_EQ(MOTOR_NOMINAL_PWM, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
    }
}

TEST(Tier4_Workloads, T4_03_RotorJam_BearingSeizure_Workload) {
    MockClock::reset();
    MockGPIO::reset();
    Sensors_Init();
    Actuator_Init();
    Actuator_Rearm();

    // Normal run for 10 cycles
    DSPPipeline dsp;
    mock_mpu6050.set_sine(25.0f, 0.85f, 9.81f, 0.02f);
    mock_dht.set_temperature(28.5f);

    for (int i = 0; i < 10; ++i) {
        dsp.reset();
        for (int s = 0; s < 64; ++s) {
            MockClock::advance_us(5000);
            float z = 0.0f;
            Sensors_SampleMPU(z);
            dsp.pushSample(z);
        }
        float temp = 0.0f;
        Sensors_SampleDHT(temp);
        DSPFeatures f = dsp.computeFeatures(temp);
        float feat_arr[4];
        f.toFeatureArray(feat_arr);
        Actuator_UpdateFSM(model_predict(feat_arr));
    }
    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());

    // Sudden mechanical rotor seizure: massive high frequency vibration spike
    mock_mpu6050.set_sine(65.0f, 5.0f, 9.81f, 0.2f);
    for (int trip_frame = 0; trip_frame < 3; ++trip_frame) {
        dsp.reset();
        for (int s = 0; s < 64; ++s) {
            MockClock::advance_us(5000);
            float z = 0.0f;
            Sensors_SampleMPU(z);
            dsp.pushSample(z);
        }
        float temp = 0.0f;
        Sensors_SampleDHT(temp);
        DSPFeatures f = dsp.computeFeatures(temp);
        float feat_arr[4];
        f.toFeatureArray(feat_arr);
        Actuator_UpdateFSM(model_predict(feat_arr));
    }

    ASSERT_EQ(STATE_ALARM, Actuator_GetState());
    ASSERT_EQ(0u, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
}

TEST(Tier4_Workloads, T4_04_ThermalRunaway_UnderContinuousLoad) {
    MockClock::reset();
    MockGPIO::reset();
    Sensors_Init();
    Actuator_Init();
    Actuator_Rearm();

    // Friction vibration with critical thermal runaway (80.0 °C)
    mock_mpu6050.set_sine(35.0f, 1.8f, 9.81f, 0.05f);
    mock_dht.set_temperature(80.0f);
    MockClock::advance_ms(2500);

    DSPPipeline dsp;
    for (int frame = 0; frame < 3; ++frame) {
        dsp.reset();
        for (int s = 0; s < 64; ++s) {
            MockClock::advance_us(5000);
            float z = 0.0f;
            Sensors_SampleMPU(z);
            dsp.pushSample(z);
        }
        float temp = 0.0f;
        Sensors_SampleDHT(temp);
        DSPFeatures f = dsp.computeFeatures(temp);
        float feat_arr[4];
        f.toFeatureArray(feat_arr);
        int pred = model_predict(feat_arr);
        ASSERT_EQ(1, pred);
        Actuator_UpdateFSM(pred);
    }

    ASSERT_EQ(STATE_ALARM, Actuator_GetState());
    ASSERT_EQ(0u, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
    ASSERT_TRUE(Actuator_IsBuzzerActive());
}

TEST(Tier4_Workloads, T4_05_IntermittentSensorFlake_And_Recovery) {
    MockClock::reset();
    MockGPIO::reset();
    Sensors_Init();
    Actuator_Init();
    Actuator_Rearm();

    DSPPipeline dsp;
    mock_mpu6050.set_sine(25.0f, 0.50f, 9.81f, 0.02f);

    for (int cycle = 0; cycle < 100; ++cycle) {
        // Every 20 cycles, introduce 1 temporary NaN sensor read
        if (cycle % 20 == 0) {
            mock_dht.set_nan_failure(true);
        } else {
            mock_dht.set_nan_failure(false);
            mock_dht.set_temperature(28.0f);
        }
        MockClock::advance_ms(2500);

        dsp.reset();
        for (int s = 0; s < 64; ++s) {
            MockClock::advance_us(5000);
            float z = 0.0f;
            Sensors_SampleMPU(z);
            dsp.pushSample(z);
        }

        float temp = 0.0f;
        Sensors_SampleDHT(temp);
        DSPFeatures f = dsp.computeFeatures(temp);
        float feat_arr[4];
        f.toFeatureArray(feat_arr);
        int pred = model_predict(feat_arr);
        Actuator_UpdateFSM(pred);

        ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
        ASSERT_EQ(MOTOR_NOMINAL_PWM, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
    }
}
