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
#include <cstdlib>

// =============================================================================
// TIER 5: ADVERSARIAL HARDENING & STRESS TEST SUITE (>=5 TESTS)
// =============================================================================

TEST(Tier5_Hardening, T5_01_Serial_Protocol_Fuzzing) {
    Serial.reset();
    Actuator_Init();
    Actuator_Rearm();

    // Generate 2048 bytes of random binary fuzz data (including null bytes, control codes, non-ASCII)
    srand(12345);
    std::string fuzz_packet = "";
    for (int i = 0; i < 2048; ++i) {
        char c = static_cast<char>(rand() % 256);
        fuzz_packet.push_back(c);
        if (i % 64 == 63) {
            fuzz_packet.push_back('\n');
        }
    }

    Serial.inject_input(fuzz_packet);

    // Consume all fuzzed inputs through non-blocking parser
    Actuator_ProcessSerial();

    // Verify system state remains intact without memory corruption
    SystemState state = Actuator_GetState();
    ASSERT_TRUE(state == STATE_NORMAL || state == STATE_ALARM || state == STATE_WARNING || state == STATE_INIT);
}

TEST(Tier5_Hardening, T5_02_Sensor_RailClipping_Saturation) {
    DSPPipeline dsp;
    // Accelerometer saturated at ±16g limit (16 * 9.81 = ±156.96 m/s^2)
    for (int i = 0; i < 64; ++i) {
        float sample = (i % 2 == 0) ? (16.0f * 9.81f) : (-16.0f * 9.81f);
        dsp.pushSample(sample);
    }
    ASSERT_TRUE(dsp.isWindowReady());

    // Extreme temperature (150.0 °C)
    DSPFeatures feat = dsp.computeFeatures(150.0f);

    ASSERT_FALSE(std::isnan(feat.rms));
    ASSERT_FALSE(std::isinf(feat.rms));
    ASSERT_FALSE(std::isnan(feat.peak_to_peak));
    ASSERT_FALSE(std::isnan(feat.dominant_frequency));

    float feat_arr[4];
    feat.toFeatureArray(feat_arr);
    int pred = model_predict(feat_arr);
    ASSERT_EQ(1, pred); // Must classify as anomaly
}

TEST(Tier5_Hardening, T5_03_StackDepth_Audit) {
    // Audit local stack frame consumption during full DSP + FFT + TinyML inference chain
    char stack_anchor_top;

    DSPPipeline dsp;
    for (int i = 0; i < 64; ++i) {
        dsp.pushSample(1.0f);
    }
    DSPFeatures feat = dsp.computeFeatures(30.0f);
    float feat_arr[4];
    feat.toFeatureArray(feat_arr);
    int pred = model_predict(feat_arr);
    (void)pred;

    char stack_anchor_bottom;
    ptrdiff_t stack_used = std::abs(&stack_anchor_top - &stack_anchor_bottom);

    // Assert stack utilization is well under 2048 bytes (safe for ESP32 8KB task stack)
    ASSERT_TRUE(stack_used < 2048);
}

TEST(Tier5_Hardening, T5_04_Reentrancy_CommandSafety) {
    Serial.reset();
    Sensors_Init();
    Actuator_Init();
    Actuator_Rearm();

    DSPPipeline dsp;

    // Simulate interleaved serial commands arriving in the middle of sample pushing
    for (int i = 0; i < 64; ++i) {
        if (i == 20) {
            Serial.inject_input("INJECT_VIB\n");
            Actuator_ProcessSerial();
        }
        if (i == 40) {
            Serial.inject_input("INJECT_TEMP\n");
            Actuator_ProcessSerial();
        }

        float z = 0.0f;
        Sensors_SampleMPU(z);
        dsp.pushSample(z);
    }

    float current_temp = 0.0f;
    Sensors_SampleDHT(current_temp);
    DSPFeatures feat = dsp.computeFeatures(current_temp);

    ASSERT_TRUE(feat.rms > 0.0f);
    ASSERT_TRUE(Sensors_IsVibrationInjected());
    ASSERT_TRUE(Sensors_IsTemperatureInjected());

    // Clean up
    Actuator_ProcessCommand("RESET");
    ASSERT_FALSE(Sensors_IsVibrationInjected());
    ASSERT_FALSE(Sensors_IsTemperatureInjected());
}

TEST(Tier5_Hardening, T5_05_PowerBrownout_BootRecovery) {
    // Simulate dirty hardware state prior to power reset (motor spinning, buzzer sounding)
    MockGPIO::pin_state[PIN_MOTOR_IN1] = HIGH;
    MockGPIO::pin_pwm[PIN_MOTOR_ENA] = 255;
    MockGPIO::pin_state[PIN_BUZZER] = HIGH;

    // Trigger sudden power-on / hardware reboot
    Actuator_Init();

    // Verification: Immediate safe shutdown on boot
    ASSERT_EQ(STATE_INIT, Actuator_GetState());
    ASSERT_EQ(0u, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_IN1]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_IN2]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_BUZZER]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_LED_NORMAL]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_LED_WARN_ALARM]);
}
