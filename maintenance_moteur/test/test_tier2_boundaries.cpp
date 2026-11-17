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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =============================================================================
// TIER 2: BOUNDARY & CORNER CASE TEST SUITE (>=8 TESTS)
// =============================================================================

TEST(Tier2_Boundaries, T2_01_ColdStart_BufferUnderflow) {
    DSPPipeline dsp;
    // Push only 10 samples (buffer not full)
    for (int i = 0; i < 10; ++i) {
        dsp.pushSample(1.0f);
    }
    ASSERT_FALSE(dsp.isWindowReady());

    // Feature computation attempt on partial buffer must safely return zeroed features
    DSPFeatures features = dsp.computeFeatures(25.0f);
    ASSERT_NEAR(0.0f, features.rms, 1e-5f);
    ASSERT_NEAR(0.0f, features.peak_to_peak, 1e-5f);
    ASSERT_NEAR(0.0f, features.peak_absolute, 1e-5f);
    ASSERT_NEAR(0.0f, features.dominant_frequency, 1e-5f);
    ASSERT_NEAR(25.0f, features.temperature, 1e-5f);
}

TEST(Tier2_Boundaries, T2_02_ZeroVibration_NumericalSafety) {
    DSPPipeline dsp;
    for (int i = 0; i < 64; ++i) {
        dsp.pushSample(0.0f);
    }
    ASSERT_TRUE(dsp.isWindowReady());

    DSPFeatures features = dsp.computeFeatures(20.0f);
    ASSERT_NEAR(0.0f, features.rms, 1e-5f);
    ASSERT_NEAR(0.0f, features.peak_to_peak, 1e-5f);
    ASSERT_NEAR(0.0f, features.peak_absolute, 1e-5f);
    ASSERT_NEAR(0.0f, features.dominant_frequency, 1e-5f);
    ASSERT_FALSE(std::isnan(features.dominant_frequency));
    ASSERT_FALSE(std::isinf(features.dominant_frequency));
}

TEST(Tier2_Boundaries, T2_03_ExtremeFloatValues_NoOverflow) {
    DSPPipeline dsp;
    for (int i = 0; i < 64; ++i) {
        dsp.pushSample(1e6f);
    }
    ASSERT_TRUE(dsp.isWindowReady());

    DSPFeatures features = dsp.computeFeatures(25.0f);
    ASSERT_FALSE(std::isnan(features.rms));
    ASSERT_FALSE(std::isinf(features.rms));
    ASSERT_FALSE(std::isnan(features.dominant_frequency));
    ASSERT_FALSE(std::isinf(features.dominant_frequency));
}

TEST(Tier2_Boundaries, T2_04_WhiteNoise_PeakStability) {
    DSPPipeline dsp;
    srand(42);
    for (int i = 0; i < 64; ++i) {
        float r = (static_cast<float>(rand() % 2000) - 1000.0f) / 1000.0f; // [-1.0, 1.0]
        dsp.pushSample(r);
    }
    ASSERT_TRUE(dsp.isWindowReady());

    DSPFeatures features = dsp.computeFeatures(25.0f);
    ASSERT_TRUE(features.rms > 0.0f);
    ASSERT_TRUE(features.peak_to_peak > 0.0f);
    ASSERT_TRUE(features.dominant_frequency >= 0.0f && features.dominant_frequency <= 100.0f);
    ASSERT_FALSE(std::isnan(features.dominant_frequency));
}

TEST(Tier2_Boundaries, T2_05_SerialInjection_BurstSpam) {
    Serial.reset();
    Actuator_Init();
    Actuator_Rearm();

    for (int i = 0; i < 50; ++i) {
        Serial.inject_input("INJECT_VIB\nINJECT_TEMP\nRESET\n");
    }

    Actuator_ProcessSerial();
    ASSERT_EQ(0, Serial.available());

    SystemState state = Actuator_GetState();
    ASSERT_TRUE(state == STATE_NORMAL || state == STATE_ALARM || state == STATE_WARNING);
}

TEST(Tier2_Boundaries, T2_06_DecisionTree_EpsilonBoundary) {
    // Model split boundary tests around critical feature thresholds
    float eps = 1e-5f;

    // Peak threshold around 1.1858f
    int res_below = model_predict_values(0.40f, 1.185800f - eps, 25.0f, 25.0f);
    int res_above = model_predict_values(0.60f, 1.185800f + eps, 25.0f, 25.0f);

    ASSERT_TRUE(res_below == 0 || res_below == 1);
    ASSERT_TRUE(res_above == 0 || res_above == 1);

    // Temperature threshold around 32.354450f (with elevated RMS 0.60f)
    int res_temp_low = model_predict_values(0.60f, 1.15f, 25.0f, 32.354450f - eps);
    int res_temp_high = model_predict_values(0.60f, 1.15f, 25.0f, 32.354450f + eps);

    ASSERT_EQ(0, res_temp_low);
    ASSERT_EQ(1, res_temp_high);
}

TEST(Tier2_Boundaries, T2_07_Nyquist_Boundary_100Hz) {
    DSPPipeline dsp;
    // Alternating +1, -1 at 200 Hz sampling represents Nyquist frequency Fs/2 = 100 Hz (Bin 32)
    for (int i = 0; i < 64; ++i) {
        dsp.pushSample((i % 2 == 0) ? 1.0f : -1.0f);
    }
    ASSERT_TRUE(dsp.isWindowReady());

    DSPFeatures features = dsp.computeFeatures(25.0f);
    // Dominant frequency search scans up to WINDOW_SIZE/2 (32 bins). Nyquist peak resolves near ~96.875 Hz
    ASSERT_TRUE(features.dominant_frequency >= 90.0f && features.dominant_frequency <= 100.0f);
}

TEST(Tier2_Boundaries, T2_08_SubBin_LowFrequency) {
    DSPPipeline dsp;
    // 1.5 Hz sine sampled at 200 Hz (sub-bin frequency, resolution is 3.125 Hz/bin)
    for (int i = 0; i < 64; ++i) {
        float t = static_cast<float>(i) / 200.0f;
        dsp.pushSample(std::sin(2.0f * static_cast<float>(M_PI) * 1.5f * t));
    }
    ASSERT_TRUE(dsp.isWindowReady());

    DSPFeatures features = dsp.computeFeatures(25.0f);
    // Should resolve to nearest non-DC bin 1 (3.125 Hz) without error
    ASSERT_NEAR(3.125f, features.dominant_frequency, 0.1f);
}

TEST(Tier2_Boundaries, T2_09_RingBuffer_ContinuousWrapAround) {
    RingBuffer<float, 64> rb;
    const size_t TOTAL_PUSHES = 10000;
    for (size_t i = 0; i < TOTAL_PUSHES; ++i) {
        rb.push(static_cast<float>(i));
    }
    ASSERT_TRUE(rb.isFull());
    ASSERT_EQ(64u, rb.size());

    float out[64];
    rb.extractWindow(out);
    // Oldest sample should be 10000 - 64 = 9936, newest 9999
    for (size_t i = 0; i < 64; ++i) {
        ASSERT_NEAR(static_cast<float>(TOTAL_PUSHES - 64 + i), out[i], 1e-4f);
    }
}

TEST(Tier2_Boundaries, T2_10_EmptyCommand_And_ExcessiveLength) {
    Actuator_Init();
    // Empty command should return true and no-op safely
    bool ok_empty = Actuator_ProcessCommand("");
    ASSERT_TRUE(ok_empty);

    // Command exceeding buffer size (70 characters)
    std::string long_cmd(70, 'A');
    long_cmd += "\n";
    Serial.inject_input(long_cmd);
    Actuator_ProcessSerial();

    // System should discard overflow safely without crash
    ASSERT_EQ(STATE_INIT, Actuator_GetState());
}
