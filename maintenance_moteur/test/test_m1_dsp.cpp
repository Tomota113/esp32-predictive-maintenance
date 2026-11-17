#include "test_framework.h"
#include <iostream>
#include <cmath>
#include <vector>
#include "config.h"
#include "sensors.h"
#include "dsp_pipeline.h"

TEST(M1_DSP, RingBuffer_PushAndWrap) {
    RingBuffer<float, 64> rb;

    ASSERT_EQ(0u, rb.size());
    ASSERT_FALSE(rb.isFull());
    ASSERT_EQ(64u, rb.capacity());

    // Push 30 samples
    for (int i = 0; i < 30; ++i) {
        rb.push(static_cast<float>(i));
    }
    ASSERT_EQ(30u, rb.size());
    ASSERT_FALSE(rb.isFull());

    // Fill to 64 samples
    for (int i = 30; i < 64; ++i) {
        rb.push(static_cast<float>(i));
    }
    ASSERT_EQ(64u, rb.size());
    ASSERT_TRUE(rb.isFull());

    float window[64];
    rb.extractWindow(window);
    for (int i = 0; i < 64; ++i) {
        ASSERT_NEAR(static_cast<float>(i), window[i], 1e-5f);
    }

    // Push 10 more samples (wrap around: 0..9 overwritten with 64..73)
    for (int i = 64; i < 74; ++i) {
        rb.push(static_cast<float>(i));
    }
    ASSERT_EQ(64u, rb.size());
    ASSERT_TRUE(rb.isFull());

    rb.extractWindow(window);
    // Oldest should now be 10, newest 73
    for (int i = 0; i < 64; ++i) {
        float expected = static_cast<float>(i + 10);
        ASSERT_NEAR(expected, window[i], 1e-5f);
    }

    rb.clear();
    ASSERT_EQ(0u, rb.size());
    ASSERT_FALSE(rb.isFull());
}

TEST(M1_DSP, DCSignal_DetrendingAndSuppression) {
    DSPPipeline dsp;

    // Push constant 9.81
    for (int i = 0; i < 64; ++i) {
        dsp.pushSample(9.81f);
    }
    ASSERT_TRUE(dsp.isWindowReady());

    DSPFeatures f = dsp.computeFeatures(24.5f);
    ASSERT_NEAR(0.0f, f.rms, 1e-4f);
    ASSERT_NEAR(0.0f, f.peak_to_peak, 1e-4f);
    ASSERT_NEAR(0.0f, f.peak_absolute, 1e-4f);
    ASSERT_NEAR(0.0f, f.dominant_frequency, 1e-4f);
    ASSERT_NEAR(24.5f, f.temperature, 1e-4f);
}

TEST(M1_DSP, SineNominal25Hz_Extraction) {
    DSPPipeline dsp;

    // 25 Hz sine wave, Fs = 200 Hz, Amplitude = 1.0 m/s^2
    const float Fs = 200.0f;
    const float freq = 25.0f;
    const float amp = 1.0f;

    for (int i = 0; i < 64; ++i) {
        float t = static_cast<float>(i) / Fs;
        float sample = amp * std::sin(2.0f * 3.14159265358979323846f * freq * t);
        dsp.pushSample(sample);
    }

    DSPFeatures f = dsp.computeFeatures(26.0f);

    // Expected RMS for sine amp 1.0 is ~0.707
    ASSERT_TRUE(f.rms > 0.60f && f.rms < 0.80f);
    ASSERT_TRUE(f.peak_absolute > 0.85f && f.peak_absolute <= 1.05f);
    ASSERT_TRUE(f.peak_to_peak > 1.70f && f.peak_to_peak <= 2.05f);
    // Dominant frequency bin should be 25.0 Hz (bin 8 * 3.125 = 25.0 Hz)
    ASSERT_NEAR(25.0f, f.dominant_frequency, 0.1f);
    ASSERT_NEAR(26.0f, f.temperature, 1e-4f);

    float feat_arr[4];
    f.toFeatureArray(feat_arr);
    ASSERT_NEAR(f.rms, feat_arr[0], 1e-5f);
    ASSERT_NEAR(f.peak_absolute, feat_arr[1], 1e-5f);
    ASSERT_NEAR(f.dominant_frequency, feat_arr[2], 1e-5f);
    ASSERT_NEAR(f.temperature, feat_arr[3], 1e-5f);
}

TEST(M1_DSP, SineAnomaly65Hz_Extraction) {
    DSPPipeline dsp;

    // 65 Hz fault sine wave, Fs = 200 Hz, Amplitude = 3.5 m/s^2
    const float Fs = 200.0f;
    const float freq = 65.0f;
    const float amp = 3.5f;

    for (int i = 0; i < 64; ++i) {
        float t = static_cast<float>(i) / Fs;
        float sample = amp * std::sin(2.0f * 3.14159265358979323846f * freq * t);
        dsp.pushSample(sample);
    }

    DSPFeatures f = dsp.computeFeatures(80.0f);

    // Expected RMS for sine amp 3.5 is ~2.47
    ASSERT_TRUE(f.rms > 2.0f && f.rms < 2.9f);
    ASSERT_TRUE(f.peak_absolute > 3.0f);
    // Bin 21 * 3.125 = 65.625 Hz
    ASSERT_NEAR(65.625f, f.dominant_frequency, 0.1f);
    ASSERT_NEAR(80.0f, f.temperature, 1e-4f);
}

TEST(M1_Sensors, CalibrationAndSampling) {
    MockClock::reset();
    Sensors_Init();

    ASSERT_TRUE(Sensors_IsMPUHealthy());
    ASSERT_TRUE(Sensors_IsDHTHealthy());

    // Calibrate baseline
    Sensors_Calibrate(50);
    float offset = Sensors_GetCalibrationOffset();
    ASSERT_TRUE(offset > 8.0f && offset < 11.5f);

    // Normal sample
    float z_sample = 0.0f;
    bool ok = Sensors_SampleMPU(z_sample);
    ASSERT_TRUE(ok);

    float temp = 0.0f;
    bool dht_ok = Sensors_SampleDHT(temp);
    ASSERT_TRUE(dht_ok);
    ASSERT_TRUE(temp > 0.0f && temp < 60.0f);

    // Test fault injection
    Sensors_SetVibrationInjection(true);
    ASSERT_TRUE(Sensors_IsVibrationInjected());

    Sensors_SetTemperatureInjection(true);
    ASSERT_TRUE(Sensors_IsTemperatureInjected());

    float injected_temp = 0.0f;
    Sensors_SampleDHT(injected_temp);
    ASSERT_NEAR(80.0f, injected_temp, 1e-4f);

    // Reset injection
    Sensors_SetVibrationInjection(false);
    Sensors_SetTemperatureInjection(false);
    ASSERT_FALSE(Sensors_IsVibrationInjected());
    ASSERT_FALSE(Sensors_IsTemperatureInjected());
}
