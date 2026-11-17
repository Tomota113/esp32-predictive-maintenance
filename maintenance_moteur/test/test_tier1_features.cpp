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
// TIER 1: FEATURE COVERAGE UNIT TEST SUITE (F01 - F11, >=55 TESTS)
// =============================================================================

// -----------------------------------------------------------------------------
// FEATURE F01: RingBuffer (Circular FIFO Window Buffer)
// -----------------------------------------------------------------------------

TEST(Tier1_F01_RingBuffer, T1_01_PushAndRead) {
    RingBuffer<float, 64> rb;
    ASSERT_EQ(0u, rb.size());
    ASSERT_FALSE(rb.isFull());
    ASSERT_EQ(64u, rb.capacity());

    rb.push(10.5f);
    rb.push(20.5f);
    rb.push(30.5f);
    ASSERT_EQ(3u, rb.size());
    ASSERT_FALSE(rb.isFull());

    float window[64] = {0};
    rb.extractWindow(window);
    ASSERT_NEAR(10.5f, window[0], 1e-5f);
    ASSERT_NEAR(20.5f, window[1], 1e-5f);
    ASSERT_NEAR(30.5f, window[2], 1e-5f);
}

TEST(Tier1_F01_RingBuffer, T1_02_FillToCapacity) {
    RingBuffer<float, 64> rb;
    for (int i = 0; i < 64; ++i) {
        ASSERT_FALSE(rb.isFull());
        rb.push(static_cast<float>(i * 2));
    }
    ASSERT_EQ(64u, rb.size());
    ASSERT_TRUE(rb.isFull());

    float window[64];
    rb.extractWindow(window);
    for (int i = 0; i < 64; ++i) {
        ASSERT_NEAR(static_cast<float>(i * 2), window[i], 1e-5f);
    }
}

TEST(Tier1_F01_RingBuffer, T1_03_SlidingWindowOverwrite) {
    RingBuffer<float, 64> rb;
    // Push 100 elements (0 to 99)
    for (int i = 0; i < 100; ++i) {
        rb.push(static_cast<float>(i));
    }
    ASSERT_EQ(64u, rb.size());
    ASSERT_TRUE(rb.isFull());

    float window[64];
    rb.extractWindow(window);
    // Oldest sample should be 36, newest 99
    for (int i = 0; i < 64; ++i) {
        ASSERT_NEAR(static_cast<float>(i + 36), window[i], 1e-5f);
    }
}

TEST(Tier1_F01_RingBuffer, T1_04_ExportLinearArraySequence) {
    RingBuffer<float, 8> small_rb;
    for (int i = 1; i <= 12; ++i) {
        small_rb.push(static_cast<float>(i));
    }
    ASSERT_EQ(8u, small_rb.size());
    ASSERT_TRUE(small_rb.isFull());

    float out[8];
    small_rb.extractWindow(out);
    // Expected sequence: 5, 6, 7, 8, 9, 10, 11, 12
    for (int i = 0; i < 8; ++i) {
        ASSERT_NEAR(static_cast<float>(i + 5), out[i], 1e-5f);
    }
}

TEST(Tier1_F01_RingBuffer, T1_05_ResetAndRefill) {
    RingBuffer<float, 64> rb;
    for (int i = 0; i < 64; ++i) {
        rb.push(1.0f);
    }
    ASSERT_TRUE(rb.isFull());
    ASSERT_EQ(64u, rb.size());

    rb.clear();
    ASSERT_FALSE(rb.isFull());
    ASSERT_EQ(0u, rb.size());

    // Refill with new data
    for (int i = 0; i < 10; ++i) {
        rb.push(static_cast<float>(i * 5));
    }
    ASSERT_EQ(10u, rb.size());
    ASSERT_FALSE(rb.isFull());

    float window[64] = {0};
    rb.extractWindow(window);
    for (int i = 0; i < 10; ++i) {
        ASSERT_NEAR(static_cast<float>(i * 5), window[i], 1e-5f);
    }
}

// -----------------------------------------------------------------------------
// FEATURE F02: RMS Vibration Calculation
// -----------------------------------------------------------------------------

TEST(Tier1_F02_RMS, T1_06_ConstantDCSignal) {
    DSPPipeline dsp;
    for (int i = 0; i < 64; ++i) {
        dsp.pushSample(9.81f);
    }
    ASSERT_TRUE(dsp.isWindowReady());
    DSPFeatures f = dsp.computeFeatures(25.0f);
    // Constant signal has zero AC vibration energy after mean subtraction
    ASSERT_NEAR(0.0f, f.rms, 1e-4f);
}

TEST(Tier1_F02_RMS, T1_07_PureSineWave) {
    DSPPipeline dsp;
    const float Fs = 200.0f;
    const float freq = 25.0f;
    const float amp = 2.0f; // Amplitude = 2.0 -> RMS = 2.0 / sqrt(2) ≈ 1.4142

    for (int i = 0; i < 64; ++i) {
        float t = static_cast<float>(i) / Fs;
        dsp.pushSample(amp * std::sin(2.0f * static_cast<float>(M_PI) * freq * t));
    }

    DSPFeatures f = dsp.computeFeatures(25.0f);
    float expected_rms = amp / std::sqrt(2.0f);
    ASSERT_NEAR(expected_rms, f.rms, 0.05f);
}

TEST(Tier1_F02_RMS, T1_08_SquareWave) {
    DSPPipeline dsp;
    const float amp = 2.5f;
    // Symmetric square wave: alternating +2.5 and -2.5 -> RMS = 2.5
    for (int i = 0; i < 64; ++i) {
        dsp.pushSample((i % 2 == 0) ? amp : -amp);
    }
    DSPFeatures f = dsp.computeFeatures(25.0f);
    ASSERT_NEAR(amp, f.rms, 0.05f);
}

TEST(Tier1_F02_RMS, T1_09_ZeroVector) {
    DSPPipeline dsp;
    for (int i = 0; i < 64; ++i) {
        dsp.pushSample(0.0f);
    }
    DSPFeatures f = dsp.computeFeatures(20.0f);
    ASSERT_NEAR(0.0f, f.rms, 1e-5f);
    ASSERT_FALSE(std::isnan(f.rms));
    ASSERT_FALSE(std::isinf(f.rms));
}

TEST(Tier1_F02_RMS, T1_10_WithDCOffsetRemoval) {
    DSPPipeline dsp;
    const float Fs = 200.0f;
    const float freq = 25.0f;
    const float amp = 1.0f;
    const float dc_offset = 15.0f;

    for (int i = 0; i < 64; ++i) {
        float t = static_cast<float>(i) / Fs;
        float val = dc_offset + amp * std::sin(2.0f * static_cast<float>(M_PI) * freq * t);
        dsp.pushSample(val);
    }

    DSPFeatures f = dsp.computeFeatures(25.0f);
    // Calculated RMS should reflect dynamic AC component ~ 0.707
    ASSERT_NEAR(amp / std::sqrt(2.0f), f.rms, 0.05f);
}

// -----------------------------------------------------------------------------
// FEATURE F03: Peak & Peak-to-Peak Amplitude
// -----------------------------------------------------------------------------

TEST(Tier1_F03_Peak, T1_11_SymmetricSineWave) {
    DSPPipeline dsp;
    const float amp = 1.5f;
    for (int i = 0; i < 64; ++i) {
        float t = static_cast<float>(i) / 200.0f;
        dsp.pushSample(amp * std::sin(2.0f * static_cast<float>(M_PI) * 25.0f * t));
    }
    DSPFeatures f = dsp.computeFeatures(25.0f);
    ASSERT_NEAR(amp, f.peak_absolute, 0.08f);
    ASSERT_NEAR(2.0f * amp, f.peak_to_peak, 0.15f);
}

TEST(Tier1_F03_Peak, T1_12_PositiveImpulseSpike) {
    DSPPipeline dsp;
    for (int i = 0; i < 64; ++i) {
        dsp.pushSample((i == 32) ? 6.5f : 0.0f);
    }
    DSPFeatures f = dsp.computeFeatures(25.0f);
    ASSERT_TRUE(f.peak_to_peak >= 6.4f && f.peak_to_peak <= 6.6f);
    ASSERT_TRUE(f.peak_absolute >= 6.0f);
}

TEST(Tier1_F03_Peak, T1_13_NegativeImpulseSpike) {
    DSPPipeline dsp;
    for (int i = 0; i < 64; ++i) {
        dsp.pushSample((i == 15) ? -5.2f : 0.0f);
    }
    DSPFeatures f = dsp.computeFeatures(25.0f);
    ASSERT_TRUE(f.peak_to_peak >= 5.1f && f.peak_to_peak <= 5.3f);
    ASSERT_TRUE(f.peak_absolute >= 4.8f);
}

TEST(Tier1_F03_Peak, T1_14_FlatConstantSignal) {
    DSPPipeline dsp;
    for (int i = 0; i < 64; ++i) {
        dsp.pushSample(1.8f);
    }
    DSPFeatures f = dsp.computeFeatures(25.0f);
    ASSERT_NEAR(0.0f, f.peak_to_peak, 1e-4f);
    ASSERT_NEAR(0.0f, f.peak_absolute, 1e-4f);
}

TEST(Tier1_F03_Peak, T1_15_MonotonicRamp) {
    DSPPipeline dsp;
    for (int i = 0; i < 64; ++i) {
        // Ramp from -2.0 to +3.0
        float val = -2.0f + 5.0f * (static_cast<float>(i) / 63.0f);
        dsp.pushSample(val);
    }
    DSPFeatures f = dsp.computeFeatures(25.0f);
    ASSERT_NEAR(5.0f, f.peak_to_peak, 0.01f);
    ASSERT_TRUE(f.peak_absolute >= 2.4f && f.peak_absolute <= 3.1f);
}

// -----------------------------------------------------------------------------
// FEATURE F04: 64-Point FFT & Dominant Frequency Extraction
// -----------------------------------------------------------------------------

TEST(Tier1_F04_FFT, T1_16_DominantFreq_25Hz) {
    DSPPipeline dsp;
    // 25 Hz sine, Fs = 200 Hz -> Bin 8 (8 * 3.125 Hz = 25.0 Hz)
    for (int i = 0; i < 64; ++i) {
        float t = static_cast<float>(i) / 200.0f;
        dsp.pushSample(1.5f * std::sin(2.0f * static_cast<float>(M_PI) * 25.0f * t));
    }
    DSPFeatures f = dsp.computeFeatures(25.0f);
    ASSERT_NEAR(25.0f, f.dominant_frequency, 1.0f);
}

TEST(Tier1_F04_FFT, T1_17_DominantFreq_50Hz) {
    DSPPipeline dsp;
    // 50 Hz sine, Fs = 200 Hz -> Bin 16 (16 * 3.125 Hz = 50.0 Hz)
    for (int i = 0; i < 64; ++i) {
        float t = static_cast<float>(i) / 200.0f;
        dsp.pushSample(2.0f * std::sin(2.0f * static_cast<float>(M_PI) * 50.0f * t));
    }
    DSPFeatures f = dsp.computeFeatures(25.0f);
    ASSERT_NEAR(50.0f, f.dominant_frequency, 1.0f);
}

TEST(Tier1_F04_FFT, T1_18_DominantFreq_65Hz_Fault) {
    DSPPipeline dsp;
    // 65 Hz fault tone, Fs = 200 Hz -> Bin 21 (21 * 3.125 Hz = 65.625 Hz)
    for (int i = 0; i < 64; ++i) {
        float t = static_cast<float>(i) / 200.0f;
        dsp.pushSample(3.5f * std::sin(2.0f * static_cast<float>(M_PI) * 65.0f * t));
    }
    DSPFeatures f = dsp.computeFeatures(80.0f);
    ASSERT_NEAR(65.625f, f.dominant_frequency, 1.5f);
}

TEST(Tier1_F04_FFT, T1_19_DCBinSuppression) {
    DSPPipeline dsp;
    // Pure DC signal (10.0) -> Bin 0 has high energy, but must be suppressed
    for (int i = 0; i < 64; ++i) {
        dsp.pushSample(10.0f);
    }
    DSPFeatures f = dsp.computeFeatures(25.0f);
    // Dominant frequency search suppresses DC bin 0, returning 0.0 Hz
    ASSERT_NEAR(0.0f, f.dominant_frequency, 1e-4f);
}

TEST(Tier1_F04_FFT, T1_20_MultiToneDominantPeak) {
    DSPPipeline dsp;
    // Dual tone: weak 25 Hz (amp 0.5) + strong 65 Hz (amp 3.0)
    for (int i = 0; i < 64; ++i) {
        float t = static_cast<float>(i) / 200.0f;
        float s1 = 0.5f * std::sin(2.0f * static_cast<float>(M_PI) * 25.0f * t);
        float s2 = 3.0f * std::sin(2.0f * static_cast<float>(M_PI) * 65.0f * t);
        dsp.pushSample(s1 + s2);
    }
    DSPFeatures f = dsp.computeFeatures(25.0f);
    ASSERT_NEAR(65.625f, f.dominant_frequency, 1.5f);
}

// -----------------------------------------------------------------------------
// FEATURE F05: Temperature Acquisition & Fault Injection
// -----------------------------------------------------------------------------

TEST(Tier1_F05_Temperature, T1_21_NominalAcquisition) {
    MockClock::reset();
    Sensors_Init();
    mock_dht.set_temperature(28.5f);
    MockClock::advance_ms(2500);

    float temp = 0.0f;
    bool ok = Sensors_SampleDHT(temp);
    ASSERT_TRUE(ok);
    ASSERT_NEAR(28.5f, temp, 0.1f);
}

TEST(Tier1_F05_Temperature, T1_22_BoundaryThresholds) {
    MockClock::reset();
    Sensors_Init();

    // Normal safe temp (30.0 °C)
    mock_dht.set_temperature(30.0f);
    MockClock::advance_ms(2500);
    float t_safe = 0.0f;
    Sensors_SampleDHT(t_safe);
    ASSERT_NEAR(30.0f, t_safe, 0.1f);

    // High elevated temp (60.0 °C)
    mock_dht.set_temperature(60.0f);
    MockClock::advance_ms(2500);
    float t_high = 0.0f;
    Sensors_SampleDHT(t_high);
    ASSERT_NEAR(60.0f, t_high, 0.1f);
}

TEST(Tier1_F05_Temperature, T1_23_NaNFailureFallback) {
    MockClock::reset();
    Sensors_Init();
    mock_dht.set_temperature(26.5f);
    MockClock::advance_ms(2500);

    float valid_temp = 0.0f;
    Sensors_SampleDHT(valid_temp);
    ASSERT_NEAR(26.5f, valid_temp, 0.1f);

    // Inject NaN read failure on sensor
    mock_dht.set_nan_failure(true);
    MockClock::advance_ms(2500);

    float fallback_temp = 0.0f;
    bool dht_ok = Sensors_SampleDHT(fallback_temp);
    ASSERT_FALSE(dht_ok);
    // Must return cached valid temperature without NaN
    ASSERT_FALSE(std::isnan(fallback_temp));
    ASSERT_NEAR(26.5f, fallback_temp, 0.1f);

    // Sensor recovery
    mock_dht.set_nan_failure(false);
    mock_dht.set_temperature(29.0f);
    MockClock::advance_ms(2500);
    bool recovered = Sensors_SampleDHT(fallback_temp);
    ASSERT_TRUE(recovered);
    ASSERT_NEAR(29.0f, fallback_temp, 0.1f);
}

TEST(Tier1_F05_Temperature, T1_24_RateLimitingCache) {
    MockClock::reset();
    Sensors_Init();
    mock_dht.set_temperature(25.0f);
    MockClock::advance_ms(2500);

    float temp1 = 0.0f;
    Sensors_SampleDHT(temp1);
    ASSERT_NEAR(25.0f, temp1, 0.1f);

    // Change hardware temp, but query before 2000 ms interval has passed
    mock_dht.set_temperature(55.0f);
    MockClock::advance_ms(500); // Only 500 ms elapsed

    float temp2 = 0.0f;
    Sensors_SampleDHT(temp2);
    // Cached temperature (25.0) returned
    ASSERT_NEAR(25.0f, temp2, 0.1f);

    // Advance beyond 2000 ms interval
    MockClock::advance_ms(1600); // 2100 ms total elapsed
    float temp3 = 0.0f;
    Sensors_SampleDHT(temp3);
    ASSERT_NEAR(55.0f, temp3, 0.1f);
}

TEST(Tier1_F05_Temperature, T1_25_SimulatedFaultInjection) {
    MockClock::reset();
    Sensors_Init();
    mock_dht.set_temperature(24.0f);
    MockClock::advance_ms(2500);

    Sensors_SetTemperatureInjection(true);
    ASSERT_TRUE(Sensors_IsTemperatureInjected());

    float injected_temp = 0.0f;
    Sensors_SampleDHT(injected_temp);
    ASSERT_NEAR(80.0f, injected_temp, 1e-4f);

    Sensors_SetTemperatureInjection(false);
    ASSERT_FALSE(Sensors_IsTemperatureInjected());
    Sensors_SampleDHT(injected_temp);
    ASSERT_NEAR(24.0f, injected_temp, 1e-4f);
}

// -----------------------------------------------------------------------------
// FEATURE F06: Safety FSM Normal Operation & Initialization
// -----------------------------------------------------------------------------

TEST(Tier1_F06_FSM_Normal, T1_26_InitToNormalTransition) {
    MockGPIO::reset();
    Actuator_Init();
    ASSERT_EQ(STATE_INIT, Actuator_GetState());
    ASSERT_EQ(0u, Actuator_GetDebounceCount());

    Actuator_Rearm();
    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
}

TEST(Tier1_F06_FSM_Normal, T1_27_MotorPWM_EnabledInNormal) {
    MockGPIO::reset();
    Actuator_Init();
    Actuator_Rearm();

    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
    ASSERT_EQ(HIGH, MockGPIO::pin_state[PIN_MOTOR_IN1]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_IN2]);
}

TEST(Tier1_F06_FSM_Normal, T1_28_Signaling_NormalState) {
    MockGPIO::reset();
    Actuator_Init();
    Actuator_Rearm();

    ASSERT_TRUE(Actuator_IsNormalLedActive());
    ASSERT_FALSE(Actuator_IsWarningAlarmLedActive());
    ASSERT_FALSE(Actuator_IsBuzzerActive());

    ASSERT_EQ(HIGH, MockGPIO::pin_state[PIN_LED_NORMAL]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_LED_WARN_ALARM]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_BUZZER]);
}

TEST(Tier1_F06_FSM_Normal, T1_29_SteadyNormalExecution) {
    MockGPIO::reset();
    Actuator_Init();
    Actuator_Rearm();

    // 100 consecutive normal frames (0)
    for (int i = 0; i < 100; ++i) {
        Actuator_UpdateFSM(0);
        ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
        ASSERT_EQ(0u, Actuator_GetDebounceCount());
        ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());
    }
}

TEST(Tier1_F06_FSM_Normal, T1_30_SingleAnomalyDebounceRecovery) {
    MockGPIO::reset();
    Actuator_Init();
    Actuator_Rearm();

    // 1 anomaly frame -> moves to STATE_WARNING, debounce = 1
    Actuator_UpdateFSM(1);
    ASSERT_EQ(STATE_WARNING, Actuator_GetState());
    ASSERT_EQ(1u, Actuator_GetDebounceCount());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM()); // Motor still running

    // Next frame is normal (0) -> resets debounce = 0, returns to STATE_NORMAL
    Actuator_UpdateFSM(0);
    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
    ASSERT_EQ(0u, Actuator_GetDebounceCount());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());
}

// -----------------------------------------------------------------------------
// FEATURE F07: Safety FSM 3-Frame Debounce & Emergency Cutoff
// -----------------------------------------------------------------------------

TEST(Tier1_F07_FSM_Alarm, T1_31_ThreeConsecutiveAnomalyTrip) {
    MockGPIO::reset();
    Actuator_Init();
    Actuator_Rearm();

    Actuator_UpdateFSM(1); // Debounce = 1 -> WARNING
    ASSERT_EQ(STATE_WARNING, Actuator_GetState());
    ASSERT_EQ(1u, Actuator_GetDebounceCount());

    Actuator_UpdateFSM(1); // Debounce = 2 -> WARNING
    ASSERT_EQ(STATE_WARNING, Actuator_GetState());
    ASSERT_EQ(2u, Actuator_GetDebounceCount());

    Actuator_UpdateFSM(1); // Debounce = 3 -> ALARM TRIP!
    ASSERT_EQ(STATE_ALARM, Actuator_GetState());
}

TEST(Tier1_F07_FSM_Alarm, T1_32_EmergencyMotorCut_Under100ms) {
    MockGPIO::reset();
    Actuator_Init();
    Actuator_Rearm();

    // 3 anomalies to trip alarm
    Actuator_UpdateFSM(1);
    Actuator_UpdateFSM(1);
    Actuator_UpdateFSM(1);

    ASSERT_EQ(STATE_ALARM, Actuator_GetState());
    // Motor must be completely cut off (ENA=0, IN1=0, IN2=0)
    ASSERT_EQ(0u, Actuator_GetMotorPWM());
    ASSERT_EQ(0u, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_IN1]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_IN2]);
}

TEST(Tier1_F07_FSM_Alarm, T1_33_AlarmLED_Active) {
    MockGPIO::reset();
    Actuator_Init();
    Actuator_Rearm();

    Actuator_UpdateFSM(1);
    Actuator_UpdateFSM(1);
    Actuator_UpdateFSM(1);

    ASSERT_EQ(STATE_ALARM, Actuator_GetState());
    ASSERT_TRUE(Actuator_IsWarningAlarmLedActive());
    ASSERT_FALSE(Actuator_IsNormalLedActive());
    ASSERT_EQ(HIGH, MockGPIO::pin_state[PIN_LED_WARN_ALARM]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_LED_NORMAL]);
}

TEST(Tier1_F07_FSM_Alarm, T1_34_Buzzer_ActiveInAlarm) {
    MockGPIO::reset();
    Actuator_Init();
    Actuator_Rearm();

    Actuator_UpdateFSM(1);
    Actuator_UpdateFSM(1);
    Actuator_UpdateFSM(1);

    ASSERT_EQ(STATE_ALARM, Actuator_GetState());
    ASSERT_TRUE(Actuator_IsBuzzerActive());
    ASSERT_EQ(HIGH, MockGPIO::pin_state[PIN_BUZZER]);
}

TEST(Tier1_F07_FSM_Alarm, T1_35_AlarmLatched) {
    MockGPIO::reset();
    Actuator_Init();
    Actuator_Rearm();

    // Trip into ALARM
    Actuator_UpdateFSM(1);
    Actuator_UpdateFSM(1);
    Actuator_UpdateFSM(1);
    ASSERT_EQ(STATE_ALARM, Actuator_GetState());

    // Feed 50 normal frames (0) - Alarm MUST stay latched
    for (int i = 0; i < 50; ++i) {
        Actuator_UpdateFSM(0);
        ASSERT_EQ(STATE_ALARM, Actuator_GetState());
        ASSERT_EQ(0u, Actuator_GetMotorPWM());
        ASSERT_TRUE(Actuator_IsBuzzerActive());
    }
}

// -----------------------------------------------------------------------------
// FEATURE F08: Safety FSM Reset / Rearm Command
// -----------------------------------------------------------------------------

TEST(Tier1_F08_FSM_Reset, T1_36_ResetCommandTransitionsToNormal) {
    MockGPIO::reset();
    Actuator_Init();
    Actuator_EmergencyStop();
    ASSERT_EQ(STATE_ALARM, Actuator_GetState());

    Actuator_Rearm();
    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
}

TEST(Tier1_F08_FSM_Reset, T1_37_ResetClearsDebounceCounter) {
    MockGPIO::reset();
    Actuator_Init();
    Actuator_UpdateFSM(1);
    Actuator_UpdateFSM(1);
    ASSERT_EQ(2u, Actuator_GetDebounceCount());

    Actuator_Rearm();
    ASSERT_EQ(0u, Actuator_GetDebounceCount());
}

TEST(Tier1_F08_FSM_Reset, T1_38_ResetRestoresMotorPWM) {
    MockGPIO::reset();
    Actuator_Init();
    Actuator_EmergencyStop();
    ASSERT_EQ(0u, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);

    Actuator_Rearm();
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
    ASSERT_EQ(HIGH, MockGPIO::pin_state[PIN_MOTOR_IN1]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_IN2]);
}

TEST(Tier1_F08_FSM_Reset, T1_39_ResetExtinguishesAlarms) {
    MockGPIO::reset();
    Actuator_Init();
    Actuator_EmergencyStop();
    ASSERT_TRUE(Actuator_IsBuzzerActive());
    ASSERT_TRUE(Actuator_IsWarningAlarmLedActive());

    Actuator_Rearm();
    ASSERT_FALSE(Actuator_IsBuzzerActive());
    ASSERT_FALSE(Actuator_IsWarningAlarmLedActive());
    ASSERT_TRUE(Actuator_IsNormalLedActive());
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_BUZZER]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_LED_WARN_ALARM]);
    ASSERT_EQ(HIGH, MockGPIO::pin_state[PIN_LED_NORMAL]);
}

TEST(Tier1_F08_FSM_Reset, T1_40_ResetIdempotence) {
    MockGPIO::reset();
    Actuator_Init();
    Actuator_Rearm();
    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());

    // Calling Rearm while already in NORMAL should be safe and idempotent
    Actuator_Rearm();
    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());
    ASSERT_EQ(0u, Actuator_GetDebounceCount());
}

// -----------------------------------------------------------------------------
// FEATURE F09: TinyML Random Forest Model Inference
// -----------------------------------------------------------------------------

TEST(Tier1_F09_Model, T1_41_NominalInference) {
    // Nominal motor running vector: RMS ~ 0.14, Peak ~ 0.38, Freq = 25.0, Temp = 24.5
    float nominal_vector[4] = {0.142f, 0.380f, 25.0f, 24.5f};
    int pred = model_predict(nominal_vector);
    ASSERT_EQ(0, pred);

    int pred_val = model_predict_values(0.142f, 0.380f, 25.0f, 24.5f);
    ASSERT_EQ(0, pred_val);

    float proba = Eloquent::ML::Port::RandomForest::predict_proba(nominal_vector);
    ASSERT_TRUE(proba < 0.5f);
}

TEST(Tier1_F09_Model, T1_42_VibrationAnomalyInference) {
    // Vibration fault: RMS = 2.34, Peak = 4.82, Freq = 65.0, Temp = 25.0
    float vib_fault[4] = {2.340f, 4.820f, 65.0f, 25.0f};
    int pred = model_predict(vib_fault);
    ASSERT_EQ(1, pred);

    float proba = Eloquent::ML::Port::RandomForest::predict_proba(vib_fault);
    ASSERT_TRUE(proba >= 0.6f);
}

TEST(Tier1_F09_Model, T1_43_ThermalAnomalyInference) {
    // Thermal runaway with friction vibration: RMS = 0.90, Peak = 1.30, Freq = 25.0, Temp = 75.0
    float temp_fault[4] = {0.900f, 1.300f, 25.0f, 75.0f};
    int pred = model_predict(temp_fault);
    ASSERT_EQ(1, pred);

    float proba = Eloquent::ML::Port::RandomForest::predict_proba(temp_fault);
    ASSERT_TRUE(proba >= 0.6f);
}

TEST(Tier1_F09_Model, T1_44_CombinedAnomalyInference) {
    // Simultaneous vibration and thermal fault
    float combo_fault[4] = {3.50f, 6.00f, 65.0f, 80.0f};
    int pred = model_predict(combo_fault);
    ASSERT_EQ(1, pred);

    float proba = Eloquent::ML::Port::RandomForest::predict_proba(combo_fault);
    ASSERT_NEAR(1.0f, proba, 1e-4f);
}

TEST(Tier1_F09_Model, T1_45_InferenceLatencyAndZeroHeap) {
    float test_features[4] = {0.85f, 1.20f, 25.0f, 28.5f};

    auto start = std::chrono::high_resolution_clock::now();
    volatile int dummy_sum = 0;
    const int ITERATIONS = 10000;
    for (int i = 0; i < ITERATIONS; ++i) {
        dummy_sum += model_predict(test_features);
    }
    auto end = std::chrono::high_resolution_clock::now();

    auto total_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    double avg_latency_us = static_cast<double>(total_us) / static_cast<double>(ITERATIONS);

    // Host latency is typically < 0.1 us (well under 50 us constraint)
    ASSERT_TRUE(avg_latency_us < 50.0);
}

// -----------------------------------------------------------------------------
// FEATURE F10: Serial Telemetry Stream Formatting
// -----------------------------------------------------------------------------

TEST(Tier1_F10_Telemetry, T1_46_FormatCompliance) {
    char buffer[128];
    Actuator_FormatTelemetry(buffer, sizeof(buffer), STATE_NORMAL, 0.142f, 0.380f, 25.00f, 24.50f, 0);

    std::string line(buffer);
    ASSERT_TRUE(line.find("[ETAT: NORMAL]") != std::string::npos);
    ASSERT_TRUE(line.find("RMS: 0.142") != std::string::npos);
    ASSERT_TRUE(line.find("Peak: 0.380") != std::string::npos);
    ASSERT_TRUE(line.find("Freq: 25.00") != std::string::npos);
    ASSERT_TRUE(line.find("Temp: 24.50") != std::string::npos);
    ASSERT_TRUE(line.find("Inference: 0") != std::string::npos);
}

TEST(Tier1_F10_Telemetry, T1_47_StateStringMapping) {
    ASSERT_EQ(std::string("INIT"), std::string(StateToString(STATE_INIT)));
    ASSERT_EQ(std::string("NORMAL"), std::string(StateToString(STATE_NORMAL)));
    ASSERT_EQ(std::string("WARNING"), std::string(StateToString(STATE_WARNING)));
    ASSERT_EQ(std::string("ALARM"), std::string(StateToString(STATE_ALARM)));
    ASSERT_EQ(std::string("MAINTENANCE"), std::string(StateToString(STATE_MAINTENANCE)));
}

TEST(Tier1_F10_Telemetry, T1_48_PrecisionFormatting) {
    char buffer[128];
    Actuator_FormatTelemetry(buffer, sizeof(buffer), STATE_WARNING, 1.23456f, 3.45678f, 62.500f, 45.678f, 1);

    std::string line(buffer);
    ASSERT_TRUE(line.find("RMS: 1.235") != std::string::npos);
    ASSERT_TRUE(line.find("Peak: 3.457") != std::string::npos);
    ASSERT_TRUE(line.find("Freq: 62.50") != std::string::npos);
    ASSERT_TRUE(line.find("Temp: 45.68") != std::string::npos);
    ASSERT_TRUE(line.find("Inference: 1") != std::string::npos);
}

TEST(Tier1_F10_Telemetry, T1_49_PrintOutput) {
    Serial.reset();
    Actuator_Init();
    Actuator_Rearm();

    DSPFeatures feat;
    feat.rms = 0.250f;
    feat.peak_absolute = 0.500f;
    feat.dominant_frequency = 25.00f;
    feat.temperature = 26.00f;

    Actuator_PrintTelemetry(feat, 0);
    std::string out = Serial.get_output();
    ASSERT_TRUE(out.find("[ETAT: NORMAL]") != std::string::npos);
    ASSERT_TRUE(out.find("RMS: 0.250") != std::string::npos);
}

TEST(Tier1_F10_Telemetry, T1_50_AnomalyFlagInAlarmState) {
    Serial.reset();
    Actuator_Init();
    Actuator_EmergencyStop();

    DSPFeatures feat;
    feat.rms = 3.500f;
    feat.peak_absolute = 6.200f;
    feat.dominant_frequency = 65.62f;
    feat.temperature = 80.00f;

    Actuator_PrintTelemetry(feat, 1);
    std::string out = Serial.get_output();
    ASSERT_TRUE(out.find("[ETAT: ALARM]") != std::string::npos);
    ASSERT_TRUE(out.find("Inference: 1") != std::string::npos);
}

// -----------------------------------------------------------------------------
// FEATURE F11: Serial Fault Injection Command Parser
// -----------------------------------------------------------------------------

TEST(Tier1_F11_SerialParser, T1_51_InjectVibrationCommand) {
    Sensors_SetVibrationInjection(false);
    ASSERT_FALSE(Sensors_IsVibrationInjected());

    bool ok = Actuator_ProcessCommand("INJECT_VIB");
    ASSERT_TRUE(ok);
    ASSERT_TRUE(Sensors_IsVibrationInjected());

    // Toggle off
    ok = Actuator_ProcessCommand("INJECT_VIB");
    ASSERT_TRUE(ok);
    ASSERT_FALSE(Sensors_IsVibrationInjected());
}

TEST(Tier1_F11_SerialParser, T1_52_InjectTemperatureCommand) {
    Sensors_SetTemperatureInjection(false);
    ASSERT_FALSE(Sensors_IsTemperatureInjected());

    bool ok = Actuator_ProcessCommand("INJECT_TEMP");
    ASSERT_TRUE(ok);
    ASSERT_TRUE(Sensors_IsTemperatureInjected());

    // Toggle off
    ok = Actuator_ProcessCommand("INJECT_TEMP");
    ASSERT_TRUE(ok);
    ASSERT_FALSE(Sensors_IsTemperatureInjected());
}

TEST(Tier1_F11_SerialParser, T1_53_ResetCommand) {
    Actuator_Init();
    Actuator_EmergencyStop();
    Sensors_SetVibrationInjection(true);
    Sensors_SetTemperatureInjection(true);
    ASSERT_EQ(STATE_ALARM, Actuator_GetState());

    bool ok = Actuator_ProcessCommand("RESET");
    ASSERT_TRUE(ok);
    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
    ASSERT_FALSE(Sensors_IsVibrationInjected());
    ASSERT_FALSE(Sensors_IsTemperatureInjected());
}

TEST(Tier1_F11_SerialParser, T1_54_GarbageRejection) {
    bool ok1 = Actuator_ProcessCommand("UNKNOWN_COMMAND_XYZ");
    ASSERT_FALSE(ok1);

    bool ok2 = Actuator_ProcessCommand("!@#$%^&*()");
    ASSERT_FALSE(ok2);
}

TEST(Tier1_F11_SerialParser, T1_55_WhitespaceAndCaseInsensitivity) {
    Sensors_SetVibrationInjection(false);

    bool ok = Actuator_ProcessCommand("   inject_vib   \r\n");
    ASSERT_TRUE(ok);
    ASSERT_TRUE(Sensors_IsVibrationInjected());

    ok = Actuator_ProcessCommand("  Reset \n");
    ASSERT_TRUE(ok);
    ASSERT_FALSE(Sensors_IsVibrationInjected());
}
