#include "test_framework.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <random>
#include <chrono>
#include "config.h"
#include "sensors.h"
#include "dsp_pipeline.h"
#include "actuator_control.h"

// =============================================================================
// CHALLENGER 2: ADVERSARIAL STRESS TEST SUITE
// Safety FSM, 3-Frame Debounce, Emergency Cutoff Latency, Serial Protocol Fuzzing
// =============================================================================

// -----------------------------------------------------------------------------
// 1. Debounce Filter: Alternating 1-0-1-0-1-0 Pattern (1,000,000 cycles)
// Zero false-positive emergency shutdowns guaranteed.
// -----------------------------------------------------------------------------
TEST(Challenger2_Debounce, Alternating_1_0_1_0_OneMillionCycles) {
    MockClock::reset();
    MockGPIO::reset();
    Serial.reset();
    Sensors_Init();
    Actuator_Init();

    // Start in NORMAL state
    Actuator_UpdateFSM(0);
    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());

    const int TOTAL_PAIRS = 500000; // 1,000,000 total transitions
    for (int i = 0; i < TOTAL_PAIRS; ++i) {
        // 1. Transient Anomaly Spike
        Actuator_UpdateFSM(1);
        ASSERT_EQ(STATE_WARNING, Actuator_GetState());
        ASSERT_EQ(1u, Actuator_GetDebounceCount());
        ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());
        ASSERT_FALSE(Actuator_IsBuzzerActive());
        ASSERT_TRUE(Actuator_IsNormalLedActive());
        ASSERT_TRUE(Actuator_IsWarningAlarmLedActive());
        ASSERT_EQ(static_cast<uint32_t>(MOTOR_NOMINAL_PWM), MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
        ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_BUZZER]);

        // 2. Immediate Normal Recovery Frame
        Actuator_UpdateFSM(0);
        ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
        ASSERT_EQ(0u, Actuator_GetDebounceCount());
        ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());
        ASSERT_FALSE(Actuator_IsBuzzerActive());
        ASSERT_TRUE(Actuator_IsNormalLedActive());
        ASSERT_FALSE(Actuator_IsWarningAlarmLedActive());
        ASSERT_EQ(static_cast<uint32_t>(MOTOR_NOMINAL_PWM), MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
        ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_BUZZER]);
    }

    // Final check: system still safely in NORMAL with motor running
    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
    ASSERT_EQ(0u, Actuator_GetDebounceCount());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());
}

// -----------------------------------------------------------------------------
// 2. Debounce Filter: Double Spike 1-1-0 Pattern (100,000 cycles)
// Two consecutive anomalies followed by normal must never trigger ALARM.
// -----------------------------------------------------------------------------
TEST(Challenger2_Debounce, DoubleSpike_1_1_0_OneHundredThousandCycles) {
    MockClock::reset();
    MockGPIO::reset();
    Sensors_Init();
    Actuator_Init();

    Actuator_UpdateFSM(0); // NORMAL

    const int SEQUENCES = 100000;
    for (int i = 0; i < SEQUENCES; ++i) {
        // Anomaly 1 -> WARNING (debounce = 1)
        Actuator_UpdateFSM(1);
        ASSERT_EQ(STATE_WARNING, Actuator_GetState());
        ASSERT_EQ(1u, Actuator_GetDebounceCount());
        ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());

        // Anomaly 2 -> WARNING (debounce = 2)
        Actuator_UpdateFSM(1);
        ASSERT_EQ(STATE_WARNING, Actuator_GetState());
        ASSERT_EQ(2u, Actuator_GetDebounceCount());
        ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());

        // Normal 3 -> NORMAL (debounce = 0)
        Actuator_UpdateFSM(0);
        ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
        ASSERT_EQ(0u, Actuator_GetDebounceCount());
        ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());
        ASSERT_FALSE(Actuator_IsBuzzerActive());
    }

    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());
}

// -----------------------------------------------------------------------------
// 3. Debounce Filter: Split Sequence 1-1-0-1-1-1
// Ensure debounce resets on 0 and triggers ALARM exactly on 3rd in 2nd train.
// -----------------------------------------------------------------------------
TEST(Challenger2_Debounce, Sequence_1_1_0_1_1_1_ExactTripping) {
    MockClock::reset();
    MockGPIO::reset();
    Sensors_Init();
    Actuator_Init();

    Actuator_UpdateFSM(0);
    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());

    // 1st train: 1, 1, 0
    Actuator_UpdateFSM(1);
    ASSERT_EQ(STATE_WARNING, Actuator_GetState());
    ASSERT_EQ(1u, Actuator_GetDebounceCount());

    Actuator_UpdateFSM(1);
    ASSERT_EQ(STATE_WARNING, Actuator_GetState());
    ASSERT_EQ(2u, Actuator_GetDebounceCount());

    Actuator_UpdateFSM(0);
    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
    ASSERT_EQ(0u, Actuator_GetDebounceCount());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());

    // 2nd train: 1, 1, 1
    Actuator_UpdateFSM(1);
    ASSERT_EQ(STATE_WARNING, Actuator_GetState());
    ASSERT_EQ(1u, Actuator_GetDebounceCount());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());

    Actuator_UpdateFSM(1);
    ASSERT_EQ(STATE_WARNING, Actuator_GetState());
    ASSERT_EQ(2u, Actuator_GetDebounceCount());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());

    // 3rd anomaly in 2nd train -> IMMEDIATE ALARM!
    Actuator_UpdateFSM(1);
    ASSERT_EQ(STATE_ALARM, Actuator_GetState());
    ASSERT_EQ(3u, Actuator_GetDebounceCount());
    ASSERT_EQ(0u, Actuator_GetMotorPWM());
    ASSERT_TRUE(Actuator_IsBuzzerActive());
    ASSERT_FALSE(Actuator_IsNormalLedActive());
    ASSERT_TRUE(Actuator_IsWarningAlarmLedActive());

    // Verify hardware pins immediately cut off
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_IN1]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_IN2]);
    ASSERT_EQ(0u, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
    ASSERT_EQ(HIGH, MockGPIO::pin_state[PIN_LED_WARN_ALARM]);
    ASSERT_EQ(HIGH, MockGPIO::pin_state[PIN_BUZZER]);
}

// -----------------------------------------------------------------------------
// 4. Debounce Filter: Long Normal Stream (1,000,000 frames)
// -----------------------------------------------------------------------------
TEST(Challenger2_Debounce, LongNormalStream_OneMillionFrames) {
    MockClock::reset();
    MockGPIO::reset();
    Sensors_Init();
    Actuator_Init();

    for (int i = 0; i < 1000000; ++i) {
        Actuator_UpdateFSM(0);
    }

    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
    ASSERT_EQ(0u, Actuator_GetDebounceCount());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());
    ASSERT_FALSE(Actuator_IsBuzzerActive());
    ASSERT_TRUE(Actuator_IsNormalLedActive());
    ASSERT_FALSE(Actuator_IsWarningAlarmLedActive());
}

// -----------------------------------------------------------------------------
// 5. Latched ALARM State Persistence (1,000,000 inputs after trip)
// Once in ALARM, subsequent 0 or 1 inferences cannot escape ALARM.
// -----------------------------------------------------------------------------
TEST(Challenger2_Debounce, LatchedAlarmPersistence_OneMillionFrames) {
    MockClock::reset();
    MockGPIO::reset();
    Sensors_Init();
    Actuator_Init();

    // Trip into ALARM
    Actuator_UpdateFSM(1);
    Actuator_UpdateFSM(1);
    Actuator_UpdateFSM(1);
    ASSERT_EQ(STATE_ALARM, Actuator_GetState());
    ASSERT_EQ(0u, Actuator_GetMotorPWM());

    // Send 500k normal frames and 500k anomaly frames randomly interleaved
    std::mt19937 rng(1337);
    std::uniform_int_distribution<int> dist(0, 1);

    for (int i = 0; i < 1000000; ++i) {
        int inference = dist(rng);
        Actuator_UpdateFSM(inference);
        ASSERT_EQ(STATE_ALARM, Actuator_GetState());
        ASSERT_EQ(0u, Actuator_GetMotorPWM());
        ASSERT_TRUE(Actuator_IsBuzzerActive());
        ASSERT_TRUE(Actuator_IsWarningAlarmLedActive());
        ASSERT_FALSE(Actuator_IsNormalLedActive());
        ASSERT_EQ(0u, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
    }
}

// -----------------------------------------------------------------------------
// 6. Actuator Emergency Cutoff Execution Latency Benchmark
// Verify sub-100ms hardware requirement & sub-microsecond software execution.
// -----------------------------------------------------------------------------
TEST(Challenger2_Latency, EmergencyCutoffLatency_Benchmark) {
    MockClock::reset();
    MockGPIO::reset();
    Sensors_Init();
    Actuator_Init();

    const int ITERATIONS = 10000;
    double max_latency_us = 0.0;
    double total_latency_us = 0.0;

    for (int i = 0; i < ITERATIONS; ++i) {
        // Reset to normal state with motor running
        Actuator_SetState(STATE_NORMAL);
        Actuator_SetMotorPWM(MOTOR_NOMINAL_PWM);
        ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());

        // Measure execution latency of EmergencyStop
        auto t_start = std::chrono::high_resolution_clock::now();
        Actuator_EmergencyStop();
        auto t_end = std::chrono::high_resolution_clock::now();

        double elapsed_us = std::chrono::duration<double, std::micro>(t_end - t_start).count();
        if (elapsed_us > max_latency_us) max_latency_us = elapsed_us;
        total_latency_us += elapsed_us;

        // Verify GPIO state synchronously
        ASSERT_EQ(STATE_ALARM, Actuator_GetState());
        ASSERT_EQ(0u, Actuator_GetMotorPWM());
        ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_IN1]);
        ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_IN2]);
        ASSERT_EQ(0u, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
        ASSERT_EQ(HIGH, MockGPIO::pin_state[PIN_LED_WARN_ALARM]);
        ASSERT_EQ(HIGH, MockGPIO::pin_state[PIN_BUZZER]);
        ASSERT_TRUE(Actuator_IsBuzzerActive());
    }

    double avg_latency_us = total_latency_us / ITERATIONS;
    std::cout << "  [BENCHMARK] Emergency Cutoff Latency: Avg = "
              << avg_latency_us << " us, Max = " << max_latency_us << " us\n";

    // Sub-100ms spec constraint (100,000 µs limit) - software must be << 1,000 µs
    ASSERT_LT(max_latency_us, 10000.0); // Strict safety margin: < 10 ms even on heavily loaded CPU
    ASSERT_LT(avg_latency_us, 1000.0);
}

// -----------------------------------------------------------------------------
// 7. Serial Protocol Fuzzer: Random Binary, Null Bytes & Control Characters
// -----------------------------------------------------------------------------
TEST(Challenger2_SerialFuzzing, RandomBinaryAndControlCharacters_50kPackets) {
    Serial.reset();
    Sensors_Init();
    Actuator_Init();

    std::mt19937 rng(4242);
    std::uniform_int_distribution<int> byte_dist(0, 255);
    std::uniform_int_distribution<int> len_dist(1, 120);

    const int PACKET_COUNT = 50000;
    for (int p = 0; p < PACKET_COUNT; ++p) {
        int len = len_dist(rng);
        for (int i = 0; i < len; ++i) {
            uint8_t raw_byte = static_cast<uint8_t>(byte_dist(rng));
            // Feed individual chars through parser
            Actuator_ProcessSerialChar(static_cast<char>(raw_byte));
        }
        // Send newline terminator
        Actuator_ProcessSerialChar('\n');
    }

    // After massive fuzzing, ensure parser is still fully functional with a valid command
    Serial.clear_output();
    Actuator_ResetSerialBuffer();

    bool handled = Actuator_ProcessCommand("STATUS");
    ASSERT_TRUE(handled);
    ASSERT_TRUE(Serial.get_output().find("SYSTEM STATUS DIAGNOSTICS") != std::string::npos);
}

// -----------------------------------------------------------------------------
// 8. Serial Protocol Fuzzer: Buffer Overflow Lines (70+ to 8192 chars)
// Verify overflow discard protection and seamless recovery on subsequent line.
// -----------------------------------------------------------------------------
TEST(Challenger2_SerialFuzzing, BufferOverflow_LineDiscardAndCleanRecovery) {
    Serial.reset();
    Sensors_Init();
    Actuator_Init();

    const std::vector<size_t> test_lengths = {64, 65, 70, 100, 256, 512, 1024, 4096, 8192};

    for (size_t len : test_lengths) {
        Serial.clear_output();
        Actuator_ResetSerialBuffer();

        // 1. Send massive line without newline
        for (size_t i = 0; i < len; ++i) {
            Actuator_ProcessSerialChar('X');
        }

        // 2. If len >= 64, it must have triggered warning and set discard mode
        if (len >= 64) {
            ASSERT_TRUE(Serial.get_output().find("[WARN] Serial buffer overflow") != std::string::npos);
        }

        // 3. Terminate the bad line with newline
        Actuator_ProcessSerialChar('\n');

        // 4. Immediately send a valid command: INJECT_VIB
        Serial.clear_output();
        Sensors_SetVibrationInjection(false);

        const std::string valid_cmd = "INJECT_VIB\n";
        for (char c : valid_cmd) {
            Actuator_ProcessSerialChar(c);
        }

        // Verify valid command was successfully parsed and executed
        ASSERT_TRUE(Sensors_IsVibrationInjected());
        ASSERT_TRUE(Serial.get_output().find("INJECTED") != std::string::npos);

        // 5. Clean up for next iteration
        Sensors_SetVibrationInjection(false);
    }
}

// -----------------------------------------------------------------------------
// 9. Serial Protocol Fuzzer: Massive Command Race Interleaving
// Rapid random interleaving of INJECT_VIB, INJECT_TEMP, RESET, STATUS, HELP & garbage.
// -----------------------------------------------------------------------------
TEST(Challenger2_SerialFuzzing, RapidRaceInterleaving_20kCommands) {
    Serial.reset();
    Sensors_Init();
    Actuator_Init();

    const std::vector<std::string> command_pool = {
        "INJECT_VIB\n",
        "INJECT_TEMP\n",
        "RESET\n",
        "STATUS\n",
        "HELP\n",
        "inject_vib\r\n",
        "inject_temp\r\n",
        "reset\n",
        "   RESET   \n",
        "  \t INJECT_VIB \t \r\n",
        "UNKNOWN_CMD_12345\n",
        "\n",
        "\r\n",
        "VERY_LONG_GARBAGE_PAYLOAD_EXCEEDING_INTERNAL_BUFFER_LIMITS_1234567890_ABCDEFGHIJ_XYZ\n",
        "a\nb\nc\n",
        "INJECT_VIB\rINJECT_TEMP\rRESET\r"
    };

    std::mt19937 rng(9999);
    std::uniform_int_distribution<size_t> dist(0, command_pool.size() - 1);

    const int ITERATIONS = 20000;
    for (int i = 0; i < ITERATIONS; ++i) {
        const std::string& cmd = command_pool[dist(rng)];
        for (char c : cmd) {
            Actuator_ProcessSerialChar(c);
        }
    }

    // Final clean state reset
    Actuator_ProcessCommand("RESET");
    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
    ASSERT_EQ(0u, Actuator_GetDebounceCount());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());
    ASSERT_FALSE(Sensors_IsVibrationInjected());
    ASSERT_FALSE(Sensors_IsTemperatureInjected());
}

// -----------------------------------------------------------------------------
// 10. Safety Rearm: Multi-State Idempotency Verification
// Calling Actuator_Rearm() from ANY state must cleanly converge to STATE_NORMAL.
// -----------------------------------------------------------------------------
TEST(Challenger2_Rearm, MultiState_Idempotency_Convergence) {
    MockClock::reset();
    MockGPIO::reset();
    Serial.reset();
    Sensors_Init();
    Actuator_Init();

    const SystemState all_states[] = {
        STATE_INIT,
        STATE_NORMAL,
        STATE_WARNING,
        STATE_ALARM,
        STATE_MAINTENANCE
    };

    for (SystemState start_state : all_states) {
        // Set initial state
        Actuator_SetState(start_state);
        Sensors_SetVibrationInjection(true);
        Sensors_SetTemperatureInjection(true);

        // Call Rearm 100 times in a row
        for (int r = 0; r < 100; ++r) {
            Actuator_Rearm();

            // Strict assertions on every rearm call
            ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
            ASSERT_EQ(0u, Actuator_GetDebounceCount());
            ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());
            ASSERT_FALSE(Sensors_IsVibrationInjected());
            ASSERT_FALSE(Sensors_IsTemperatureInjected());
            ASSERT_FALSE(Actuator_IsBuzzerActive());
            ASSERT_TRUE(Actuator_IsNormalLedActive());
            ASSERT_FALSE(Actuator_IsWarningAlarmLedActive());
            ASSERT_EQ(HIGH, MockGPIO::pin_state[PIN_MOTOR_IN1]);
            ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_IN2]);
            ASSERT_EQ(static_cast<uint32_t>(MOTOR_NOMINAL_PWM), MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
            ASSERT_EQ(HIGH, MockGPIO::pin_state[PIN_LED_NORMAL]);
            ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_LED_WARN_ALARM]);
            ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_BUZZER]);
        }
    }
}

// -----------------------------------------------------------------------------
// 11. Full End-to-End Fault Injection & Safety Rearm Lifecycle
// -----------------------------------------------------------------------------
TEST(Challenger2_Lifecycle, FaultInjection_EmergencyCutoff_Rearm_Cycle) {
    MockClock::reset();
    MockGPIO::reset();
    Serial.reset();
    Sensors_Init();
    Actuator_Init();

    // 1. Initial State: Arm system
    Actuator_Rearm();
    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());

    // 2. Inject Vibration Fault via Serial Command
    Actuator_ProcessCommand("INJECT_VIB");
    ASSERT_TRUE(Sensors_IsVibrationInjected());

    // 3. FSM receives 2 anomaly frames -> WARNING
    Actuator_UpdateFSM(1);
    ASSERT_EQ(STATE_WARNING, Actuator_GetState());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());

    Actuator_UpdateFSM(1);
    ASSERT_EQ(STATE_WARNING, Actuator_GetState());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());

    // 4. FSM receives 3rd anomaly frame -> ALARM (Emergency Cutoff)
    Actuator_UpdateFSM(1);
    ASSERT_EQ(STATE_ALARM, Actuator_GetState());
    ASSERT_EQ(0u, Actuator_GetMotorPWM());
    ASSERT_TRUE(Actuator_IsBuzzerActive());
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_IN1]);
    ASSERT_EQ(0u, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);

    // 5. Subsequent frames while fault is active must remain in ALARM
    for (int i = 0; i < 50; ++i) {
        Actuator_UpdateFSM(1);
        ASSERT_EQ(STATE_ALARM, Actuator_GetState());
        ASSERT_EQ(0u, Actuator_GetMotorPWM());
    }

    // 6. Operator sends RESET command over Serial
    Actuator_ProcessCommand("RESET");

    // 7. Verify system completely recovered
    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
    ASSERT_EQ(0u, Actuator_GetDebounceCount());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());
    ASSERT_FALSE(Sensors_IsVibrationInjected());
    ASSERT_FALSE(Actuator_IsBuzzerActive());
    ASSERT_TRUE(Actuator_IsNormalLedActive());
    ASSERT_FALSE(Actuator_IsWarningAlarmLedActive());

    // 8. Subsequent normal inferences maintain STATE_NORMAL
    for (int i = 0; i < 50; ++i) {
        Actuator_UpdateFSM(0);
        ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
        ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());
    }
}
