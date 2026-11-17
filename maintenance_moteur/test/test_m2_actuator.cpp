#include "test_framework.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include "config.h"
#include "sensors.h"
#include "dsp_pipeline.h"
#include "actuator_control.h"

// =============================================================================
// Milestone M2 Unit Test Suite: Actuator Control, Safety FSM & Serial Parser
// =============================================================================

TEST(M2_Actuator, InitStateAndOutputs) {
    MockClock::reset();
    MockGPIO::reset();
    Serial.reset();

    Actuator_Init();

    ASSERT_EQ(STATE_INIT, Actuator_GetState());
    ASSERT_EQ(0u, Actuator_GetDebounceCount());
    ASSERT_EQ(0u, Actuator_GetMotorPWM());
    ASSERT_FALSE(Actuator_IsBuzzerActive());
    ASSERT_FALSE(Actuator_IsNormalLedActive());
    ASSERT_FALSE(Actuator_IsWarningAlarmLedActive());

    // Hardware GPIO checks
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_IN1]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_IN2]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_ENA]);
    ASSERT_EQ(0u, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_LED_NORMAL]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_LED_WARN_ALARM]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_BUZZER]);
}

TEST(M2_Actuator, StateToString_AllStates) {
    ASSERT_EQ(std::string("INIT"), std::string(StateToString(STATE_INIT)));
    ASSERT_EQ(std::string("NORMAL"), std::string(StateToString(STATE_NORMAL)));
    ASSERT_EQ(std::string("WARNING"), std::string(StateToString(STATE_WARNING)));
    ASSERT_EQ(std::string("ALARM"), std::string(StateToString(STATE_ALARM)));
    ASSERT_EQ(std::string("MAINTENANCE"), std::string(StateToString(STATE_MAINTENANCE)));
    ASSERT_EQ(std::string("UNKNOWN"), std::string(StateToString(static_cast<SystemState>(99))));
}

TEST(M2_Actuator, FSM_NormalOperation) {
    MockGPIO::reset();
    Actuator_Init();

    // In INIT state, receiving normal inference (0) transitions to NORMAL
    Actuator_UpdateFSM(0);

    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
    ASSERT_EQ(0u, Actuator_GetDebounceCount());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());
    ASSERT_TRUE(Actuator_IsNormalLedActive());
    ASSERT_FALSE(Actuator_IsWarningAlarmLedActive());
    ASSERT_FALSE(Actuator_IsBuzzerActive());

    // Check GPIOs
    ASSERT_EQ(HIGH, MockGPIO::pin_state[PIN_MOTOR_IN1]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_IN2]);
    ASSERT_EQ(static_cast<uint32_t>(MOTOR_NOMINAL_PWM), MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
    ASSERT_EQ(HIGH, MockGPIO::pin_state[PIN_LED_NORMAL]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_LED_WARN_ALARM]);
}

TEST(M2_Actuator, FSM_Debounce_1_2_3_Anomalies_Triggers_Alarm) {
    MockGPIO::reset();
    Actuator_Init();

    // Start in NORMAL state
    Actuator_UpdateFSM(0);
    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());

    // 1st anomaly -> STATE_WARNING (debounce = 1)
    Actuator_UpdateFSM(1);
    ASSERT_EQ(STATE_WARNING, Actuator_GetState());
    ASSERT_EQ(1u, Actuator_GetDebounceCount());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM()); // Motor still running
    ASSERT_TRUE(Actuator_IsNormalLedActive());
    ASSERT_TRUE(Actuator_IsWarningAlarmLedActive());
    ASSERT_FALSE(Actuator_IsBuzzerActive());

    // 2nd anomaly -> STATE_WARNING (debounce = 2)
    Actuator_UpdateFSM(1);
    ASSERT_EQ(STATE_WARNING, Actuator_GetState());
    ASSERT_EQ(2u, Actuator_GetDebounceCount());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM()); // Motor still running

    // 3rd consecutive anomaly -> STATE_ALARM with Emergency Stop!
    Actuator_UpdateFSM(1);
    ASSERT_EQ(STATE_ALARM, Actuator_GetState());
    ASSERT_EQ(3u, Actuator_GetDebounceCount());
    ASSERT_EQ(0u, Actuator_GetMotorPWM()); // Motor CUT to 0
    ASSERT_FALSE(Actuator_IsNormalLedActive());
    ASSERT_TRUE(Actuator_IsWarningAlarmLedActive());
    ASSERT_TRUE(Actuator_IsBuzzerActive());

    // Verify hardware pins immediately cut
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_IN1]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_IN2]);
    ASSERT_EQ(0u, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
    ASSERT_EQ(HIGH, MockGPIO::pin_state[PIN_LED_WARN_ALARM]);
    ASSERT_EQ(HIGH, MockGPIO::pin_state[PIN_BUZZER]);
}

TEST(M2_Actuator, FSM_DebounceRecovery_SpikeDoesNotTriggerAlarm) {
    MockGPIO::reset();
    Actuator_Init();

    Actuator_UpdateFSM(0); // NORMAL
    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());

    // Frame 1: Anomaly -> WARNING (debounce = 1)
    Actuator_UpdateFSM(1);
    ASSERT_EQ(STATE_WARNING, Actuator_GetState());
    ASSERT_EQ(1u, Actuator_GetDebounceCount());

    // Frame 2: Normal -> returns to NORMAL (debounce reset to 0)
    Actuator_UpdateFSM(0);
    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
    ASSERT_EQ(0u, Actuator_GetDebounceCount());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());

    // Frame 3: Anomaly -> WARNING (debounce = 1)
    Actuator_UpdateFSM(1);
    ASSERT_EQ(STATE_WARNING, Actuator_GetState());
    ASSERT_EQ(1u, Actuator_GetDebounceCount());

    // Frame 4: Anomaly -> WARNING (debounce = 2)
    Actuator_UpdateFSM(1);
    ASSERT_EQ(STATE_WARNING, Actuator_GetState());
    ASSERT_EQ(2u, Actuator_GetDebounceCount());

    // Frame 5: Normal -> returns to NORMAL (debounce reset to 0)
    Actuator_UpdateFSM(0);
    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
    ASSERT_EQ(0u, Actuator_GetDebounceCount());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM()); // Never stopped
}

TEST(M2_Actuator, FSM_LatchedAlarmState) {
    MockGPIO::reset();
    Actuator_Init();

    // Trigger Alarm via 3 consecutive anomalies
    Actuator_UpdateFSM(1);
    Actuator_UpdateFSM(1);
    Actuator_UpdateFSM(1);
    ASSERT_EQ(STATE_ALARM, Actuator_GetState());
    ASSERT_EQ(0u, Actuator_GetMotorPWM());
    ASSERT_TRUE(Actuator_IsBuzzerActive());

    // Subsequent normal frames must NOT leave ALARM state
    Actuator_UpdateFSM(0);
    ASSERT_EQ(STATE_ALARM, Actuator_GetState());
    ASSERT_EQ(0u, Actuator_GetMotorPWM());
    ASSERT_TRUE(Actuator_IsBuzzerActive());

    Actuator_UpdateFSM(0);
    ASSERT_EQ(STATE_ALARM, Actuator_GetState());

    Actuator_UpdateFSM(1);
    ASSERT_EQ(STATE_ALARM, Actuator_GetState());
}

TEST(M2_Actuator, EmergencyStop_DirectCall) {
    MockGPIO::reset();
    Actuator_Init();

    Actuator_UpdateFSM(0); // In NORMAL, motor running
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());

    // Direct Emergency Stop
    Actuator_EmergencyStop();

    ASSERT_EQ(STATE_ALARM, Actuator_GetState());
    ASSERT_EQ(0u, Actuator_GetMotorPWM());
    ASSERT_FALSE(Actuator_IsNormalLedActive());
    ASSERT_TRUE(Actuator_IsWarningAlarmLedActive());
    ASSERT_TRUE(Actuator_IsBuzzerActive());

    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_IN1]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[PIN_MOTOR_IN2]);
    ASSERT_EQ(0u, MockGPIO::pin_pwm[PIN_MOTOR_ENA]);
}

TEST(M2_Actuator, Rearm_RestoresNormalStateAndMotor) {
    MockGPIO::reset();
    Serial.reset();
    Sensors_Init();
    Actuator_Init();

    // Set injection and trigger alarm
    Sensors_SetVibrationInjection(true);
    Sensors_SetTemperatureInjection(true);
    Actuator_EmergencyStop();

    ASSERT_EQ(STATE_ALARM, Actuator_GetState());
    ASSERT_TRUE(Sensors_IsVibrationInjected());
    ASSERT_TRUE(Sensors_IsTemperatureInjected());

    // Rearm
    Actuator_Rearm();

    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
    ASSERT_EQ(0u, Actuator_GetDebounceCount());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());
    ASSERT_FALSE(Sensors_IsVibrationInjected());
    ASSERT_FALSE(Sensors_IsTemperatureInjected());
    ASSERT_FALSE(Actuator_IsBuzzerActive());
    ASSERT_TRUE(Actuator_IsNormalLedActive());
    ASSERT_FALSE(Actuator_IsWarningAlarmLedActive());

    // Check UART rearm message
    std::string serial_out = Serial.get_output();
    ASSERT_TRUE(serial_out.find("REARMED") != std::string::npos);
}

TEST(M2_Actuator, TelemetryFormatting) {
    char buf[128];

    Actuator_FormatTelemetry(buf, sizeof(buf), STATE_NORMAL, 0.142f, 0.380f, 25.00f, 24.50f, 0);
    ASSERT_EQ(std::string("[ETAT: NORMAL] | RMS: 0.142 | Peak: 0.380 | Freq: 25.00 | Temp: 24.50 | Inference: 0"),
              std::string(buf));

    Actuator_FormatTelemetry(buf, sizeof(buf), STATE_WARNING, 0.890f, 1.750f, 62.50f, 26.00f, 1);
    ASSERT_EQ(std::string("[ETAT: WARNING] | RMS: 0.890 | Peak: 1.750 | Freq: 62.50 | Temp: 26.00 | Inference: 1"),
              std::string(buf));

    Actuator_FormatTelemetry(buf, sizeof(buf), STATE_ALARM, 2.340f, 4.820f, 65.00f, 78.00f, 1);
    ASSERT_EQ(std::string("[ETAT: ALARM] | RMS: 2.340 | Peak: 4.820 | Freq: 65.00 | Temp: 78.00 | Inference: 1"),
              std::string(buf));
}

TEST(M2_Actuator, TelemetryPrintOverSerial) {
    Serial.reset();
    Actuator_Init();
    Actuator_UpdateFSM(0); // NORMAL

    DSPFeatures feat;
    feat.rms = 0.155f;
    feat.peak_absolute = 0.420f;
    feat.dominant_frequency = 25.0f;
    feat.temperature = 28.0f;

    Actuator_PrintTelemetry(feat, 0);
    std::string out = Serial.get_output();
    ASSERT_TRUE(out.find("[ETAT: NORMAL]") != std::string::npos);
    ASSERT_TRUE(out.find("RMS: 0.155") != std::string::npos);
    ASSERT_TRUE(out.find("Peak: 0.420") != std::string::npos);
    ASSERT_TRUE(out.find("Freq: 25.00") != std::string::npos);
    ASSERT_TRUE(out.find("Temp: 28.00") != std::string::npos);
    ASSERT_TRUE(out.find("Inference: 0") != std::string::npos);
}

TEST(M2_Actuator, SerialParser_InjectVibCommand) {
    Serial.reset();
    Sensors_Init();
    Sensors_SetVibrationInjection(false);

    bool handled = Actuator_ProcessCommand("INJECT_VIB");
    ASSERT_TRUE(handled);
    ASSERT_TRUE(Sensors_IsVibrationInjected());
    ASSERT_TRUE(Serial.get_output().find("INJECTED") != std::string::npos);

    // Toggle off
    Serial.clear_output();
    handled = Actuator_ProcessCommand("inject_vib"); // Case insensitive
    ASSERT_TRUE(handled);
    ASSERT_FALSE(Sensors_IsVibrationInjected());
    ASSERT_TRUE(Serial.get_output().find("CLEARED") != std::string::npos);
}

TEST(M2_Actuator, SerialParser_InjectTempCommand) {
    Serial.reset();
    Sensors_Init();
    Sensors_SetTemperatureInjection(false);

    bool handled = Actuator_ProcessCommand("  INJECT_TEMP \r\n"); // Trimming test
    ASSERT_TRUE(handled);
    ASSERT_TRUE(Sensors_IsTemperatureInjected());
    ASSERT_TRUE(Serial.get_output().find("INJECTED") != std::string::npos);

    // Toggle off
    Serial.clear_output();
    handled = Actuator_ProcessCommand("inject_temp");
    ASSERT_TRUE(handled);
    ASSERT_FALSE(Sensors_IsTemperatureInjected());
    ASSERT_TRUE(Serial.get_output().find("CLEARED") != std::string::npos);
}

TEST(M2_Actuator, SerialParser_ResetCommand) {
    Serial.reset();
    Sensors_Init();
    Actuator_Init();

    // Trigger Alarm
    Actuator_EmergencyStop();
    ASSERT_EQ(STATE_ALARM, Actuator_GetState());

    // Send RESET command
    bool handled = Actuator_ProcessCommand("RESET");
    ASSERT_TRUE(handled);
    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());
}

TEST(M2_Actuator, SerialParser_StatusAndHelpCommands) {
    Serial.reset();
    Sensors_Init();
    Actuator_Init();

    bool handled = Actuator_ProcessCommand("STATUS");
    ASSERT_TRUE(handled);
    std::string out = Serial.get_output();
    ASSERT_TRUE(out.find("SYSTEM STATUS DIAGNOSTICS") != std::string::npos);
    ASSERT_TRUE(out.find("FSM State") != std::string::npos);
    ASSERT_TRUE(out.find("Debounce Count") != std::string::npos);

    Serial.clear_output();
    handled = Actuator_ProcessCommand("HELP");
    ASSERT_TRUE(handled);
    out = Serial.get_output();
    ASSERT_TRUE(out.find("COMMAND MENU") != std::string::npos);
    ASSERT_TRUE(out.find("INJECT_VIB") != std::string::npos);
    ASSERT_TRUE(out.find("RESET") != std::string::npos);
}

TEST(M2_Actuator, SerialParser_UnknownAndEmptyCommands) {
    Serial.reset();

    // Empty command
    ASSERT_TRUE(Actuator_ProcessCommand(""));
    ASSERT_TRUE(Actuator_ProcessCommand("   \n"));

    // Unknown command
    Serial.clear_output();
    ASSERT_FALSE(Actuator_ProcessCommand("FOOBAR_123"));
    ASSERT_TRUE(Serial.get_output().find("[ERROR] Unknown command") != std::string::npos);
}

TEST(M2_Actuator, SerialStream_InteractiveExecution) {
    Serial.reset();
    Sensors_Init();
    Actuator_Init();

    // Inject multiple commands sequentially in stream
    Serial.inject_input("INJECT_VIB\nINJECT_TEMP\nSTATUS\nRESET\n");

    Actuator_ProcessSerial();

    ASSERT_EQ(0, Serial.available());
    ASSERT_EQ(STATE_NORMAL, Actuator_GetState());
    ASSERT_EQ(MOTOR_NOMINAL_PWM, Actuator_GetMotorPWM());
}

TEST(M2_Actuator, SerialParser_BufferOverflowResilience) {
    Serial.reset();
    Actuator_ResetSerialBuffer();

    // Send 70 'A' characters without newline (buffer is 64 bytes)
    for (int i = 0; i < 70; ++i) {
        Actuator_ProcessSerialChar('A');
    }
    // End the oversized line with newline
    Actuator_ProcessSerialChar('\n');

    std::string warn_out = Serial.get_output();
    ASSERT_TRUE(warn_out.find("[WARN] Serial buffer overflow") != std::string::npos);

    // Now send valid command
    Serial.clear_output();
    Actuator_ProcessSerialChar('H');
    Actuator_ProcessSerialChar('E');
    Actuator_ProcessSerialChar('L');
    Actuator_ProcessSerialChar('P');
    Actuator_ProcessSerialChar('\n');

    std::string out = Serial.get_output();
    ASSERT_TRUE(out.find("COMMAND MENU") != std::string::npos);
}
