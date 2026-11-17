#pragma once
#include <Arduino.h>
#include <cstdint>
#include <cstddef>
#include "config.h"
#include "dsp_pipeline.h"
#include "sensors.h"

/**
 * @brief System Operating States for Safety State Machine (FSM).
 */
enum SystemState {
    STATE_INIT = 0,         // Hardware initialization & calibration
    STATE_NORMAL = 1,       // Nominal operation (Class 0, motor running nominal)
    STATE_WARNING = 2,      // 1 or 2 consecutive anomaly frames (Debounce count < 3)
    STATE_ALARM = 3,        // Exactly 3 consecutive anomaly frames (Latched Emergency Stop)
    STATE_MAINTENANCE = 4   // Manual inspection / maintenance mode
};

/**
 * @brief Convert SystemState enum to printable string name.
 * @param state Target SystemState.
 * @return Null-terminated string representation ("INIT", "NORMAL", "WARNING", "ALARM", "MAINTENANCE").
 */
const char* StateToString(SystemState state);

/**
 * @brief Initialize motor driver GPIOs, PWM channels, buzzer, and status LEDs.
 */
void Actuator_Init();

/**
 * @brief Update safety FSM based on TinyML inference output with 3-frame debounce.
 * @param inference_result Prediction class (0 = NORMAL, 1 = ANOMALY).
 */
void Actuator_UpdateFSM(int inference_result);

/**
 * @brief Immediate synchronous emergency cutoff (< 10 µs software, < 100 ms hardware).
 * Cuts motor PWM to 0, pulls IN1/IN2/ENA LOW, activates alarm LED and buzzer, and latches STATE_ALARM.
 */
void Actuator_EmergencyStop();

/**
 * @brief Operator safety rearm & system reset.
 * Clears fault injections, resets debounce counter, silences buzzer, turns on green LED,
 * soft-starts the DC motor, and transitions to STATE_NORMAL.
 */
void Actuator_Rearm();

/**
 * @brief Set DC motor PWM duty cycle directly.
 * @param duty 8-bit duty cycle (0 = stop, 255 = maximum speed).
 */
void Actuator_SetMotorPWM(uint8_t duty);

/**
 * @brief Soft-start DC motor by ramping PWM to target duty.
 * @param target_pwm Target duty cycle (default: MOTOR_NOMINAL_PWM = 200).
 */
void Actuator_SoftStartMotor(uint8_t target_pwm = MOTOR_NOMINAL_PWM);

/**
 * @brief Get the current FSM system state.
 */
SystemState Actuator_GetState();

/**
 * @brief Force set the FSM system state and apply corresponding actuator outputs.
 * @param new_state Target SystemState.
 */
void Actuator_SetState(SystemState new_state);

/**
 * @brief Get current debounce anomaly counter value (0 to DEBOUNCE_THRESHOLD).
 */
uint8_t Actuator_GetDebounceCount();

/**
 * @brief Reset the debounce anomaly counter to zero.
 */
void Actuator_ResetDebounce();

/**
 * @brief Get the currently commanded motor PWM duty cycle.
 */
uint8_t Actuator_GetMotorPWM();

/**
 * @brief Check if acoustic alarm buzzer is currently active.
 */
bool Actuator_IsBuzzerActive();

/**
 * @brief Check if green status LED is currently active.
 */
bool Actuator_IsNormalLedActive();

/**
 * @brief Check if warning/alarm LED is currently active.
 */
bool Actuator_IsWarningAlarmLedActive();

/**
 * @brief Update status LED and buzzer outputs based on current FSM state.
 */
void Actuator_UpdateSignaling();

/**
 * @brief Format a single telemetry stream line into a user-provided buffer (zero dynamic allocation).
 * Format: [ETAT: %-7s] | RMS: %.3f | Peak: %.3f | Freq: %.2f | Temp: %.2f | Inference: %d
 */
void Actuator_FormatTelemetry(char* buffer, size_t max_len, SystemState state,
                              float rms, float peak, float freq, float temp, int inference);

/**
 * @brief Emit formatted telemetry line over Serial.
 */
void Actuator_PrintTelemetry(float rms, float peak, float freq, float temp, int inference);

/**
 * @brief Emit formatted telemetry line from DSPFeatures and inference result over Serial.
 */
void Actuator_PrintTelemetry(const DSPFeatures& features, int inference);

/**
 * @brief Output complete internal diagnostics and telemetry over Serial.
 */
void Actuator_PrintStatus();

/**
 * @brief Output interactive command menu over Serial.
 */
void Actuator_PrintHelp();

/**
 * @brief Process a single received serial character through the non-blocking command buffer.
 * @param c Incoming character from UART.
 */
void Actuator_ProcessSerialChar(char c);

/**
 * @brief Non-blocking read and process of all pending serial characters.
 */
void Actuator_ProcessSerial();

/**
 * @brief Process and execute a null-terminated command string directly.
 * Supported commands: INJECT_VIB, INJECT_TEMP, RESET, STATUS, HELP.
 * @param cmd Null-terminated command string.
 * @return True if command recognized and executed, false otherwise.
 */
bool Actuator_ProcessCommand(const char* cmd);

/**
 * @brief Clear internal serial reception buffer.
 */
void Actuator_ResetSerialBuffer();
