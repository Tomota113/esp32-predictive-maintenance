#include "actuator_control.h"
#include <cstdio>
#include <cstring>
#include <cctype>
#include <strings.h>

// =============================================================================
// Internal Static Subsystem State (Zero Dynamic Memory Allocation)
// =============================================================================
static SystemState s_current_state = STATE_INIT;
static uint8_t s_debounce_count = 0;
static uint8_t s_motor_pwm = 0;
static bool s_buzzer_active = false;
static bool s_led_normal_active = false;
static bool s_led_warn_alarm_active = false;

// Serial RX Buffer (64 bytes static circular/line buffer)
static char s_rx_buffer[64];
static uint8_t s_rx_index = 0;
static bool s_rx_overflow_discard = false;

// =============================================================================
// Helper Utilities
// =============================================================================
static void trim_whitespace(char* str) {
    if (str == nullptr) return;
    
    // Trim leading whitespace
    char* start = str;
    while (*start != '\0' && (isspace((unsigned char)*start) || *start == '\r' || *start == '\n')) {
        start++;
    }
    
    // Trim trailing whitespace
    char* end = start + strlen(start);
    while (end > start && (isspace((unsigned char)*(end - 1)) || *(end - 1) == '\r' || *(end - 1) == '\n')) {
        end--;
    }
    *end = '\0';

    // Move trimmed content to beginning if needed
    if (start != str) {
        memmove(str, start, (end - start) + 1);
    }
}

// =============================================================================
// State Enum to String Conversion
// =============================================================================
const char* StateToString(SystemState state) {
    switch (state) {
        case STATE_INIT:        return "INIT";
        case STATE_NORMAL:      return "NORMAL";
        case STATE_WARNING:     return "WARNING";
        case STATE_ALARM:       return "ALARM";
        case STATE_MAINTENANCE: return "MAINTENANCE";
        default:                return "UNKNOWN";
    }
}

// =============================================================================
// Actuator Initialization
// =============================================================================
void Actuator_Init() {
    // Configure Direction and Signaling GPIOs as OUTPUT
    pinMode(PIN_MOTOR_IN1, OUTPUT);
    pinMode(PIN_MOTOR_IN2, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_LED_NORMAL, OUTPUT);
    pinMode(PIN_LED_WARN_ALARM, OUTPUT);

    // Configure ESP32 LEDC PWM on ENA pin
    ledcSetup(PWM_CHANNEL_MOTOR, PWM_FREQ_MOTOR, PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_MOTOR_ENA, PWM_CHANNEL_MOTOR);

    // Initial hardware shutoff
    ledcWrite(PWM_CHANNEL_MOTOR, 0);
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, LOW);

    digitalWrite(PIN_LED_NORMAL, LOW);
    digitalWrite(PIN_LED_WARN_ALARM, LOW);
    noTone(PIN_BUZZER);
    digitalWrite(PIN_BUZZER, LOW);

    s_current_state = STATE_INIT;
    s_debounce_count = 0;
    s_motor_pwm = 0;
    s_buzzer_active = false;
    s_led_normal_active = false;
    s_led_warn_alarm_active = false;
    s_rx_index = 0;
    s_rx_overflow_discard = false;
    std::memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
}

// =============================================================================
// Motor Actuation Control
// =============================================================================
void Actuator_SetMotorPWM(uint8_t duty) {
    s_motor_pwm = duty;
    if (duty > 0) {
        // Forward rotation
        digitalWrite(PIN_MOTOR_IN1, HIGH);
        digitalWrite(PIN_MOTOR_IN2, LOW);
        ledcWrite(PWM_CHANNEL_MOTOR, duty);
    } else {
        // Motor brake / cut
        ledcWrite(PWM_CHANNEL_MOTOR, 0);
        digitalWrite(PIN_MOTOR_IN1, LOW);
        digitalWrite(PIN_MOTOR_IN2, LOW);
    }
}

void Actuator_SoftStartMotor(uint8_t target_pwm) {
    digitalWrite(PIN_MOTOR_IN1, HIGH);
    digitalWrite(PIN_MOTOR_IN2, LOW);

    if (target_pwm > 0) {
        // Progressive ramp-up in 4 increments to reduce startup inrush current
        uint8_t step1 = target_pwm / 4;
        uint8_t step2 = target_pwm / 2;
        uint8_t step3 = (target_pwm * 3) / 4;

        ledcWrite(PWM_CHANNEL_MOTOR, step1);
        delayMicroseconds(500);
        ledcWrite(PWM_CHANNEL_MOTOR, step2);
        delayMicroseconds(500);
        ledcWrite(PWM_CHANNEL_MOTOR, step3);
        delayMicroseconds(500);
        ledcWrite(PWM_CHANNEL_MOTOR, target_pwm);
    } else {
        ledcWrite(PWM_CHANNEL_MOTOR, 0);
    }
    s_motor_pwm = target_pwm;
}

// =============================================================================
// Signaling Control
// =============================================================================
void Actuator_UpdateSignaling() {
    switch (s_current_state) {
        case STATE_INIT:
            s_led_normal_active = false;
            s_led_warn_alarm_active = false;
            s_buzzer_active = false;
            digitalWrite(PIN_LED_NORMAL, LOW);
            digitalWrite(PIN_LED_WARN_ALARM, LOW);
            noTone(PIN_BUZZER);
            digitalWrite(PIN_BUZZER, LOW);
            break;

        case STATE_NORMAL:
            s_led_normal_active = true;
            s_led_warn_alarm_active = false;
            s_buzzer_active = false;
            digitalWrite(PIN_LED_NORMAL, HIGH);
            digitalWrite(PIN_LED_WARN_ALARM, LOW);
            noTone(PIN_BUZZER);
            digitalWrite(PIN_BUZZER, LOW);
            break;

        case STATE_WARNING:
            s_led_normal_active = true;
            s_led_warn_alarm_active = true;
            s_buzzer_active = false; // Intermittent warning / amber signal
            digitalWrite(PIN_LED_NORMAL, HIGH);
            digitalWrite(PIN_LED_WARN_ALARM, HIGH);
            noTone(PIN_BUZZER);
            digitalWrite(PIN_BUZZER, LOW);
            break;

        case STATE_ALARM:
            s_led_normal_active = false;
            s_led_warn_alarm_active = true;
            s_buzzer_active = true;
            digitalWrite(PIN_LED_NORMAL, LOW);
            digitalWrite(PIN_LED_WARN_ALARM, HIGH);
            tone(PIN_BUZZER, 1000);
            break;

        case STATE_MAINTENANCE:
            s_led_normal_active = false;
            s_led_warn_alarm_active = true;
            s_buzzer_active = false;
            digitalWrite(PIN_LED_NORMAL, LOW);
            digitalWrite(PIN_LED_WARN_ALARM, HIGH);
            noTone(PIN_BUZZER);
            digitalWrite(PIN_BUZZER, LOW);
            break;
    }
}

// =============================================================================
// Emergency Stop (< 10 µs software latency, latched STATE_ALARM)
// =============================================================================
void Actuator_EmergencyStop() {
    // Immediate synchronous GPIO and PWM cutoff
    ledcWrite(PWM_CHANNEL_MOTOR, 0);
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, LOW);
    s_motor_pwm = 0;

    // Trigger visual and acoustic alarm
    digitalWrite(PIN_LED_NORMAL, LOW);
    digitalWrite(PIN_LED_WARN_ALARM, HIGH);
    tone(PIN_BUZZER, 1000);

    s_led_normal_active = false;
    s_led_warn_alarm_active = true;
    s_buzzer_active = true;

    // Latch emergency alarm state
    s_current_state = STATE_ALARM;
}

// =============================================================================
// Safety Rearm & Reset
// =============================================================================
void Actuator_Rearm() {
    // 1. Reset debounce anomaly counter
    s_debounce_count = 0;

    // 2. Clear fault injection flags in sensor subsystem
    Sensors_SetVibrationInjection(false);
    Sensors_SetTemperatureInjection(false);

    // 3. Silence buzzer and turn off alarm LED
    noTone(PIN_BUZZER);
    digitalWrite(PIN_BUZZER, LOW);
    s_buzzer_active = false;

    digitalWrite(PIN_LED_WARN_ALARM, LOW);
    s_led_warn_alarm_active = false;

    // 4. Turn on green normal status LED
    digitalWrite(PIN_LED_NORMAL, HIGH);
    s_led_normal_active = true;

    // 5. Soft-start DC motor back to nominal operational PWM
    Actuator_SoftStartMotor(MOTOR_NOMINAL_PWM);

    // 6. Transition state machine to STATE_NORMAL
    s_current_state = STATE_NORMAL;

    // 7. Telemetry notification over Serial
    Serial.println("[SYSTEM] Safety State Machine REARMED. Motor restarted in STATE_NORMAL.");
}

// =============================================================================
// 3-Frame Debounce Anomaly Filter & FSM Update Engine
// =============================================================================
void Actuator_UpdateFSM(int inference_result) {
    // STATE_ALARM is LATCHED: system remains in ALARM until explicit operator RESET / Rearm
    if (s_current_state == STATE_ALARM) {
        // Ensure actuators remain cut off
        Actuator_EmergencyStop();
        return;
    }

    if (inference_result == 1) {
        // Anomaly frame detected
        s_debounce_count++;

        if (s_debounce_count >= DEBOUNCE_THRESHOLD) {
            // 3 consecutive anomaly frames -> IMMEDIATE EMERGENCY SHUTDOWN
            Actuator_EmergencyStop();
        } else {
            // 1 or 2 anomalies -> STATE_WARNING
            s_current_state = STATE_WARNING;
            Actuator_SetMotorPWM(MOTOR_NOMINAL_PWM);
            Actuator_UpdateSignaling();
        }
    } else {
        // Normal frame detected (0)
        s_debounce_count = 0;
        s_current_state = STATE_NORMAL;
        Actuator_SetMotorPWM(MOTOR_NOMINAL_PWM);
        Actuator_UpdateSignaling();
    }
}

// =============================================================================
// State & Counter Accessors
// =============================================================================
SystemState Actuator_GetState() {
    return s_current_state;
}

void Actuator_SetState(SystemState new_state) {
    s_current_state = new_state;
    if (new_state == STATE_ALARM) {
        Actuator_EmergencyStop();
    } else if (new_state == STATE_NORMAL) {
        Actuator_SetMotorPWM(MOTOR_NOMINAL_PWM);
        Actuator_UpdateSignaling();
    } else if (new_state == STATE_WARNING) {
        Actuator_SetMotorPWM(MOTOR_NOMINAL_PWM);
        Actuator_UpdateSignaling();
    } else {
        Actuator_SetMotorPWM(0);
        Actuator_UpdateSignaling();
    }
}

uint8_t Actuator_GetDebounceCount() {
    return s_debounce_count;
}

void Actuator_ResetDebounce() {
    s_debounce_count = 0;
}

uint8_t Actuator_GetMotorPWM() {
    return s_motor_pwm;
}

bool Actuator_IsBuzzerActive() {
    return s_buzzer_active;
}

bool Actuator_IsNormalLedActive() {
    return s_led_normal_active;
}

bool Actuator_IsWarningAlarmLedActive() {
    return s_led_warn_alarm_active;
}

// =============================================================================
// Telemetry Stream Formatting & Output
// =============================================================================
void Actuator_FormatTelemetry(char* buffer, size_t max_len, SystemState state,
                              float rms, float peak, float freq, float temp, int inference) {
    if (buffer == nullptr || max_len == 0) return;
    const char* state_str = StateToString(state);
    std::snprintf(buffer, max_len,
                  "[ETAT: %s] | RMS: %.3f | Peak: %.3f | Freq: %.2f | Temp: %.2f | Inference: %d",
                  state_str, rms, peak, freq, temp, inference);
}

void Actuator_PrintTelemetry(float rms, float peak, float freq, float temp, int inference) {
    char line_buf[128];
    Actuator_FormatTelemetry(line_buf, sizeof(line_buf), s_current_state, rms, peak, freq, temp, inference);
    Serial.println(line_buf);
}

void Actuator_PrintTelemetry(const DSPFeatures& features, int inference) {
    Actuator_PrintTelemetry(features.rms, features.peak_absolute, features.dominant_frequency, features.temperature, inference);
}

void Actuator_PrintStatus() {
    Serial.println("================ SYSTEM STATUS DIAGNOSTICS ================");
    Serial.print("FSM State        : ");
    Serial.println(StateToString(s_current_state));

    Serial.print("Debounce Count   : ");
    Serial.print(static_cast<int>(s_debounce_count));
    Serial.print(" / ");
    Serial.println(DEBOUNCE_THRESHOLD);

    Serial.print("Motor PWM Duty   : ");
    Serial.print(static_cast<int>(s_motor_pwm));
    Serial.print(" / 255 (State: ");
    if (s_current_state == STATE_ALARM) {
        Serial.println("EMERGENCY CUT)");
    } else if (s_motor_pwm > 0) {
        Serial.println("RUNNING)");
    } else {
        Serial.println("STOPPED)");
    }

    Serial.print("Signaling        : Green LED=");
    Serial.print(s_led_normal_active ? "ON" : "OFF");
    Serial.print(", Warn/Alarm LED=");
    Serial.print(s_led_warn_alarm_active ? "ON" : "OFF");
    Serial.print(", Buzzer=");
    Serial.println(s_buzzer_active ? "ON" : "OFF");

    Serial.print("Fault Injections : VIB=");
    Serial.print(Sensors_IsVibrationInjected() ? "ACTIVE" : "OFF");
    Serial.print(", TEMP=");
    Serial.println(Sensors_IsTemperatureInjected() ? "ACTIVE" : "OFF");

    Serial.print("Sensors Health   : MPU6050=");
    Serial.print(Sensors_IsMPUHealthy() ? "OK" : "FAULT");
    Serial.print(", DHT11=");
    Serial.println(Sensors_IsDHTHealthy() ? "OK" : "FAULT");
    Serial.println("===========================================================");
}

void Actuator_PrintHelp() {
    Serial.println("================== COMMAND MENU ==================");
    Serial.println("INJECT_VIB   : Toggle synthetic vibration fault (65 Hz, 3.5 m/s^2)");
    Serial.println("INJECT_TEMP  : Toggle thermal drift fault (80.0 C)");
    Serial.println("RESET        : Rearm safety FSM, reset debounce, restart motor");
    Serial.println("STATUS       : Display current subsystem diagnostics");
    Serial.println("HELP         : Display this command summary");
    Serial.println("==================================================");
}

// =============================================================================
// Non-Blocking Serial Command Parser
// =============================================================================
bool Actuator_ProcessCommand(const char* cmd) {
    if (cmd == nullptr) return false;

    // Make local copy for trimming (zero heap allocation)
    char clean_cmd[64];
    std::strncpy(clean_cmd, cmd, sizeof(clean_cmd) - 1);
    clean_cmd[sizeof(clean_cmd) - 1] = '\0';
    trim_whitespace(clean_cmd);

    // Empty line check
    if (clean_cmd[0] == '\0') {
        return true;
    }

    if (strcasecmp(clean_cmd, "INJECT_VIB") == 0) {
        bool current = Sensors_IsVibrationInjected();
        Sensors_SetVibrationInjection(!current);
        if (!current) {
            Serial.println("[FAULT INJECTION] Vibration fault INJECTED (65 Hz, 3.5 m/s^2).");
        } else {
            Serial.println("[FAULT INJECTION] Vibration fault CLEARED.");
        }
        return true;
    }

    if (strcasecmp(clean_cmd, "INJECT_TEMP") == 0) {
        bool current = Sensors_IsTemperatureInjected();
        Sensors_SetTemperatureInjection(!current);
        if (!current) {
            Serial.println("[FAULT INJECTION] Temperature drift INJECTED (80.0 C).");
        } else {
            Serial.println("[FAULT INJECTION] Temperature drift CLEARED.");
        }
        return true;
    }

    if (strcasecmp(clean_cmd, "RESET") == 0) {
        Actuator_Rearm();
        return true;
    }

    if (strcasecmp(clean_cmd, "STATUS") == 0) {
        Actuator_PrintStatus();
        return true;
    }

    if (strcasecmp(clean_cmd, "HELP") == 0) {
        Actuator_PrintHelp();
        return true;
    }

    // Unrecognized command
    Serial.print("[ERROR] Unknown command: '");
    Serial.print(clean_cmd);
    Serial.println("'. Type HELP for available commands.");
    return false;
}

void Actuator_ProcessSerialChar(char c) {
    if (c == '\r' || c == '\n') {
        if (s_rx_overflow_discard) {
            // Reached newline after overflow, reset discard flag
            s_rx_overflow_discard = false;
            s_rx_index = 0;
            std::memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
        } else if (s_rx_index > 0) {
            s_rx_buffer[s_rx_index] = '\0';
            Actuator_ProcessCommand(s_rx_buffer);
            s_rx_index = 0;
            std::memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
        }
    } else {
        if (s_rx_overflow_discard) {
            // Discarding oversized line until newline arrives
            return;
        }
        if (s_rx_index < sizeof(s_rx_buffer) - 1) {
            s_rx_buffer[s_rx_index++] = c;
        } else {
            // Buffer overflow: safely discard this line until next newline
            s_rx_overflow_discard = true;
            s_rx_index = 0;
            std::memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
            Serial.println("[WARN] Serial buffer overflow, line discarded.");
        }
    }
}

void Actuator_ProcessSerial() {
    while (Serial.available() > 0) {
        int byte_in = Serial.read();
        if (byte_in >= 0) {
            Actuator_ProcessSerialChar(static_cast<char>(byte_in));
        }
    }
}

void Actuator_ResetSerialBuffer() {
    s_rx_index = 0;
    s_rx_overflow_discard = false;
    std::memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
}
