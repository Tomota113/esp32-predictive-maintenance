#pragma once
#include <Arduino.h>

// ==========================================
// 1. PINOUT CONFIGURATION (ESP32 DevKit V1)
// ==========================================
// I2C MPU6050 Accelerometer
#define PIN_I2C_SDA          21
#define PIN_I2C_SCL          22

// DHT11 1-Wire Temperature Sensor
#define PIN_DHT_DATA         4
#ifndef DHT_TYPE
#define DHT_TYPE             DHT11
#endif

// L293D Dual H-Bridge Motor Driver
#define PIN_MOTOR_ENA        18  // PWM speed control
#define PIN_MOTOR_IN1        19  // Direction Forward
#define PIN_MOTOR_IN2        23  // Direction Reverse

// Safety Signaling & Actuators
#define PIN_BUZZER           27  // Acoustic alarm buzzer
#define PIN_LED_NORMAL       2   // Green LED (Normal operation)
#define PIN_LED_WARN_ALARM   13  // Red / Amber LED (Warning / Alarm)

// ==========================================
// 2. DSP & SAMPLING PARAMETERS
// ==========================================
#define SAMPLING_FREQ_HZ     200.0f
#define SAMPLE_INTERVAL_US   5000     // 1 / 200 Hz = 5000 microseconds (5.0 ms)
#define WINDOW_SIZE          64       // 64 samples @ 200 Hz = 320 ms window
#define DHT_INTERVAL_MS      2000     // Non-blocking DHT11 polling (2.0s interval)

// ==========================================
// 3. PWM MOTOR PARAMETERS
// ==========================================
#define PWM_CHANNEL_MOTOR    0
#define PWM_FREQ_MOTOR       1000     // 1 kHz PWM frequency (optimal for L293D bipolar transistors)
#define PWM_RESOLUTION_BITS  8        // 8-bit resolution (0 - 255)
#define MOTOR_NOMINAL_PWM    255      // 100% duty cycle to compensate for L293D internal voltage drop (~2.5V)

// ==========================================
// 4. SAFETY FSM PARAMETERS
// ==========================================
#define DEBOUNCE_THRESHOLD   3        // Exactly 3 consecutive anomaly frames before ALARM
#define SERIAL_BAUD_RATE     115200   // Projector telemetry & serial UART baud rate
