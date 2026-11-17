#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include "config.h"

/**
 * @brief Unified Sensor Snapshot Data Structure
 */
struct SensorData {
    float raw_accel_z;       // Raw Z-axis acceleration in m/s^2
    float filtered_accel_z;  // Baseline detrended zero-mean acceleration in m/s^2
    float temperature;       // Temperature in degrees Celsius (°C)
    bool mpu_healthy;        // MPU6050 I2C bus & health status
    bool dht_healthy;        // DHT11 1-Wire sensor health status
};

/**
 * @brief Initialize I2C bus, MPU6050 accelerometer, and DHT11 temperature sensor.
 */
void Sensors_Init();

/**
 * @brief Perform baseline standstill calibration to compute Earth gravity / sensor DC offset.
 * @param sample_count Number of baseline samples to average (default: 100).
 */
void Sensors_Calibrate(uint16_t sample_count = 100);

/**
 * @brief Sample MPU6050 Z-axis acceleration, apply detrending and fault injection if active.
 * @param[out] z_sample Extracted Z-axis sample (m/s^2).
 * @return True if sample acquired successfully, false if I2C/sensor error.
 */
bool Sensors_SampleMPU(float &z_sample);

/**
 * @brief Asynchronously poll DHT11 temperature sensor with non-blocking rate limit.
 * @param[out] temperature Current or cached valid temperature (°C).
 * @return True if sensor read was valid, false on NaN or read failure (cached value returned).
 */
bool Sensors_SampleDHT(float &temperature);

/**
 * @brief Enable or disable simulated vibration fault injection (INJECT_VIB).
 * @param active True to inject 65 Hz / 3.5 m/s^2 anomaly, false for normal sensor path.
 */
void Sensors_SetVibrationInjection(bool active);

/**
 * @brief Enable or disable simulated temperature drift fault injection (INJECT_TEMP).
 * @param active True to inject critical 80.0°C thermal drift, false for normal sensor path.
 */
void Sensors_SetTemperatureInjection(bool active);

/**
 * @brief Check if vibration fault injection is currently active.
 */
bool Sensors_IsVibrationInjected();

/**
 * @brief Check if temperature fault injection is currently active.
 */
bool Sensors_IsTemperatureInjected();

/**
 * @brief Get the baseline gravitational DC offset (m/s^2) computed during calibration.
 */
float Sensors_GetCalibrationOffset();

/**
 * @brief Manually set the baseline gravitational DC offset (m/s^2) for testing/tuning.
 */
void Sensors_SetCalibrationOffset(float offset);

/**
 * @brief Get the current health status of the MPU6050 accelerometer.
 */
bool Sensors_IsMPUHealthy();

/**
 * @brief Get the current health status of the DHT11 temperature sensor.
 */
bool Sensors_IsDHTHealthy();
