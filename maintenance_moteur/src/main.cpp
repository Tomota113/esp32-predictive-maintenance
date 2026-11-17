/**
 * @file main.cpp
 * @brief Production Firmware Entry Point: Industrial Predictive Maintenance for DC Motor
 * 
 * Unites:
 *  - 200 Hz MPU6050 Accelerometer Sampling & Calibration (Sensors)
 *  - Non-blocking 0.5 Hz DHT11 Temperature Polling (Sensors)
 *  - Real-Time 64-point FFT & Feature Extraction (DSPPipeline)
 *  - Edge TinyML Random Forest Anomaly Classification (model.h)
 *  - 5-State Safety Finite State Machine with 3-Frame Debounce (ActuatorControl)
 *  - L293D Dual H-Bridge Motor Control & Acoustic/Visual Signaling
 *  - Projector-Friendly Telemetry Stream & Non-Blocking Serial Fault Injection
 * 
 * Target: ESP32 DevKit V1 (Arduino Framework) & Native Host Simulation
 * Zero dynamic heap allocations in steady-state loop.
 */

#ifndef UNIT_TEST

#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "dsp_pipeline.h"
#include "model.h"
#include "actuator_control.h"

// =============================================================================
// Static Global Subsystems (Zero Dynamic Memory Allocation)
// =============================================================================
static DSPPipeline s_dsp_pipeline;
static unsigned long s_last_sample_micros = 0;
static unsigned long s_last_dht_millis = 0;
static float s_current_temperature = 25.0f;

// =============================================================================
// System Setup & Hardware Initialization
// =============================================================================
void setup() {
    // 1. Initialize Serial Communication at 115200 baud
    Serial.begin(SERIAL_BAUD_RATE);
    
    // Allow USB CDC / UART to stabilize
    unsigned long serial_wait_start = millis();
    while (!Serial && (millis() - serial_wait_start < 2000)) {
        // Wait for serial on native USB boards if attached
    }

    Serial.println();
    Serial.println("================================================================");
    Serial.println("   INDUSTRIAL PREDICTIVE MAINTENANCE SYSTEM - DC MOTOR (ESP32)  ");
    Serial.println("   Edge TinyML (Random Forest) + 64-pt FFT DSP + Safety FSM     ");
    Serial.println("================================================================");
    Serial.println("[INIT] Bootstrapping hardware peripherals and control loops...");

    // 2. Initialize Actuators (Motor PWM, Direction GPIOs, Buzzer, Status LEDs)
    Actuator_Init();
    Serial.println("[INIT] Actuator subsystem initialized (L293D, Buzzer, Status LEDs).");

    // 3. Initialize Sensors (I2C MPU6050 Accelerometer & 1-Wire DHT11)
    Sensors_Init();
    if (Sensors_IsMPUHealthy()) {
        Serial.println("[INIT] Sensor subsystem initialized (MPU6050 detected @ 200 Hz, DHT11).");
    } else {
        Serial.println("[INIT] [NOTE] MPU6050 hardware non détecté sur le bus I2C (SDA:21, SCL:22).");
        Serial.println("[INIT] -> Mode simulation de secours actif (télémétrie nominale & commandes de démo actives).");
    }

    // 4. Standstill Baseline Calibration (100 samples = ~500 ms)
    Serial.println("[INIT] Calibrating MPU6050 standstill baseline gravity offset...");
    Sensors_Calibrate(100);
    Serial.print("[INIT] Baseline gravity offset: ");
    Serial.print(Sensors_GetCalibrationOffset(), 3);
    Serial.println(" m/s^2");

    // 5. Arm Safety State Machine to STATE_NORMAL and Soft-Start Motor
    Actuator_Rearm();

    // 6. Display Diagnostics and Interactive Command Menu
    Actuator_PrintStatus();
    Actuator_PrintHelp();

    Serial.println("[INIT] System operational. Telemetry stream started.");
    Serial.println("----------------------------------------------------------------");

    // 7. Initialize High-Resolution Timestamps
    s_last_sample_micros = micros();
    s_last_dht_millis = millis();
}

// =============================================================================
// Main Non-Blocking Real-Time Execution Loop
// =============================================================================
void loop() {
    // -------------------------------------------------------------------------
    // 1. Non-Blocking Serial Command Listener (INJECT_VIB, INJECT_TEMP, RESET)
    // -------------------------------------------------------------------------
    Actuator_ProcessSerial();

    // -------------------------------------------------------------------------
    // 2. Non-Blocking Periodic DHT11 Temperature Polling (2000 ms interval)
    // -------------------------------------------------------------------------
    unsigned long now_ms = millis();
    if (now_ms - s_last_dht_millis >= DHT_INTERVAL_MS) {
        s_last_dht_millis = now_ms;
        Sensors_SampleDHT(s_current_temperature);
    }

    // -------------------------------------------------------------------------
    // 3. High-Rate MPU6050 Accelerometer Sampling (200 Hz -> 5000 µs interval)
    // -------------------------------------------------------------------------
    unsigned long now_us = micros();
    if (now_us - s_last_sample_micros >= SAMPLE_INTERVAL_US) {
        s_last_sample_micros = now_us;

        float z_sample = 0.0f;
        if (Sensors_SampleMPU(z_sample)) {
            s_dsp_pipeline.pushSample(z_sample);
        }

        // ---------------------------------------------------------------------
        // 4. Feature Extraction & TinyML Inference upon 64-Sample Window
        // ---------------------------------------------------------------------
        if (s_dsp_pipeline.isWindowReady()) {
            // Compute 4 DSP Features: RMS, Peak Absolute, Dominant Frequency, Temp
            DSPFeatures features = s_dsp_pipeline.computeFeatures(s_current_temperature);

            // Serialize into 4-element float array [RMS, Peak, Freq, Temp]
            float feature_vector[4];
            features.toFeatureArray(feature_vector);

            // Edge TinyML Random Forest Classification (0 = NORMAL, 1 = ANOMALY)
            int inference_result = Eloquent::ML::Port::RandomForest::predict(feature_vector);

            // Update Safety FSM with 3-frame debounce and sub-100ms emergency cutoff
            Actuator_UpdateFSM(inference_result);

            // Emit live high-visibility telemetry stream over UART
            Actuator_PrintTelemetry(features, inference_result);

            // Reset DSP pipeline buffer for the next 64-sample window
            s_dsp_pipeline.reset();
        }
    }
}

// =============================================================================
// Standalone Native Entry Point (for native host execution outside unit tests)
// =============================================================================
#if !defined(ARDUINO)
int main(int argc, char** argv) {
    (void)argc;
    (void)argv;
    setup();
    while (true) {
        loop();
    }
    return 0;
}
#endif

#endif // UNIT_TEST
