# Project: Industrial Predictive Maintenance for DC Motor (ESP32 / PlatformIO C++ & Python 3 MLOps)

## Architecture
The system is an industrial edge predictive maintenance platform combining embedded DSP and TinyML inference on ESP32 with an automated Python 3 MLOps pipeline and interactive serial fault injection for live demonstrations.

```
+---------------------------------------------------------------------------------------------------+
|                                      SYSTEM ARCHITECTURE                                          |
|                                                                                                   |
|  [ SENSORS ]                                                                                      |
|  - MPU6050 (I2C Z-axis @ 200 Hz) ---> [ RingBuffer<float, 64> ]                                   |
|  - DHT11 (1-Wire @ 0.5 Hz)       ---> [ Cached Temp ]                                             |
|                                            |                                                      |
|  [ DSP PIPELINE ]                          v                                                      |
|  - DC Offset Detrending                                                                           |
|  - Feature 1: RMS                                                                                 |
|  - Feature 2: Peak / Peak-to-Peak          ===> Feature Vector [RMS, Peak, Freq, Temp]            |
|  - Feature 3: 64-pt FFT Dominant Freq                   |                                         |
|  - Feature 4: Temperature                               v                                         |
|                                              [ TinyML INFERENCE (model.h) ]                       |
|                                              - 3-Tree Random Forest (max depth 4)                 |
|                                              - 0 dynamic memory allocations                       |
|                                              - Output: 0 (Normal) / 1 (Anomaly)                   |
|                                                         |                                         |
|  [ SAFETY FSM & ACTUATION ]                             v                                         |
|  - States: INIT, NORMAL, WARNING, ALARM, MAINT <--- [ 3-Frame Debounce Filter ]                  |
|  - L293D DC Motor PWM (5 kHz 8-bit)                                                               |
|  - Warning LED & Buzzer Acoustic Alarm                                                            |
|  - Emergency Cutoff (< 100 ms) upon 3 consecutive anomalies                                       |
|                                                                                                   |
|  [ SERIAL TELEMETRY & INJECTION ]                                                                 |
|  - Telemetry Stream: [ETAT: ...] | RMS: ... | Peak: ... | Freq: ... | Temp: ... | Inference: ...   |
|  - Non-blocking parser for commands: INJECT_VIB, INJECT_TEMP, RESET                                 |
|                                                                                                   |
|  [ PYTHON 3 MLOPS PIPELINE (run_pipeline.py) ]                                                    |
|  - Serial auto-discovery & guided acquisition / synthetic physics generation                      |
|  - Datasets: data/normal.csv (200) & data/anomaly.csv (200)                                       |
|  - RandomForestClassifier training (accuracy > 95%) & direct AST C++ emission to include/model.h  |
+---------------------------------------------------------------------------------------------------+
```

## Feature Inventory
| # | Feature | Description | Milestone | Source |
|---|---------|-------------|-----------|--------|
| 1 | RingBuffer Storage | 64-sample static circular buffer with linear snapshot extraction | M1 | Survey |
| 2 | MPU6050 & DHT11 Drivers | I2C 200 Hz continuous sampling, 1-wire temp polling, baseline calibration, fault injection hooks | M1 | Survey |
| 3 | Real-Time DSP Pipeline | Feature extraction: RMS, Peak/Peak-to-Peak, 64-point FFT dominant frequency via arduinoFFT, Temp | M1 | Survey |
| 4 | L293D Motor & Actuators | 5 kHz 8-bit PWM speed control, directional GPIOs, Buzzer & LED alarm signaling | M2 | Survey |
| 5 | Safety FSM & Debounce | 5-state safety FSM, 3-frame debounce anomaly filter, sub-100ms emergency cut | M2 | Survey |
| 6 | Serial Telemetry & Parser | Projector-friendly serial telemetry streaming & non-blocking parser (INJECT_VIB, INJECT_TEMP, RESET) | M2 | Survey |
| 7 | MLOps Data Acquisition & Synth | Guided serial acquisition & physics-based Gaussian synthetic generator (normal.csv, anomaly.csv) | M3 | Survey |
| 8 | MLOps Training Pipeline | RandomForestClassifier(n_estimators=3, max_depth=4), stratified 80/20 split, evaluation metrics (>95%) | M3 | Survey |
| 9 | MLOps C++ AST Code Emitter | Standalone `#pragma once` header code generator for `include/model.h` (0 bytes heap) | M3 | Survey |
| 10 | MLOps Unified CLI Runner | `mlops/run_pipeline.py` CLI supporting `--mock`, `--port`, `--samples`, `--eval-only` | M3 | Survey |
| 11 | Production Firmware Integration | Complete `platformio.ini` and `src/main.cpp` dual-mode runtime (live inference + injection) | M4 | Survey |
| 12 | Presentation Documentation | Comprehensive step-by-step presentation script & wiring guide in `README.md` | M4 | Survey |
| 13 | Dual-Target Mock Layer | Native host mocks for Arduino, Wire, Adafruit_MPU6050, DHT, arduinoFFT, and Virtual Clock | T1 | Survey |
| 14 | 5-Tier E2E Test Suite | 55+ Tier 1 unit tests, Tier 2 boundaries, Tier 3 combos, Tier 4 workloads, Python test suite | T2 | Survey |
| 15 | Final Verification & Hardening | 100% Tier 1-4 E2E test pass, Tier 5 Adversarial Coverage Hardening, zero-leak verification | M5 | Survey |

## Milestones

### Implementation Track
| # | Name | Scope | Dependencies | Status |
|---|------|-------|-------------|--------|
| M1 | Embedded DSP & Sensors | `include/config.h`, `include/sensors.h`, `include/dsp_pipeline.h`, `src/sensors.cpp`, `src/dsp_pipeline.cpp` | none | DONE |
| M2 | Actuator FSM & Serial Parser | `include/actuator_control.h`, `src/actuator_control.cpp` | M1 | DONE |
| M3 | Python MLOps & Model Emitter | `mlops/requirements.txt`, `mlops/run_pipeline.py`, `mlops/data/normal.csv`, `mlops/data/anomaly.csv`, `include/model.h` | none | DONE |
| M4 | Firmware Integration & README | `platformio.ini`, `src/main.cpp`, `README.md` | M1, M2, M3 | DONE |
| M5 | Final E2E Pass & Hardening | 100% E2E test suite pass (Tiers 1-4) & Tier 5 Adversarial Coverage Hardening | M4, T2 | DONE |

### E2E Testing Track (Parallel)
| # | Name | Scope | Dependencies | Status |
|---|------|-------|-------------|--------|
| T1 | Native Host Mock Layer | `test/mocks/` (Arduino, Wire, Adafruit_MPU6050, DHT, arduinoFFT), `test/test_framework.h`, `Makefile` | none | DONE |
| T2 | 5-Tier E2E Test Suite | `test/test_tier1_features.cpp`, `test/test_tier2_boundaries.cpp`, `test/test_tier3_combos.cpp`, `test/test_tier4_workloads.cpp`, `test/test_main.cpp`, `mlops/tests/`, `scripts/run_all_tests.sh`, `TEST_READY.md` | T1 | DONE |

## Interface Contracts

### Sensors ↔ DSP Pipeline
```cpp
struct SensorData {
    float raw_accel_z;
    float filtered_accel_z;
    float temperature;
    bool mpu_healthy;
    bool dht_healthy;
};
void Sensors_Init();
void Sensors_Calibrate(uint16_t sample_count = 100);
bool Sensors_SampleMPU(float &z_sample);
bool Sensors_SampleDHT(float &temperature);
void Sensors_SetVibrationInjection(bool active);
void Sensors_SetTemperatureInjection(bool active);
bool Sensors_IsVibrationInjected();
bool Sensors_IsTemperatureInjected();
```

### DSP Pipeline ↔ TinyML Model
```cpp
struct DSPFeatures {
    float rms;
    float peak_to_peak;
    float peak_absolute;
    float dominant_frequency;
    float temperature;
    void toFeatureArray(float* out_features) const;
};
class DSPPipeline {
public:
    DSPPipeline();
    void pushSample(float z_sample);
    bool isWindowReady() const;
    DSPFeatures computeFeatures(float current_temp);
};
```

### TinyML Model ↔ Safety FSM
```cpp
namespace Eloquent {
namespace ML {
namespace Port {
class RandomForest {
public:
    static int predict(const float* x); // returns 0 (NORMAL) or 1 (ANOMALY)
    static int predict(float rms, float peak, float freq, float temp);
    static float predict_proba(const float* x);
};
}
}
}
int model_predict(const float* features);
```

### Safety FSM ↔ Actuator / Telemetry
```cpp
enum SystemState {
    STATE_INIT = 0,
    STATE_NORMAL = 1,
    STATE_WARNING = 2,
    STATE_ALARM = 3,
    STATE_MAINTENANCE = 4
};
const char* StateToString(SystemState state);
void Actuator_Init();
void Actuator_UpdateFSM(int inference_result);
void Actuator_EmergencyStop();
void Actuator_Rearm();
void Actuator_SetMotorPWM(uint8_t duty);
SystemState Actuator_GetState();
uint8_t Actuator_GetDebounceCount();
```

## Code Layout
```
maintenance_moteur/
├── platformio.ini
├── Makefile
├── include/
│   ├── config.h
│   ├── sensors.h
│   ├── dsp_pipeline.h
│   ├── actuator_control.h
│   └── model.h
├── src/
│   ├── sensors.cpp
│   ├── dsp_pipeline.cpp
│   ├── actuator_control.cpp
│   └── main.cpp
├── mlops/
│   ├── data/
│   │   ├── normal.csv
│   │   └── anomaly.csv
│   ├── requirements.txt
│   ├── run_pipeline.py
│   └── tests/
│       ├── test_dataset.py
│       ├── test_training.py
│       ├── test_model_export.py
│       └── test_equivalence.py
├── test/
│   ├── mocks/
│   │   ├── Arduino.h
│   │   ├── Wire.h
│   │   ├── Adafruit_MPU6050.h
│   │   ├── Adafruit_Sensor.h
│   │   ├── DHT.h
│   │   └── arduinoFFT.h
│   ├── test_framework.h
│   ├── test_tier1_features.cpp
│   ├── test_tier2_boundaries.cpp
│   ├── test_tier3_combos.cpp
│   ├── test_tier4_workloads.cpp
│   ├── test_tier5_hardening.cpp
│   └── test_main.cpp
├── scripts/
│   └── run_all_tests.sh
└── README.md
```
