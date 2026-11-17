# E2E Test Infra: Industrial Predictive Maintenance for DC Motor

## Test Philosophy
- Opaque-box, requirement-driven, dual-target verification.
- 100% native host testability without requiring physical ESP32 or sensors in CI.
- Zero heap allocation and microsecond execution guarantees.
- Methodology: Category-Partition + Boundary Value Analysis (BVA) + Pairwise Combinatorial Testing + Real-World Workload Testing + Adversarial Hardening.

## Feature Inventory
| # | Feature | Source (requirement) | Tier 1 | Tier 2 | Tier 3 |
|---|---------|---------------------|:------:|:------:|:------:|
| 1 | RingBuffer (64-sample FIFO, circular window) | ORIGINAL_REQUEST §R1 | 5 | 1 | ✓ |
| 2 | RMS Calculation (detrended signal energy) | ORIGINAL_REQUEST §R1 | 5 | 1 | ✓ |
| 3 | Peak / Peak-to-Peak Amplitude | ORIGINAL_REQUEST §R1 | 5 | 1 | ✓ |
| 4 | 64-point FFT & Dominant Frequency Extraction | ORIGINAL_REQUEST §R1 | 5 | 1 | ✓ |
| 5 | DHT11 Temperature Acquisition & NaN Protection | ORIGINAL_REQUEST §R1 | 5 | 1 | ✓ |
| 6 | Safety FSM Normal Operation & Init | ORIGINAL_REQUEST §R1 | 5 | 1 | ✓ |
| 7 | Safety FSM 3-Frame Debounce & Emergency Cut | ORIGINAL_REQUEST §R1 | 5 | 1 | ✓ |
| 8 | Safety FSM Rearm / Reset Command | ORIGINAL_REQUEST §R2 | 5 | 1 | ✓ |
| 9 | TinyML Random Forest Model Inference (model.h)| ORIGINAL_REQUEST §R1 | 5 | 1 | ✓ |
| 10 | Serial Telemetry Stream Formatting | ORIGINAL_REQUEST §R2 | 5 | 1 | ✓ |
| 11 | Serial Fault Injection Parser (VIB/TEMP/RESET) | ORIGINAL_REQUEST §R2 | 5 | 1 | ✓ |

## Test Architecture
- **Host Native Harness**: Standalone `Makefile` compiling C++17 native tests with GCC/Clang (`g++ -std=c++17 -Wall -Wextra -O2 -DNATIVE_BUILD -DUNIT_TEST -Iinclude -Itest/mocks -Itest`).
- **Mock Layer (`test/mocks/`)**: Virtual clock (`micros()`, `millis()`), Mock GPIO (`digitalWrite`, `pinMode`, `ledcWrite`), Mock Serial (`MockSerial` RX injection / TX capture), Mock MPU6050 (`Adafruit_MPU6050`), Mock DHT (`DHT`), and Native Radix-2 FFT engine (`arduinoFFT.h`).
- **Python Verification Suite (`mlops/tests/`)**: Automated pytest suite verifying dataset schemas, distributions, RF training parameters, C++ header code emission, and Python vs C++ mathematical prediction equivalence across 400+ vectors.

## Real-World Application Scenarios (Tier 4)
| # | Scenario | Features Exercised | Complexity |
|---|----------|--------------------|------------|
| 1 | 1000-Cycle Steady State Run | F1, F2, F3, F4, F5, F6, F9, F10 | High |
| 2 | Complete 7-Phase Industrial Lifecycle (Boot -> Normal -> Transient Spike -> Degradation -> 3-Anomaly Trip -> Emergency Cut < 100ms -> Manual RESET -> Recovery) | F1-F11 | High |
| 3 | Sudden Mechanical Rotor Jam / Bearing Seizure | F1, F2, F3, F4, F7, F9 | Medium |
| 4 | Thermal Runaway under Continuous Heavy Load | F5, F7, F9, F10 | Medium |
| 5 | Intermittent Sensor Communication Flake / Recovery | F2, F5, F6, F10 | Medium |

## Coverage Thresholds
- **Tier 1 (Feature Coverage)**: 55 unit tests (≥5 per feature across 11 features)
- **Tier 2 (Boundary & Corner Cases)**: ≥8 tests covering Nyquist limits, buffer wrap-arounds, zero vibration, and extreme floats
- **Tier 3 (Cross-Feature Combinations)**: ≥5 tests covering simultaneous vibration + thermal faults, race conditions, and debounce interleaving
- **Tier 4 (Real-World Application Scenarios)**: ≥5 realistic lifecycle and continuous workload tests
- **Tier 5 (Adversarial Coverage Hardening)**: Protocol fuzzing, sensor clipping, stack depth audits, brownout recovery
- **Total Minimum Test Cases**: ≥75 automated C++ and Python test cases
