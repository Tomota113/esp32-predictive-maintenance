# Original User Request

## 2026-08-21T13:37:09Z

Industrial predictive maintenance architecture for a DC motor running on ESP32 (PlatformIO / C++) and Python 3 MLOps, featuring a real-time live demonstration and fault-injection system for academic/jury presentations.

Working directory: /home/ibrahim-tomota/Documents/antigravity/resilient-volta/maintenance_moteur
Integrity mode: development

## Requirements

### R1. Embedded Signal Processing & TinyML Inference (ESP32 / C++)
- Continuous 200 Hz sampling on MPU6050 (Z-axis, 64-sample window) and periodic DHT11 temperature reading.
- Real-time DSP pipeline extracting 4 features: RMS, Peak-to-Peak / Peak, Dominant Frequency (via `arduinoFFT::complexToMagnitude()`), and Temperature.
- Microsecond-scale Random Forest inference embedded in `model.h` with a debounce filter (3 consecutive anomalous frames) before triggering emergency shutdown (L293D H-bridge cut, buzzer, warning LED).

### R2. Live Demonstration & Interactive Serial Fault-Injection Interface
- High-visibility projector-friendly serial telemetry streaming: `[ETAT: ...] | RMS: ... | Peak: ... | Freq: ... | Temp: ... | Inference: ...`.
- Non-blocking serial command listener supporting live commands:
  - `INJECT_VIB`: Injects artificial vibration anomaly to trigger DSP alarm & safety cut.
  - `INJECT_TEMP`: Injects critical thermal drift.
  - `RESET`: Rearms the safety state machine and restarts the DC motor.

### R3. Automated Python MLOps Pipeline (`run_pipeline.py`)
- Guided terminal workflow with auto-detection of active serial port (`/dev/ttyUSB*`, `/dev/ttyACM*`).
- Automatic dataset acquisition: 200 normal samples (`data/normal.csv`) and 200 anomaly samples (`data/anomaly.csv`).
- Training pipeline with `RandomForestClassifier(n_estimators=3, max_depth=4)` and direct C++ header code generation into `include/model.h` (using `micromlgen` or native C++ emission with `#pragma once`).

### R4. Complete Project Structure & Presentation Guide
- Full project structure:
  maintenance_moteur/
  ├── platformio.ini
  ├── include/
  │   ├── config.h             # Brochage complet, paramètres DSP, seuils de sécurité
  │   ├── sensors.h            # Abstraction MPU6050 (I2C) & DHT11
  │   ├── dsp_pipeline.h       # RingBuffer, calcul RMS, Peak, FFT 200 Hz via arduinoFFT
  │   ├── actuator_control.h   # Machine à états (FSM) de sécurité + Alarmes + Réarmement
  │   └── model.h              # Header C++ auto-généré par le script MLOps
  ├── src/
  │   ├── sensors.cpp
  │   ├── dsp_pipeline.cpp
  │   ├── actuator_control.cpp
  │   └── main.cpp             # Double mode : Acquisition / Inférence temps réel + Injection de pannes série
  ├── mlops/
  │   ├── data/
  │   │   ├── normal.csv       # Dataset généré automatiquement
  │   │   └── anomaly.csv
  │   ├── requirements.txt     # pyserial, pandas, scikit-learn, micromlgen
  │   └── run_pipeline.py      # Script 100% automatique : Collecte guidée -> Entraînement -> Export model.h
  └── README.md                # Guide pas à pas spécial "Jour de Présentation"
- Fully configured `platformio.ini` with all required library dependencies (`arduinoFFT`, `Adafruit MPU6050`, `DHT sensor library`, etc.).
- Modular C++ codebase without placeholder code or `// TODO`.
- Comprehensive `README.md` step-by-step presentation script for the live jury demonstration.

## Acceptance Criteria

### Embedded Firmware Build & Logic
- [ ] PlatformIO project compiles cleanly without missing symbols, unresolved includes, or syntax errors.
- [ ] DSP pipeline correctly executes 64-point FFT and extracts all 4 features in under 10 ms.
- [ ] Emergency stop transitions the FSM to ALARM state and cuts L293D GPIOs within 100 ms of 3 consecutive anomalous detections.
- [ ] Serial command parser reliably handles `INJECT_VIB`, `INJECT_TEMP`, and `RESET` without blocking the main control loop.

### MLOps Pipeline & Automation
- [ ] `run_pipeline.py` executes without errors in Python 3 with `requirements.txt`.
- [ ] Auto-generates clean, standalone C++ inference code in `include/model.h`.
- [ ] Model inference accuracy on generated test split exceeds 95% with zero memory allocations at runtime.

### Documentation & Demo Readiness
- [ ] `README.md` includes clear wiring schematics, CLI instructions, and a timeline-based demo script for the jury.
