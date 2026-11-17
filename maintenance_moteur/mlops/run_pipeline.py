#!/usr/bin/env python3
"""
===============================================================================
Industrial Predictive Maintenance for DC Motor — Python 3 MLOps Pipeline
===============================================================================
Component: mlops/run_pipeline.py
Target: ESP32 Edge Predictive Maintenance (TinyML 3-Tree Random Forest)

This pipeline performs:
1. Serial data acquisition from ESP32 or synthetic physics-based generation.
2. Dataset storage in mlops/data/normal.csv and mlops/data/anomaly.csv.
3. Training of a deterministic RandomForestClassifier(n_estimators=3, max_depth=4).
4. Full evaluation (accuracy, precision, recall, f1, confusion matrix, feature importances).
5. Direct AST C++ code generation producing a zero-heap standalone header (include/model.h).
6. Automatic self-verification and equivalence checking against g++.
===============================================================================
"""

import argparse
import os
import re
import sys
import time
import subprocess
import tempfile
from pathlib import Path
from typing import List, Optional, Tuple, Dict, Any

import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import (
    accuracy_score,
    precision_score,
    recall_score,
    f1_score,
    confusion_matrix,
    classification_report,
)
from sklearn.model_selection import train_test_split

try:
    import serial
    import serial.tools.list_ports
    HAS_PYSERIAL = True
except ImportError:
    HAS_PYSERIAL = False


# =============================================================================
# 1. Physics Distribution Parameters for Synthetic Generation
# =============================================================================

NORMAL_DIST = {
    'rms':           {'mean': 0.35, 'std': 0.08, 'min': 0.15, 'max': 0.65},  # g
    'peak':          {'mean': 0.85, 'std': 0.15, 'min': 0.40, 'max': 1.40},  # g
    'dominant_freq': {'mean': 33.0, 'std': 3.0,  'min': 25.0, 'max': 42.0},  # Hz (nominal 2000 RPM)
    'temperature':   {'mean': 27.5, 'std': 2.5,  'min': 20.0, 'max': 35.0},  # °C (ambient)
}

VIB_FAULT_DIST = {
    'rms':           {'mean': 2.80, 'std': 0.60, 'min': 1.50, 'max': 5.50},  # g (mechanical imbalance)
    'peak':          {'mean': 5.50, 'std': 1.20, 'min': 3.00, 'max': 10.00}, # g
    'dominant_freq': {'mean': 75.0, 'std': 8.0,  'min': 55.0, 'max': 95.0},  # Hz (bearing harmonics)
    'temperature':   {'mean': 32.0, 'std': 4.0,  'min': 24.0, 'max': 42.0},  # °C
}

THERMAL_FAULT_DIST = {
    'rms':           {'mean': 1.20, 'std': 0.35, 'min': 0.60, 'max': 2.20},  # g (friction vibration)
    'peak':          {'mean': 2.40, 'std': 0.70, 'min': 1.20, 'max': 4.00},  # g
    'dominant_freq': {'mean': 35.0, 'std': 5.0,  'min': 25.0, 'max': 50.0},  # Hz
    'temperature':   {'mean': 65.0, 'std': 6.0,  'min': 50.0, 'max': 85.0},  # °C (overheating / stall)
}

FEATURE_COLUMNS = ['rms', 'peak', 'dominant_freq', 'temperature']


# =============================================================================
# 2. Synthetic Dataset Generation
# =============================================================================

def generate_synthetic_normal(n_samples: int = 200, seed: int = 42) -> pd.DataFrame:
    """Generate realistic physics-modeled normal DC motor baseline data."""
    rng = np.random.RandomState(seed)
    data = {}
    for feat, params in NORMAL_DIST.items():
        raw = rng.normal(params['mean'], params['std'], size=n_samples)
        clipped = np.clip(raw, params['min'], params['max'])
        data[feat] = np.round(clipped, 4)
    return pd.DataFrame(data)[FEATURE_COLUMNS]


def generate_synthetic_anomaly(n_samples: int = 200, seed: int = 42) -> pd.DataFrame:
    """
    Generate realistic physics-modeled anomaly DC motor data.
    Equally split between mechanical vibration faults and thermal overheat faults.
    """
    rng = np.random.RandomState(seed + 100)
    n_vib = n_samples // 2
    n_thermal = n_samples - n_vib

    # Vibration fault subset
    vib_data = {}
    for feat, params in VIB_FAULT_DIST.items():
        raw = rng.normal(params['mean'], params['std'], size=n_vib)
        clipped = np.clip(raw, params['min'], params['max'])
        vib_data[feat] = np.round(clipped, 4)
    df_vib = pd.DataFrame(vib_data)

    # Thermal fault subset
    thermal_data = {}
    for feat, params in THERMAL_FAULT_DIST.items():
        raw = rng.normal(params['mean'], params['std'], size=n_thermal)
        clipped = np.clip(raw, params['min'], params['max'])
        thermal_data[feat] = np.round(clipped, 4)
    df_thermal = pd.DataFrame(thermal_data)

    df_anomaly = pd.concat([df_vib, df_thermal], ignore_index=True)
    # Shuffle anomaly samples for uniform distribution
    df_anomaly = df_anomaly.sample(frac=1.0, random_state=seed).reset_index(drop=True)
    return df_anomaly[FEATURE_COLUMNS]


def generate_synthetic_datasets(
    data_dir: Path, n_samples: int = 200, seed: int = 42
) -> Tuple[pd.DataFrame, pd.DataFrame]:
    """Generate both normal and anomaly synthetic datasets and save to CSV."""
    data_dir.mkdir(parents=True, exist_ok=True)
    df_normal = generate_synthetic_normal(n_samples=n_samples, seed=seed)
    df_anomaly = generate_synthetic_anomaly(n_samples=n_samples, seed=seed)

    normal_path = data_dir / "normal.csv"
    anomaly_path = data_dir / "anomaly.csv"

    df_normal.to_csv(normal_path, index=False)
    df_anomaly.to_csv(anomaly_path, index=False)

    return df_normal, df_anomaly


# =============================================================================
# 3. Serial Port Detection & Telemetry Parsing
# =============================================================================

def list_serial_ports() -> List[Any]:
    """List available serial communication ports."""
    if not HAS_PYSERIAL:
        return []
    return list(serial.tools.list_ports.comports())


def find_esp32_port() -> Optional[str]:
    """Auto-detect connected ESP32 USB-UART device port."""
    if not HAS_PYSERIAL:
        return None
    ports = list_serial_ports()
    if not ports:
        return None

    # Known ESP32 USB-UART VID:PID or descriptions
    known_signatures = [
        "CP210", "CH340", "CH341", "FTDI", "UART", "USB",
        "10c4:ea60", "1a86:7523", "0403:6001", "303a:"
    ]
    for p in ports:
        desc_full = f"{p.device} {p.description} {p.hwid}".lower()
        for sig in known_signatures:
            if sig.lower() in desc_full:
                return p.device

    # Fallback to first available port
    return ports[0].device


def parse_telemetry_line(line: str) -> Optional[List[float]]:
    """
    Robust telemetry parser supporting:
    1. Projector Display: [ETAT: NORMAL] | RMS: 0.342 | Peak: 0.812 | Freq: 33.2 | Temp: 26.5 | Inference: 0
    2. Compact CSV: DATA,0.342,0.812,33.2,26.5 or 0.342,0.812,33.2,26.5
    """
    if not line:
        return None
    clean = line.strip()
    if not clean:
        return None

    # Format 1: Structured display format with named fields
    m1 = re.search(
        r"RMS:\s*([-+]?[0-9]*\.?[0-9]+)\s*\|\s*Peak:\s*([-+]?[0-9]*\.?[0-9]+)\s*\|\s*Freq:\s*([-+]?[0-9]*\.?[0-9]+)\s*\|\s*Temp:\s*([-+]?[0-9]*\.?[0-9]+)",
        clean,
        re.IGNORECASE,
    )
    if m1:
        try:
            return [
                float(m1.group(1)),
                float(m1.group(2)),
                float(m1.group(3)),
                float(m1.group(4)),
            ]
        except ValueError:
            pass

    # Format 2: Tagged CSV format 'DATA,val1,val2,val3,val4'
    if clean.startswith("DATA,") or clean.startswith("DATA:"):
        parts = clean[5:].split(",")
        if len(parts) == 4:
            try:
                return [float(p.strip()) for p in parts]
            except ValueError:
                pass

    # Format 3: Raw 4-token CSV line
    parts = clean.split(",")
    if len(parts) == 4:
        try:
            return [float(p.strip()) for p in parts]
        except ValueError:
            pass

    return None


def acquire_serial_dataset(
    port: str,
    baud: int,
    n_samples: int,
    phase_title: str,
    instructions: List[str],
    output_path: Path,
    command_to_send: Optional[str] = None,
    non_interactive: bool = False,
) -> pd.DataFrame:
    """Acquire live feature frames from ESP32 serial stream."""
    if not HAS_PYSERIAL:
        raise RuntimeError("pyserial is not installed; cannot acquire from live hardware.")

    print("\n" + "=" * 64)
    print(f" {phase_title.upper()} ({n_samples} samples)")
    print("=" * 64)
    for instr in instructions:
        print(f" - {instr}")
    print("-" * 64)

    if not non_interactive and sys.stdin.isatty():
        input("Press [ENTER] when ready to start acquisition...")

    print(f"Connecting to {port} @ {baud} baud...")
    ser = serial.Serial(port, baudrate=baud, timeout=2.0)
    time.sleep(1.0)
    ser.reset_input_buffer()

    if command_to_send:
        print(f"Sending serial trigger command: '{command_to_send}'")
        ser.write(f"{command_to_send}\n".encode("utf-8"))
        ser.flush()
        time.sleep(0.5)

    samples: List[List[float]] = []
    print(f"Streaming telemetry data from {port}...")

    start_time = time.time()
    last_ui_update = time.time()

    while len(samples) < n_samples:
        raw_line = ser.readline().decode("utf-8", errors="ignore")
        features = parse_telemetry_line(raw_line)
        if features is not None:
            samples.append(features)
            now = time.time()
            if now - last_ui_update > 0.05 or len(samples) == n_samples:
                last_ui_update = now
                pct = int((len(samples) / n_samples) * 100)
                bar_len = 30
                filled = int(bar_len * len(samples) / n_samples)
                bar = "=" * filled + "-" * (bar_len - filled)
                print(f"\rProgress: [{bar}] {len(samples)}/{n_samples} samples ({pct}%)", end="", flush=True)

        if time.time() - start_time > 60.0 and len(samples) == 0:
            ser.close()
            raise TimeoutError(f"No valid telemetry received from {port} within 60 seconds.")

    print("\nAcquisition complete!")
    ser.close()

    df = pd.DataFrame(samples, columns=FEATURE_COLUMNS)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(output_path, index=False)
    print(f"Saved dataset to {output_path}")
    return df


# =============================================================================
# 4. Model Training & Evaluation
# =============================================================================

def train_model(
    df_normal: pd.DataFrame,
    df_anomaly: pd.DataFrame,
    seed: int = 42,
    test_size: float = 0.20,
) -> Tuple[RandomForestClassifier, Dict[str, Any], np.ndarray, np.ndarray]:
    """
    Train a 3-tree, max-depth-4 Random Forest Classifier with deterministic seed.
    Evaluates on a stratified train/test split.
    """
    # Ensure expected columns are present
    X_normal = df_normal[FEATURE_COLUMNS].values
    X_anomaly = df_anomaly[FEATURE_COLUMNS].values

    y_normal = np.zeros(len(X_normal), dtype=int)
    y_anomaly = np.ones(len(X_anomaly), dtype=int)

    X = np.vstack([X_normal, X_anomaly])
    y = np.concatenate([y_normal, y_anomaly])

    # Stratified 80/20 train/test split
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=test_size, random_state=seed, stratify=y
    )

    # 3-tree Random Forest, max depth 4
    clf = RandomForestClassifier(
        n_estimators=3,
        max_depth=4,
        criterion='gini',
        random_state=seed,
        n_jobs=1,
    )
    clf.fit(X_train, y_train)

    y_pred = clf.predict(X_test)
    y_proba = clf.predict_proba(X_test)[:, 1]

    acc = accuracy_score(y_test, y_pred)
    prec = precision_score(y_test, y_pred, zero_division=0)
    rec = recall_score(y_test, y_pred, zero_division=0)
    f1 = f1_score(y_test, y_pred, zero_division=0)
    cm = confusion_matrix(y_test, y_pred)

    metrics = {
        'accuracy': float(acc),
        'precision': float(prec),
        'recall': float(rec),
        'f1': float(f1),
        'confusion_matrix': cm,
        'train_samples': len(X_train),
        'test_samples': len(X_test),
        'feature_importances': dict(zip(FEATURE_COLUMNS, clf.feature_importances_.tolist())),
        'classification_report': classification_report(y_test, y_pred, target_names=['Normal (0)', 'Anomaly (1)']),
    }

    return clf, metrics, X_test, y_test


def display_training_report(metrics: Dict[str, Any]) -> None:
    """Print formatted evaluation summary to terminal."""
    print("\n" + "=" * 64)
    print(" MODEL TRAINING & VALIDATION RESULTS")
    print("=" * 64)
    print(f" Train Samples: {metrics['train_samples']} | Test Samples: {metrics['test_samples']}")
    print(f" Test Accuracy: {metrics['accuracy'] * 100.0:.2f}%")
    print(f" Precision:     {metrics['precision'] * 100.0:.2f}%")
    print(f" Recall:        {metrics['recall'] * 100.0:.2f}%")
    print(f" F1-Score:      {metrics['f1'] * 100.0:.2f}%")
    print("-" * 64)
    print(" Confusion Matrix (Test Split):")
    cm = metrics['confusion_matrix']
    print(f"                 Predicted Normal (0)   Predicted Anomaly (1)")
    print(f" Actual Normal (0)       {cm[0, 0]:<20}   {cm[0, 1]:<20}")
    print(f" Actual Anomaly (1)      {cm[1, 0]:<20}   {cm[1, 1]:<20}")
    print("-" * 64)
    print(" Feature Importances:")
    sorted_feats = sorted(metrics['feature_importances'].items(), key=lambda x: x[1], reverse=True)
    for rank, (feat, imp) in enumerate(sorted_feats, 1):
        print(f"  {rank}. {feat:<16}: {imp * 100.0:5.2f}%")
    print("=" * 64 + "\n")


# =============================================================================
# 5. Standalone AST C++ Header Code Generator (include/model.h)
# =============================================================================

def generate_cpp_header(
    clf: RandomForestClassifier,
    feature_names: List[str] = FEATURE_COLUMNS,
    accuracy: float = 1.0,
) -> str:
    """
    Transpile scikit-learn RandomForestClassifier into a pure C++ header.
    - Zero dynamic memory allocation (0 bytes heap).
    - Namespace: Eloquent::ML::Port::RandomForest.
    - Microsecond inference latency.
    - Compatible with GCC / Clang / PlatformIO Xtensa GCC.
    """
    num_features = len(feature_names)
    num_trees = len(clf.estimators_)

    tree_methods: List[str] = []
    vote_tallies: List[str] = []
    proba_tallies: List[str] = []

    for i, estimator in enumerate(clf.estimators_):
        tree = estimator.tree_

        def emit_node(node_id: int, depth: int) -> str:
            indent = "    " * depth
            # Leaf node detection (children_left == -1 / TREE_LEAF)
            if tree.children_left[node_id] == -1:
                class_idx = int(np.argmax(tree.value[node_id][0]))
                return f"{indent}return {class_idx};\n"
            else:
                feat = int(tree.feature[node_id])
                thresh = float(tree.threshold[node_id])
                feat_name = feature_names[feat] if feat < len(feature_names) else f"feat_{feat}"
                left_sub = emit_node(tree.children_left[node_id], depth + 1)
                right_sub = emit_node(tree.children_right[node_id], depth + 1)

                code = f"{indent}if (x[{feat}] <= {thresh:.6f}f) {{ // {feat_name}\n"
                code += left_sub
                code += f"{indent}}} else {{\n"
                code += right_sub
                code += f"{indent}}}\n"
                return code

        tree_body = emit_node(0, 2)
        tree_method = (
            f"    static inline int predict_tree_{i}(const float* x) {{\n"
            f"{tree_body}"
            f"    }}"
        )
        tree_methods.append(tree_method)
        vote_tallies.append(f"        votes[predict_tree_{i}(x)]++;")
        proba_tallies.append(f"        if (predict_tree_{i}(x) == 1) anomaly_votes++;")

    all_tree_methods = "\n\n".join(tree_methods)
    all_vote_tallies = "\n".join(vote_tallies)
    all_proba_tallies = "\n".join(proba_tallies)

    header_template = f"""#pragma once
/**
 * @file model.h
 * @brief Auto-generated TinyML Random Forest classifier for ESP32 Predictive Maintenance.
 * @note Generated by mlops/run_pipeline.py. Zero heap allocation, microsecond inference.
 * @note Test Accuracy: {accuracy * 100.0:.2f}%, Trees: {num_trees}, Features: {num_features}
 */
#include <stdint.h>
#include <stddef.h>

namespace Eloquent {{
namespace ML {{
namespace Port {{

class RandomForest {{
public:
    static constexpr size_t NUM_FEATURES = {num_features};
    static constexpr size_t NUM_TREES = {num_trees};
    static constexpr size_t NUM_CLASSES = 2;

    enum ClassLabel {{
        NORMAL = 0,
        ANOMALY = 1
    }};

    enum FeatureIndex {{
        FEATURE_RMS = 0,
        FEATURE_PEAK = 1,
        FEATURE_DOMINANT_FREQ = 2,
        FEATURE_TEMPERATURE = 3
    }};

private:
{all_tree_methods}

public:
    /**
     * @brief Predict class label (0 = NORMAL, 1 = ANOMALY)
     * @param x Pointer to float array [rms, peak, dominant_freq, temperature]
     * @return int Class label (0 or 1)
     */
    static int predict(const float* x) {{
        int votes[2] = {{0, 0}};
{all_vote_tallies}
        return (votes[1] > votes[0]) ? 1 : 0;
    }}

    /**
     * @brief Predict anomaly probability (fraction of trees voting ANOMALY)
     * @param x Pointer to float array of features
     * @return float Probability in range [0.0, 1.0]
     */
    static inline float predict_proba(const float* x) {{
        int anomaly_votes = 0;
{all_proba_tallies}
        return static_cast<float>(anomaly_votes) / {num_trees}.0f;
    }}

    /**
     * @brief Predict class label from individual feature values
     */
    static inline int predict(float rms, float peak, float freq, float temp) {{
        const float x[4] = {{rms, peak, freq, temp}};
        return predict(x);
    }}
}};

}} // namespace Port
}} // namespace ML
}} // namespace Eloquent

// C-compatible standalone wrapper functions
static inline int model_predict(const float* features) {{
    return Eloquent::ML::Port::RandomForest::predict(features);
}}

static inline int model_predict_values(float rms, float peak, float freq, float temp) {{
    return Eloquent::ML::Port::RandomForest::predict(rms, peak, freq, temp);
}}
"""
    return header_template


def save_cpp_header(header_content: str, output_path: Path) -> None:
    """Save emitted C++ header to file."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(header_content)
    print(f"Generated C++ model header: {output_path}")


# =============================================================================
# 6. Verification & Self-Testing
# =============================================================================

def verify_cpp_compilation_and_equivalence(
    header_path: Path,
    clf: RandomForestClassifier,
    X_test: np.ndarray,
    y_test: np.ndarray,
) -> bool:
    """
    Compiles a small C++ test binary using g++ (if installed), passes test samples
    to the generated C++ model, and asserts 100% equivalence with scikit-learn.
    """
    # 1. First test Python AST parity
    py_preds = clf.predict(X_test)

    # 2. Check if g++ is available for host compilation check
    gpp_available = subprocess.run(["which", "g++"], capture_output=True).returncode == 0
    if not gpp_available:
        print("[Verification] g++ compiler not found. Skipped C++ compilation check.")
        return True

    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_dir_path = Path(tmpdir)
        cpp_src_path = tmp_dir_path / "test_model.cpp"
        bin_path = tmp_dir_path / "test_model_bin"

        # Generate C++ test runner that tests each sample in X_test
        samples_code = []
        for i, row in enumerate(X_test):
            samples_code.append(
                f"    {{\n"
                f"        const float x[4] = {{{row[0]:.6f}f, {row[1]:.6f}f, {row[2]:.6f}f, {row[3]:.6f}f}};\n"
                f"        int pred = Eloquent::ML::Port::RandomForest::predict(x);\n"
                f"        int pred_val = Eloquent::ML::Port::RandomForest::predict(x[0], x[1], x[2], x[3]);\n"
                f"        int c_pred = model_predict(x);\n"
                f"        int c_pred_val = model_predict_values(x[0], x[1], x[2], x[3]);\n"
                f"        float proba = Eloquent::ML::Port::RandomForest::predict_proba(x);\n"
                f"        if (pred != pred_val || pred != c_pred || pred != c_pred_val) {{\n"
                f"            return 10 + {i};\n"
                f"        }}\n"
                f"        if (proba < 0.0f || proba > 1.0f) {{\n"
                f"            return 200 + {i};\n"
                f"        }}\n"
                f"        std::cout << pred << std::endl;\n"
                f"    }}\n"
            )

        cpp_code = f"""#include <iostream>
#include "{header_path.resolve()}"

int main() {{
{"".join(samples_code)}
    return 0;
}}
"""
        with open(cpp_src_path, "w", encoding="utf-8") as f:
            f.write(cpp_code)

        # Compile with strict flags: -Wall -Wextra -Werror -O2
        compile_cmd = [
            "g++", "-Wall", "-Wextra", "-Werror", "-O2",
            "-std=c++17", str(cpp_src_path), "-o", str(bin_path)
        ]
        comp_res = subprocess.run(compile_cmd, capture_output=True, text=True)
        if comp_res.returncode != 0:
            print(f"[Verification FAIL] C++ Compilation Error:\n{comp_res.stderr}")
            return False

        # Run test binary and collect C++ predictions
        run_res = subprocess.run([str(bin_path)], capture_output=True, text=True)
        if run_res.returncode != 0:
            print(f"[Verification FAIL] C++ Runtime Error: exit code {run_res.returncode}")
            return False

        cpp_preds = [int(line.strip()) for line in run_res.stdout.strip().split("\n") if line.strip()]
        if len(cpp_preds) != len(py_preds):
            print(f"[Verification FAIL] Prediction count mismatch: C++={len(cpp_preds)}, Python={len(py_preds)}")
            return False

        mismatches = sum(1 for cp, pp in zip(cpp_preds, py_preds) if cp != pp)
        if mismatches > 0:
            print(f"[Verification FAIL] Found {mismatches} mismatches between C++ and Scikit-Learn predictions!")
            return False

        print("[Verification PASS] C++ header compiles cleanly (-Wall -Wextra -Werror) and achieves 100% equivalence with Scikit-Learn!")
        return True


# =============================================================================
# 7. Unified CLI Main Entrypoint
# =============================================================================

def resolve_default_paths() -> Tuple[Path, Path]:
    """Find default locations for mlops/data and include/model.h relative to workspace."""
    script_dir = Path(__file__).resolve().parent
    # Check if script is inside maintenance_moteur/mlops
    if script_dir.name == "mlops":
        project_root = script_dir.parent
    elif (script_dir / "maintenance_moteur").exists():
        project_root = script_dir / "maintenance_moteur"
    else:
        project_root = script_dir.parent

    default_data_dir = project_root / "mlops" / "data"
    default_output_header = project_root / "include" / "model.h"
    return default_data_dir, default_output_header


def main() -> int:
    default_data_dir, default_output_header = resolve_default_paths()

    parser = argparse.ArgumentParser(
        description="Automated MLOps Pipeline: ESP32 DC Motor Predictive Maintenance",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--mock", "--synthetic",
        action="store_true",
        dest="mock",
        help="Bypass hardware serial; generate synthetic physics datasets (default: auto-fallback if no device)",
    )
    parser.add_argument(
        "--port",
        type=str,
        default="auto",
        help="Serial port path (e.g. /dev/ttyUSB0, COM3, or 'auto' for autodetection)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate (default: 115200)",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=200,
        help="Number of samples per class to collect (default: 200)",
    )
    parser.add_argument(
        "--output-header",
        type=str,
        default=str(default_output_header),
        help=f"Path to generated C++ header file (default: {default_output_header})",
    )
    parser.add_argument(
        "--data-dir",
        type=str,
        default=str(default_data_dir),
        help=f"Directory to store normal.csv and anomaly.csv (default: {default_data_dir})",
    )
    parser.add_argument(
        "--skip-acquire",
        action="store_true",
        help="Skip data acquisition phase; train immediately from existing CSVs",
    )
    parser.add_argument(
        "--eval-only",
        action="store_true",
        help="Train and evaluate model performance without writing C++ header",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for reproducible dataset generation and model training (default: 42)",
    )
    parser.add_argument(
        "--non-interactive",
        action="store_true",
        help="Do not prompt for user confirmation; use defaults / auto-fallback",
    )

    args = parser.parse_args()

    data_dir = Path(args.data_dir).resolve()
    output_header = Path(args.output_header).resolve()
    normal_csv = data_dir / "normal.csv"
    anomaly_csv = data_dir / "anomaly.csv"

    print("=" * 64)
    print(" ESP32 DC MOTOR PREDICTIVE MAINTENANCE — MLOps PIPELINE")
    print("=" * 64)
    print(f" Mode:           {'Synthetic Physics / Mock' if args.mock else 'Hardware Serial / Auto-Fallback'}")
    print(f" Target Header:  {output_header}")
    print(f" Data Directory: {data_dir}")
    print(f" Samples/Class:  {args.samples}")
    print(f" Random Seed:    {args.seed}")
    print("=" * 64)

    # -------------------------------------------------------------------------
    # Step 1: Data Acquisition or Synthetic Generation
    # -------------------------------------------------------------------------
    df_normal: Optional[pd.DataFrame] = None
    df_anomaly: Optional[pd.DataFrame] = None

    if args.skip_acquire:
        print("\n[Phase 1] Skipping acquisition. Loading existing CSV datasets...")
        if not normal_csv.exists() or not anomaly_csv.exists():
            print(f"Error: Datasets missing at {normal_csv} or {anomaly_csv}. Cannot skip acquisition.")
            return 1
        df_normal = pd.read_csv(normal_csv)
        df_anomaly = pd.read_csv(anomaly_csv)
    else:
        use_mock = args.mock
        selected_port = None

        if not use_mock:
            if args.port.lower() == "auto":
                selected_port = find_esp32_port()
            else:
                selected_port = args.port

            if not selected_port:
                if args.non_interactive or not sys.stdin.isatty():
                    print("\n[Phase 1] No active serial port detected. Auto-switching to synthetic generator.")
                    use_mock = True
                else:
                    ans = input("\n[Phase 1] No serial port found. Switch to synthetic data generation? [Y/n]: ").strip().lower()
                    if ans in ("", "y", "yes"):
                        use_mock = True
                    else:
                        print("Aborting acquisition.")
                        return 1

        if use_mock:
            print("\n[Phase 1] Generating physics-accurate synthetic datasets...")
            df_normal, df_anomaly = generate_synthetic_datasets(
                data_dir=data_dir, n_samples=args.samples, seed=args.seed
            )
            print(f"Generated {len(df_normal)} normal samples -> {normal_csv}")
            print(f"Generated {len(df_anomaly)} anomaly samples -> {anomaly_csv}")
        else:
            print(f"\n[Phase 1] Using serial hardware on port: {selected_port}")
            try:
                df_normal = acquire_serial_dataset(
                    port=selected_port,
                    baud=args.baud,
                    n_samples=args.samples,
                    phase_title="Phase 1: Normal Motor Operation Baseline",
                    instructions=[
                        "Ensure DC motor is running nominally at ~2000 RPM.",
                        "No artificial vibration or thermal drift injected.",
                    ],
                    output_path=normal_csv,
                    command_to_send="RESET",
                    non_interactive=args.non_interactive,
                )

                df_anomaly = acquire_serial_dataset(
                    port=selected_port,
                    baud=args.baud,
                    n_samples=args.samples,
                    phase_title="Phase 2: Anomaly Operation Acquisition",
                    instructions=[
                        "Inject mechanical vibration fault (send INJECT_VIB or apply physical load).",
                        "Inject thermal anomaly (send INJECT_TEMP or apply heat).",
                    ],
                    output_path=anomaly_csv,
                    command_to_send="INJECT_VIB",
                    non_interactive=args.non_interactive,
                )
            except Exception as e:
                print(f"Serial acquisition error: {e}")
                if args.non_interactive:
                    print("Falling back to synthetic data generation...")
                    df_normal, df_anomaly = generate_synthetic_datasets(
                        data_dir=data_dir, n_samples=args.samples, seed=args.seed
                    )
                else:
                    return 1

    # -------------------------------------------------------------------------
    # Step 2: Training & Validation
    # -------------------------------------------------------------------------
    print("\n[Phase 2] Training TinyML Random Forest Classifier (n_estimators=3, max_depth=4)...")
    clf, metrics, X_test, y_test = train_model(df_normal, df_anomaly, seed=args.seed)
    display_training_report(metrics)

    if metrics['accuracy'] < 0.95:
        print(f"[ERROR] Model accuracy {metrics['accuracy'] * 100:.2f}% is below required 95.0% threshold!")
        return 1

    # -------------------------------------------------------------------------
    # Step 3: C++ Header Generation
    # -------------------------------------------------------------------------
    if not args.eval_only:
        print(f"[Phase 3] Generating standalone C++ AST header: {output_header}")
        header_code = generate_cpp_header(
            clf=clf,
            feature_names=FEATURE_COLUMNS,
            accuracy=metrics['accuracy'],
        )
        save_cpp_header(header_code, output_header)

        # ---------------------------------------------------------------------
        # Step 4: Verification & Compiler Check
        # ---------------------------------------------------------------------
        print("\n[Phase 4] Verifying C++ compilation & inference equivalence...")
        success = verify_cpp_compilation_and_equivalence(
            header_path=output_header,
            clf=clf,
            X_test=X_test,
            y_test=y_test,
        )
        if not success:
            print("[ERROR] Verification failed!")
            return 1

    print("\n" + "=" * 64)
    print(" MLOps PIPELINE COMPLETED SUCCESSFULLY!")
    print("=" * 64 + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
