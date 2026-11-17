"""Unit tests for MLOps dataset validation and synthetic physics generator."""
import unittest
import pandas as pd
import numpy as np
from pathlib import Path

from maintenance_moteur.mlops.run_pipeline import (
    generate_synthetic_normal,
    generate_synthetic_anomaly,
    generate_synthetic_datasets,
    FEATURE_COLUMNS,
    NORMAL_DIST,
    VIB_FAULT_DIST,
    THERMAL_FAULT_DIST,
)

DATA_DIR = Path(__file__).resolve().parent.parent / "data"


class TestDataset(unittest.TestCase):
    def test_csv_datasets_exist(self):
        """Verify that normal.csv and anomaly.csv exist in data directory."""
        normal_csv = DATA_DIR / "normal.csv"
        anomaly_csv = DATA_DIR / "anomaly.csv"
        self.assertTrue(normal_csv.exists(), f"normal.csv missing at {normal_csv}")
        self.assertTrue(anomaly_csv.exists(), f"anomaly.csv missing at {anomaly_csv}")

    def test_dataset_shapes_and_columns(self):
        """Verify exact sample count (200) and 4 required feature columns."""
        df_normal = pd.read_csv(DATA_DIR / "normal.csv")
        df_anomaly = pd.read_csv(DATA_DIR / "anomaly.csv")

        self.assertEqual(df_normal.shape, (200, 4), f"Unexpected normal.csv shape: {df_normal.shape}")
        self.assertEqual(df_anomaly.shape, (200, 4), f"Unexpected anomaly.csv shape: {df_anomaly.shape}")

        self.assertEqual(list(df_normal.columns), FEATURE_COLUMNS)
        self.assertEqual(list(df_anomaly.columns), FEATURE_COLUMNS)

    def test_no_null_or_infinite_values(self):
        """Verify that datasets have 0 NaN or Inf entries."""
        df_normal = pd.read_csv(DATA_DIR / "normal.csv")
        df_anomaly = pd.read_csv(DATA_DIR / "anomaly.csv")

        self.assertFalse(df_normal.isnull().any().any(), "NaN found in normal.csv")
        self.assertFalse(df_anomaly.isnull().any().any(), "NaN found in anomaly.csv")
        self.assertTrue(np.isfinite(df_normal.values).all(), "Inf found in normal.csv")
        self.assertTrue(np.isfinite(df_anomaly.values).all(), "Inf found in anomaly.csv")

    def test_synthetic_normal_bounds(self):
        """Verify synthetic normal generator produces values within specified physical bounds."""
        df = generate_synthetic_normal(n_samples=500, seed=42)
        self.assertEqual(len(df), 500)
        for feat, bounds in NORMAL_DIST.items():
            self.assertGreaterEqual(df[feat].min(), bounds['min'] - 1e-4)
            self.assertLessEqual(df[feat].max(), bounds['max'] + 1e-4)

    def test_synthetic_anomaly_bounds(self):
        """Verify synthetic anomaly generator produces values within specified physical bounds."""
        df = generate_synthetic_anomaly(n_samples=500, seed=42)
        self.assertEqual(len(df), 500)
        self.assertLessEqual(df['rms'].max(), max(VIB_FAULT_DIST['rms']['max'], THERMAL_FAULT_DIST['rms']['max']))
        self.assertLessEqual(df['temperature'].max(), max(VIB_FAULT_DIST['temperature']['max'], THERMAL_FAULT_DIST['temperature']['max']))


if __name__ == "__main__":
    unittest.main()
