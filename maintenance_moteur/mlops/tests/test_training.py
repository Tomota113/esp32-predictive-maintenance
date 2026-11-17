"""Unit tests for model training and evaluation metrics."""
import unittest
import pandas as pd
import numpy as np
from pathlib import Path

from maintenance_moteur.mlops.run_pipeline import train_model

DATA_DIR = Path(__file__).resolve().parent.parent / "data"


class TestTraining(unittest.TestCase):
    def test_model_training_hyperparameters(self):
        """Verify model hyperparameters match 3 trees and max depth 4."""
        df_normal = pd.read_csv(DATA_DIR / "normal.csv")
        df_anomaly = pd.read_csv(DATA_DIR / "anomaly.csv")

        clf, metrics, X_test, y_test = train_model(df_normal, df_anomaly, seed=42)

        self.assertEqual(clf.n_estimators, 3)
        self.assertEqual(clf.max_depth, 4)
        self.assertEqual(clf.random_state, 42)
        self.assertEqual(len(clf.estimators_), 3)

    def test_model_training_accuracy_threshold(self):
        """Verify test accuracy exceeds 95% threshold (target 100%)."""
        df_normal = pd.read_csv(DATA_DIR / "normal.csv")
        df_anomaly = pd.read_csv(DATA_DIR / "anomaly.csv")

        clf, metrics, X_test, y_test = train_model(df_normal, df_anomaly, seed=42)

        self.assertGreaterEqual(metrics['accuracy'], 0.95)
        self.assertGreaterEqual(metrics['precision'], 0.95)
        self.assertGreaterEqual(metrics['recall'], 0.95)
        self.assertGreaterEqual(metrics['f1'], 0.95)

    def test_reproducibility(self):
        """Verify exact deterministic reproducibility across repeated runs."""
        df_normal = pd.read_csv(DATA_DIR / "normal.csv")
        df_anomaly = pd.read_csv(DATA_DIR / "anomaly.csv")

        clf1, metrics1, _, _ = train_model(df_normal, df_anomaly, seed=42)
        clf2, metrics2, _, _ = train_model(df_normal, df_anomaly, seed=42)

        self.assertEqual(metrics1['accuracy'], metrics2['accuracy'])
        self.assertTrue(np.allclose(clf1.feature_importances_, clf2.feature_importances_))


if __name__ == "__main__":
    unittest.main()
