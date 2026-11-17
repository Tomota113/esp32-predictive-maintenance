"""Unit tests for AST C++ code generator."""
import unittest
import pandas as pd
from pathlib import Path

from maintenance_moteur.mlops.run_pipeline import train_model, generate_cpp_header

DATA_DIR = Path(__file__).resolve().parent.parent / "data"


class TestModelExport(unittest.TestCase):
    def test_cpp_header_structure(self):
        """Verify generated C++ header contains all required definitions and namespaces."""
        df_normal = pd.read_csv(DATA_DIR / "normal.csv")
        df_anomaly = pd.read_csv(DATA_DIR / "anomaly.csv")

        clf, metrics, _, _ = train_model(df_normal, df_anomaly, seed=42)
        header = generate_cpp_header(clf, accuracy=metrics['accuracy'])

        # Check Header guard and namespaces
        self.assertIn("#pragma once", header)
        self.assertIn("namespace Eloquent", header)
        self.assertIn("namespace ML", header)
        self.assertIn("namespace Port", header)
        self.assertIn("class RandomForest", header)

        # Check Static constants
        self.assertIn("NUM_FEATURES = 4;", header)
        self.assertIn("NUM_TREES = 3;", header)
        self.assertIn("NUM_CLASSES = 2;", header)

        # Check Predict methods
        self.assertIn("static int predict(const float* x)", header)
        self.assertIn("static inline float predict_proba(const float* x)", header)
        self.assertIn("static inline int predict(float rms, float peak, float freq, float temp)", header)

        # Check C wrappers
        self.assertIn("model_predict(const float* features)", header)
        self.assertIn("model_predict_values(float rms, float peak, float freq, float temp)", header)

        # Check Tree methods
        for i in range(3):
            self.assertIn(f"predict_tree_{i}", header)


if __name__ == "__main__":
    unittest.main()
