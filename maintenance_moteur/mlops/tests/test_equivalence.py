"""Unit tests for inference equivalence between scikit-learn and C++ model.h, plus telemetry parsing."""
import unittest
import tempfile
import pandas as pd
import numpy as np
from pathlib import Path

from maintenance_moteur.mlops.run_pipeline import (
    train_model,
    generate_cpp_header,
    save_cpp_header,
    verify_cpp_compilation_and_equivalence,
    parse_telemetry_line,
)

DATA_DIR = Path(__file__).resolve().parent.parent / "data"
INCLUDE_DIR = Path(__file__).resolve().parent.parent.parent / "include"


class TestEquivalence(unittest.TestCase):
    def test_telemetry_parser_structured(self):
        """Verify parsing of standard projector telemetry string."""
        line = "[ETAT: NORMAL] | RMS: 0.342 | Peak: 0.812 | Freq: 33.2 | Temp: 26.5 | Inference: 0"
        features = parse_telemetry_line(line)
        self.assertIsNotNone(features)
        self.assertTrue(np.isclose(features, [0.342, 0.812, 33.2, 26.5]).all())

    def test_telemetry_parser_csv(self):
        """Verify parsing of tagged CSV and raw CSV telemetry."""
        line_tagged = "DATA,0.450,1.120,31.25,28.4"
        line_raw = "0.450,1.120,31.25,28.4"

        f1 = parse_telemetry_line(line_tagged)
        f2 = parse_telemetry_line(line_raw)

        self.assertIsNotNone(f1)
        self.assertTrue(np.isclose(f1, [0.450, 1.120, 31.25, 28.4]).all())
        self.assertIsNotNone(f2)
        self.assertTrue(np.isclose(f2, [0.450, 1.120, 31.25, 28.4]).all())

    def test_telemetry_parser_invalid_lines(self):
        """Verify parser ignores unparseable noise/boot lines."""
        self.assertIsNone(parse_telemetry_line(""))
        self.assertIsNone(parse_telemetry_line("rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)"))
        self.assertIsNone(parse_telemetry_line("RMS: 0.342 | Peak: "))

    def test_full_model_cpp_equivalence(self):
        """Verify generated C++ header compiles cleanly and matches scikit-learn predictions 100%."""
        df_normal = pd.read_csv(DATA_DIR / "normal.csv")
        df_anomaly = pd.read_csv(DATA_DIR / "anomaly.csv")

        clf, metrics, X_test, y_test = train_model(df_normal, df_anomaly, seed=42)
        header_code = generate_cpp_header(clf, accuracy=metrics['accuracy'])

        with tempfile.TemporaryDirectory() as tmpdir:
            header_file = Path(tmpdir) / "model.h"
            save_cpp_header(header_code, header_file)

            passed = verify_cpp_compilation_and_equivalence(
                header_path=header_file,
                clf=clf,
                X_test=X_test,
                y_test=y_test,
            )
            self.assertTrue(passed, "C++ compilation or prediction equivalence failed.")


if __name__ == "__main__":
    unittest.main()
