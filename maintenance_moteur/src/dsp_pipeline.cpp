#include "dsp_pipeline.h"
#include <cmath>
#include <cstring>
#include <algorithm>

DSPPipeline::DSPPipeline()
    : fft_(vReal_, vImag_, WINDOW_SIZE, SAMPLING_FREQ_HZ) {
    reset();
}

void DSPPipeline::pushSample(float z_sample) {
    ring_buffer_.push(z_sample);
}

bool DSPPipeline::isWindowReady() const {
    return ring_buffer_.isFull();
}

void DSPPipeline::reset() {
    ring_buffer_.clear();
    std::memset(vReal_, 0, sizeof(vReal_));
    std::memset(vImag_, 0, sizeof(vImag_));
}

DSPFeatures DSPPipeline::computeFeatures(float current_temp) {
    DSPFeatures features;
    features.rms = 0.0f;
    features.peak_to_peak = 0.0f;
    features.peak_absolute = 0.0f;
    features.dominant_frequency = 0.0f;
    features.temperature = current_temp;

    if (!ring_buffer_.isFull()) {
        return features;
    }

    // Extract chronological window of samples (oldest to newest)
    float window[WINDOW_SIZE];
    ring_buffer_.extractWindow(window);

    // 1. Calculate local window mean
    double sum = 0.0;
    for (size_t i = 0; i < WINDOW_SIZE; ++i) {
        sum += static_cast<double>(window[i]);
    }
    double mean = sum / static_cast<double>(WINDOW_SIZE);

    // 2. Compute RMS, Peak-to-Peak, and Absolute Peak
    double sum_sq = 0.0;
    float min_val = window[0];
    float max_val = window[0];
    float peak_abs = 0.0f;

    for (size_t i = 0; i < WINDOW_SIZE; ++i) {
        float raw_val = window[i];
        float centered = static_cast<float>(static_cast<double>(raw_val) - mean);

        sum_sq += static_cast<double>(centered) * static_cast<double>(centered);

        if (raw_val < min_val) min_val = raw_val;
        if (raw_val > max_val) max_val = raw_val;

        float abs_val = std::fabs(centered);
        if (abs_val > peak_abs) {
            peak_abs = abs_val;
        }

        // Populate FFT buffers with zero-mean centered vibration signal
        vReal_[i] = static_cast<double>(centered);
        vImag_[i] = 0.0;
    }

    features.rms = static_cast<float>(std::sqrt(sum_sq / static_cast<double>(WINDOW_SIZE)));
    features.peak_to_peak = max_val - min_val;
    features.peak_absolute = peak_abs;

    // 3. Apply Hamming Windowing to attenuate spectral leakage
    fft_.windowing(FFTWindow::Hamming, FFTDirection::Forward);

    // 4. Compute Radix-2 Cooley-Tukey FFT
    fft_.compute(FFTDirection::Forward);

    // 5. Convert Complex FFT Coefficients to Magnitude Spectrum
    fft_.complexToMagnitude();

    // 6. Dominant Frequency Search (Suppressing DC Bin k = 0)
    // Nyquist limit = WINDOW_SIZE / 2 = 32 bins (0 Hz to 100 Hz, 3.125 Hz/bin)
    double max_mag = 0.0;
    size_t max_bin = 1;
    for (size_t k = 1; k < WINDOW_SIZE / 2; ++k) {
        if (vReal_[k] > max_mag) {
            max_mag = vReal_[k];
            max_bin = k;
        }
    }

    // Frequency resolution delta_f = SAMPLING_FREQ_HZ / WINDOW_SIZE = 200.0 / 64 = 3.125 Hz
    if (max_mag > 1e-6) {
        features.dominant_frequency = static_cast<float>(max_bin) * (SAMPLING_FREQ_HZ / static_cast<float>(WINDOW_SIZE));
    } else {
        features.dominant_frequency = 0.0f;
    }

    return features;
}
