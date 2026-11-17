#pragma once
#include <Arduino.h>
#include <arduinoFFT.h>
#include <cstring>
#include <cmath>
#include "config.h"

/**
 * @brief Static circular ring buffer with zero dynamic memory allocation.
 * @tparam T Data type of buffered samples.
 * @tparam N Fixed capacity of the buffer.
 */
template <typename T, size_t N>
class RingBuffer {
private:
    T buffer_[N];
    size_t head_ = 0;
    size_t count_ = 0;

public:
    RingBuffer() {
        clear();
    }

    /**
     * @brief Push a new sample into the ring buffer (overwriting oldest when full).
     */
    void push(T sample) {
        buffer_[head_] = sample;
        head_ = (head_ + 1) % N;
        if (count_ < N) {
            count_++;
        }
    }

    /**
     * @brief Check whether the buffer contains a complete window (N samples).
     */
    bool isFull() const {
        return count_ >= N;
    }

    /**
     * @brief Number of samples currently in the buffer.
     */
    size_t size() const {
        return count_;
    }

    /**
     * @brief Maximum capacity of the buffer.
     */
    size_t capacity() const {
        return N;
    }

    /**
     * @brief Clear all samples from the buffer.
     */
    void clear() {
        head_ = 0;
        count_ = 0;
        std::memset(buffer_, 0, sizeof(buffer_));
    }

    /**
     * @brief Extract samples into a linear array in chronological order (oldest to newest).
     * @param[out] dest Destination array with at least count_ elements.
     */
    void extractWindow(T* dest) const {
        if (count_ == 0 || dest == nullptr) return;
        size_t start = (head_ + N - count_) % N;
        for (size_t i = 0; i < count_; ++i) {
            dest[i] = buffer_[(start + i) % N];
        }
    }
};

/**
 * @brief Physical feature vector extracted by real-time DSP.
 */
struct DSPFeatures {
    float rms;                 // Root Mean Square vibration energy (m/s^2)
    float peak_to_peak;        // Peak-to-Peak amplitude (m/s^2)
    float peak_absolute;       // Maximum absolute deviation (m/s^2)
    float dominant_frequency;  // Fundamental vibration harmonic (Hz)
    float temperature;         // Monitored motor temperature (°C)

    /**
     * @brief Serialize features into a 4-element array for TinyML model inference.
     * Order: [RMS, Peak, Dominant_Frequency, Temperature]
     */
    void toFeatureArray(float* out_features) const {
        if (out_features != nullptr) {
            out_features[0] = rms;
            out_features[1] = peak_absolute;
            out_features[2] = dominant_frequency;
            out_features[3] = temperature;
        }
    }
};

/**
 * @brief Real-time digital signal processing pipeline for 200 Hz accelerometer data.
 */
class DSPPipeline {
private:
    RingBuffer<float, WINDOW_SIZE> ring_buffer_;
    double vReal_[WINDOW_SIZE];
    double vImag_[WINDOW_SIZE];
    ArduinoFFT<double> fft_;

public:
    DSPPipeline();

    /**
     * @brief Push a new accelerometer sample into the DSP buffer.
     */
    void pushSample(float z_sample);

    /**
     * @brief Check if a full 64-sample window is ready for feature extraction.
     */
    bool isWindowReady() const;

    /**
     * @brief Reset the DSP ring buffer and internal state.
     */
    void reset();

    /**
     * @brief Execute complete DSP feature extraction on the current 64-sample window.
     * Computes RMS, Peak-to-Peak, Peak Absolute, 64-point FFT with Hamming windowing,
     * and dominant harmonic search excluding DC bin.
     * @param current_temp Current temperature from DHT11 (°C).
     * @return Extracted DSPFeatures struct.
     */
    DSPFeatures computeFeatures(float current_temp);

    /**
     * @brief Const reference to the internal ring buffer for diagnostic testing.
     */
    const RingBuffer<float, WINDOW_SIZE>& getRingBuffer() const {
        return ring_buffer_;
    }
};
