#pragma once
#include <cmath>
#include <cstdint>
#include <algorithm>

// =============================================================================
// Native Cooley-Tukey Radix-2 FFT Engine (arduinoFFT Compatible)
// =============================================================================

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Legacy & v2.x Compatibility Enums and Constants
#define FFT_FORWARD 0x01
#define FFT_REVERSE 0x00

#define FFT_WIN_TYP_RECTANGLE 0x00
#define FFT_WIN_TYP_HAMMING   0x01
#define FFT_WIN_TYP_HANN      0x02
#define FFT_WIN_TYP_TRIANGLE  0x03
#define FFT_WIN_TYP_BLACKMAN  0x04
#define FFT_WIN_TYP_FLT_TOP   0x05
#define FFT_WIN_TYP_WELCH     0x06

enum class FFTWindow {
    Rectangle = 0,
    Hamming   = 1,
    Hann      = 2,
    Triangle  = 3,
    Blackman  = 4,
    FlatTop   = 5,
    Cosine    = 6,
    Welch     = 7
};

enum class FFTDirection {
    Forward = 1,
    Reverse = 0
};

template <typename T>
class ArduinoFFT {
private:
    T *vReal = nullptr;
    T *vImag = nullptr;
    uint16_t samples = 0;
    T samplingFrequency = 0;

    static uint16_t reverse_bits(uint16_t x, uint8_t bits) {
        uint16_t y = 0;
        for (uint8_t i = 0; i < bits; ++i) {
            y = (y << 1) | (x & 1);
            x >>= 1;
        }
        return y;
    }

    static uint8_t compute_power_of_two(uint16_t n) {
        uint8_t power = 0;
        while ((1U << power) < n) {
            power++;
        }
        return power;
    }

public:
    ArduinoFFT() = default;

    ArduinoFFT(T *vR, T *vI, uint16_t s, T sF)
        : vReal(vR), vImag(vI), samples(s), samplingFrequency(sF) {}

    // Method to set data buffers after default construction
    void setBuffers(T *vR, T *vI, uint16_t s, T sF) {
        vReal = vR;
        vImag = vI;
        samples = s;
        samplingFrequency = sF;
    }

    // -------------------------------------------------------------------------
    // Windowing Functions
    // -------------------------------------------------------------------------
    static void windowing(T *vR, uint16_t s, FFTWindow windowType, FFTDirection dir = FFTDirection::Forward) {
        if (!vR || s == 0) return;
        (void)dir;

        if (windowType == FFTWindow::Rectangle) {
            return;
        }

        const double two_pi = 2.0 * M_PI;
        const double samples_minus_1 = static_cast<double>(s - 1);

        for (uint16_t i = 0; i < s; ++i) {
            double ratio = (samples_minus_1 > 0) ? (static_cast<double>(i) / samples_minus_1) : 0.0;
            double factor = 1.0;

            switch (windowType) {
                case FFTWindow::Hamming:
                    factor = 0.54 - 0.46 * std::cos(two_pi * ratio);
                    break;
                case FFTWindow::Hann:
                    factor = 0.50 * (1.0 - std::cos(two_pi * ratio));
                    break;
                case FFTWindow::Triangle:
                    factor = 1.0 - std::abs((2.0 * static_cast<double>(i) - samples_minus_1) / samples_minus_1);
                    break;
                case FFTWindow::Blackman:
                    factor = 0.42 - 0.50 * std::cos(two_pi * ratio) + 0.08 * std::cos(2.0 * two_pi * ratio);
                    break;
                case FFTWindow::FlatTop:
                    factor = 0.21557895
                           - 0.41663158 * std::cos(two_pi * ratio)
                           + 0.277263158 * std::cos(2.0 * two_pi * ratio)
                           - 0.083578947 * std::cos(3.0 * two_pi * ratio)
                           + 0.006947368 * std::cos(4.0 * two_pi * ratio);
                    break;
                case FFTWindow::Cosine:
                    factor = std::sin(M_PI * ratio);
                    break;
                case FFTWindow::Welch: {
                    double term = (static_cast<double>(i) - 0.5 * samples_minus_1) / (0.5 * samples_minus_1);
                    factor = 1.0 - term * term;
                    break;
                }
                default:
                    factor = 1.0;
                    break;
            }

            vR[i] = static_cast<T>(vR[i] * factor);
        }
    }

    void windowing(FFTWindow windowType, FFTDirection dir = FFTDirection::Forward) {
        windowing(vReal, samples, windowType, dir);
    }

    void windowing(uint8_t windowType, uint8_t dir = FFT_FORWARD) {
        windowing(vReal, samples, static_cast<FFTWindow>(windowType), static_cast<FFTDirection>(dir));
    }

    void Windowing(FFTWindow windowType, FFTDirection dir = FFTDirection::Forward) {
        windowing(windowType, dir);
    }

    void Windowing(uint8_t windowType, uint8_t dir = FFT_FORWARD) {
        windowing(windowType, dir);
    }

    // -------------------------------------------------------------------------
    // Compute FFT (Cooley-Tukey Radix-2)
    // -------------------------------------------------------------------------
    static void compute(T *vR, T *vI, uint16_t s, FFTDirection dir = FFTDirection::Forward) {
        if (!vR || !vI || s < 2) return;

        uint8_t power = compute_power_of_two(s);
        uint16_t n = 1U << power;

        // Bit-reversal permutation
        for (uint16_t i = 0; i < n; ++i) {
            uint16_t j = reverse_bits(i, power);
            if (j > i) {
                std::swap(vR[i], vR[j]);
                std::swap(vI[i], vI[j]);
            }
        }

        // Cooley-Tukey Radix-2 Stages
        for (uint8_t stage = 1; stage <= power; ++stage) {
            uint16_t m = 1U << stage;
            uint16_t m2 = m >> 1;
            double theta = (dir == FFTDirection::Forward ? -2.0 : 2.0) * M_PI / static_cast<double>(m);
            double w_r = std::cos(theta);
            double w_i = std::sin(theta);

            for (uint16_t k = 0; k < n; k += m) {
                double u_r = 1.0;
                double u_i = 0.0;

                for (uint16_t j = 0; j < m2; ++j) {
                    uint16_t t = k + j + m2;
                    uint16_t u = k + j;

                    double vR_t = static_cast<double>(vR[t]);
                    double vI_t = static_cast<double>(vI[t]);

                    double t_r = u_r * vR_t - u_i * vI_t;
                    double t_i = u_r * vI_t + u_i * vR_t;

                    double vR_u = static_cast<double>(vR[u]);
                    double vI_u = static_cast<double>(vI[u]);

                    vR[t] = static_cast<T>(vR_u - t_r);
                    vI[t] = static_cast<T>(vI_u - t_i);
                    vR[u] = static_cast<T>(vR_u + t_r);
                    vI[u] = static_cast<T>(vI_u + t_i);

                    double next_u_r = u_r * w_r - u_i * w_i;
                    double next_u_i = u_r * w_i + u_i * w_r;
                    u_r = next_u_r;
                    u_i = next_u_i;
                }
            }
        }

        // Normalization on Inverse FFT
        if (dir == FFTDirection::Reverse) {
            double inv_n = 1.0 / static_cast<double>(n);
            for (uint16_t i = 0; i < n; ++i) {
                vR[i] = static_cast<T>(vR[i] * inv_n);
                vI[i] = static_cast<T>(vI[i] * inv_n);
            }
        }
    }

    void compute(FFTDirection dir = FFTDirection::Forward) {
        compute(vReal, vImag, samples, dir);
    }

    void compute(uint8_t dir) {
        compute(vReal, vImag, samples, static_cast<FFTDirection>(dir));
    }

    void Compute(FFTDirection dir = FFTDirection::Forward) {
        compute(dir);
    }

    void Compute(uint8_t dir) {
        compute(dir);
    }

    // -------------------------------------------------------------------------
    // Complex to Magnitude
    // -------------------------------------------------------------------------
    static void complexToMagnitude(T *vR, T *vI, uint16_t s) {
        if (!vR || !vI || s == 0) return;
        uint16_t half = s / 2;
        for (uint16_t i = 0; i < half; ++i) {
            double r = static_cast<double>(vR[i]);
            double im = static_cast<double>(vI[i]);
            vR[i] = static_cast<T>(std::sqrt(r * r + im * im));
            vI[i] = 0;
        }
    }

    void complexToMagnitude() {
        complexToMagnitude(vReal, vImag, samples);
    }

    void ComplexToMagnitude() {
        complexToMagnitude();
    }

    // -------------------------------------------------------------------------
    // Major Peak Frequency Search
    // -------------------------------------------------------------------------
    static T majorPeak(const T *vR, uint16_t s, T sF) {
        if (!vR || s < 4) return 0;
        T max_val = 0;
        uint16_t max_bin = 1;
        uint16_t half = s / 2;

        // Skip DC component (bin 0)
        for (uint16_t i = 1; i < half; ++i) {
            if (vR[i] > max_val) {
                max_val = vR[i];
                max_bin = i;
            }
        }

        return static_cast<T>(max_bin) * (sF / static_cast<T>(s));
    }

    T majorPeak() const {
        return majorPeak(vReal, samples, samplingFrequency);
    }

    T MajorPeak() const {
        return majorPeak();
    }

    void majorPeak(T *f, T *v) const {
        if (!f || !v || !vReal || samples < 4) return;
        T max_val = 0;
        uint16_t max_bin = 1;
        uint16_t half = samples / 2;

        for (uint16_t i = 1; i < half; ++i) {
            if (vReal[i] > max_val) {
                max_val = vReal[i];
                max_bin = i;
            }
        }

        *f = static_cast<T>(max_bin) * (samplingFrequency / static_cast<T>(samples));
        *v = max_val;
    }

    void MajorPeak(T *f, T *v) const {
        majorPeak(f, v);
    }
};

// Aliases for double and float instantiations
using arduinoFFT = ArduinoFFT<double>;
using ArduinoFFT_f = ArduinoFFT<float>;
