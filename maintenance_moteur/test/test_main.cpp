#include "test_framework.h"
#include "mocks/Arduino.h"
#include "mocks/Wire.h"
#include "mocks/Adafruit_Sensor.h"
#include "mocks/Adafruit_MPU6050.h"
#include "mocks/DHT.h"
#include "mocks/arduinoFFT.h"
#include <cmath>
#include <vector>

// =============================================================================
// Mock Layer Self-Test Suite
// =============================================================================

// -----------------------------------------------------------------------------
// 1. MockClock & Virtual Time Tests
// -----------------------------------------------------------------------------
TEST(MockLayerTest, VirtualClock_TimeProgression) {
    MockClock::reset();
    ASSERT_EQ(0u, millis());
    ASSERT_EQ(0u, micros());

    MockClock::advance_us(5000); // 5 ms = 5000 us
    ASSERT_EQ(5000u, micros());
    ASSERT_EQ(5u, millis());

    MockClock::advance_ms(100);
    ASSERT_EQ(105000u, micros());
    ASSERT_EQ(105u, millis());

    delay(50);
    ASSERT_EQ(155u, millis());
    ASSERT_EQ(155000u, micros());

    delayMicroseconds(500);
    ASSERT_EQ(155500u, micros());

    MockClock::reset();
    ASSERT_EQ(0u, micros());
    ASSERT_EQ(0u, millis());
}

// -----------------------------------------------------------------------------
// 2. MockGPIO & LEDC Tests
// -----------------------------------------------------------------------------
TEST(MockLayerTest, MockGPIO_DigitalAndAnalog) {
    MockGPIO::reset();

    pinMode(13, OUTPUT);
    ASSERT_EQ(OUTPUT, MockGPIO::pin_mode[13]);

    pinMode(12, INPUT_PULLUP);
    ASSERT_EQ(INPUT_PULLUP, MockGPIO::pin_mode[12]);

    digitalWrite(13, HIGH);
    ASSERT_EQ(HIGH, digitalRead(13));
    ASSERT_EQ(HIGH, MockGPIO::pin_state[13]);

    digitalWrite(13, LOW);
    ASSERT_EQ(LOW, digitalRead(13));
    ASSERT_EQ(LOW, MockGPIO::pin_state[13]);

    analogWrite(25, 128);
    ASSERT_EQ(128u, MockGPIO::pin_pwm[25]);

    // LEDC ESP32 PWM emulation
    ledcSetup(0, 5000, 8);
    ledcAttachPin(18, 0);
    ledcWrite(0, 200);
    ASSERT_EQ(200u, MockGPIO::pin_pwm[18]);

    MockGPIO::reset();
    ASSERT_EQ(0u, MockGPIO::pin_pwm[18]);
    ASSERT_EQ(LOW, MockGPIO::pin_state[13]);
}

// -----------------------------------------------------------------------------
// 3. MockSerial RX Injection & TX Capture Tests
// -----------------------------------------------------------------------------
TEST(MockLayerTest, MockSerial_InputOutput) {
    Serial.reset();
    ASSERT_EQ(0, Serial.available());

    // TX Capture
    Serial.print("RMS: ");
    Serial.print(1.2345, 2);
    Serial.print(" | State: ");
    Serial.println("NORMAL");

    std::string output = Serial.get_output();
    ASSERT_EQ(std::string("RMS: 1.23 | State: NORMAL\n"), output);

    // TX Base Formatting
    Serial.clear_output();
    Serial.print(255, HEX);
    ASSERT_EQ(std::string("FF"), Serial.get_output());
    Serial.clear_output();
    Serial.print(10, BIN);
    ASSERT_EQ(std::string("1010"), Serial.get_output());

    // RX Injection
    Serial.inject_input("INJECT_VIB\nRESET\n");
    ASSERT_TRUE(Serial.available() > 0);

    std::string cmd1 = Serial.readStringUntil('\n');
    ASSERT_EQ(std::string("INJECT_VIB"), cmd1);

    std::string cmd2 = Serial.readStringUntil('\n');
    ASSERT_EQ(std::string("RESET"), cmd2);

    ASSERT_EQ(0, Serial.available());
    ASSERT_EQ(-1, Serial.read());
}

// -----------------------------------------------------------------------------
// 4. TwoWire Mock Tests
// -----------------------------------------------------------------------------
TEST(MockLayerTest, TwoWire_TransmissionAndReceive) {
    Wire.reset();
    ASSERT_TRUE(Wire.begin(21, 22));

    Wire.beginTransmission(0x68);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x00); // Wake up
    ASSERT_EQ(0, Wire.endTransmission());

    const auto& tx = Wire.mock_get_tx();
    ASSERT_EQ(2u, tx.size());
    ASSERT_EQ(0x6B, tx[0]);
    ASSERT_EQ(0x00, tx[1]);

    // Request from device
    std::vector<uint8_t> rx_data = {0x12, 0x34, 0x56};
    Wire.mock_inject_rx(rx_data);
    ASSERT_EQ(3, Wire.requestFrom(0x68, 3));
    ASSERT_EQ(3, Wire.available());
    ASSERT_EQ(0x12, Wire.read());
    ASSERT_EQ(0x34, Wire.read());
    ASSERT_EQ(0x56, Wire.read());
    ASSERT_EQ(0, Wire.available());

    // Device absent NACK
    Wire.mock_set_device_present(false);
    Wire.beginTransmission(0x68);
    ASSERT_EQ(2, Wire.endTransmission());
}

// -----------------------------------------------------------------------------
// 5. Adafruit_MPU6050 Mock & Synthetic Generator Tests
// -----------------------------------------------------------------------------
TEST(MockLayerTest, MPU6050_ConstantMode) {
    mock_mpu6050.reset();
    ASSERT_TRUE(mock_mpu6050.begin(0x68));

    mock_mpu6050.set_constant(9.81f, 0.1f, -0.2f);
    sensors_event_t accel, gyro, temp;
    ASSERT_TRUE(mock_mpu6050.getEvent(&accel, &gyro, &temp));

    ASSERT_NEAR(9.81f, accel.acceleration.z, 0.001f);
    ASSERT_NEAR(0.1f, accel.acceleration.x, 0.001f);
    ASSERT_NEAR(-0.2f, accel.acceleration.y, 0.001f);
    ASSERT_NEAR(25.0f, temp.temperature, 0.001f);
}

TEST(MockLayerTest, MPU6050_SineWaveMode) {
    mock_mpu6050.reset();
    MockClock::reset();

    // 25 Hz sine, amplitude 2.0, DC offset 9.81, zero noise for precision check
    mock_mpu6050.set_sine(25.0f, 2.0f, 9.81f, 0.0f);

    // At t = 0 ms -> z = 9.81
    sensors_event_t accel;
    mock_mpu6050.getEvent(&accel);
    ASSERT_NEAR(9.81f, accel.acceleration.z, 0.01f);

    // At t = 10 ms (1/4 cycle of 25 Hz) -> sin(pi/2) = 1.0 -> z = 9.81 + 2.0 = 11.81
    MockClock::set_micros(10000);
    mock_mpu6050.getEvent(&accel);
    ASSERT_NEAR(11.81f, accel.acceleration.z, 0.01f);

    // At t = 20 ms (1/2 cycle) -> sin(pi) = 0 -> z = 9.81
    MockClock::set_micros(20000);
    mock_mpu6050.getEvent(&accel);
    ASSERT_NEAR(9.81f, accel.acceleration.z, 0.01f);

    // At t = 30 ms (3/4 cycle) -> sin(3pi/2) = -1.0 -> z = 9.81 - 2.0 = 7.81
    MockClock::set_micros(30000);
    mock_mpu6050.getEvent(&accel);
    ASSERT_NEAR(7.81f, accel.acceleration.z, 0.01f);
}

TEST(MockLayerTest, MPU6050_PlaybackAndFaultSpike) {
    mock_mpu6050.reset();
    std::vector<float> custom_samples = {1.0f, 2.0f, 3.0f, 4.0f};
    mock_mpu6050.set_playback(custom_samples);

    sensors_event_t accel;
    for (float expected : custom_samples) {
        ASSERT_TRUE(mock_mpu6050.getEvent(&accel));
        ASSERT_NEAR(expected, accel.acceleration.z, 0.001f);
    }
    // Loop back
    ASSERT_TRUE(mock_mpu6050.getEvent(&accel));
    ASSERT_NEAR(1.0f, accel.acceleration.z, 0.001f);

    // Fault spike
    mock_mpu6050.set_fault_spike(50.0f, 2);
    mock_mpu6050.getEvent(&accel);
    ASSERT_NEAR(50.0f, accel.acceleration.z, 0.001f);
    mock_mpu6050.getEvent(&accel);
    ASSERT_NEAR(50.0f, accel.acceleration.z, 0.001f);
}

// -----------------------------------------------------------------------------
// 6. Mock DHT11 Tests
// -----------------------------------------------------------------------------
TEST(MockLayerTest, DHT11_ReadAndFaultInjection) {
    mock_dht.reset();
    mock_dht.begin();

    ASSERT_NEAR(28.5f, mock_dht.readTemperature(), 0.01f);
    ASSERT_NEAR(45.0f, mock_dht.readHumidity(), 0.01f);

    mock_dht.set_temperature(75.2f);
    mock_dht.set_humidity(80.0f);
    ASSERT_NEAR(75.2f, mock_dht.readTemperature(), 0.01f);
    ASSERT_NEAR(80.0f, mock_dht.readHumidity(), 0.01f);

    // Fahrenheit conversion
    float fahrenheit = mock_dht.readTemperature(true);
    ASSERT_NEAR(75.2f * 1.8f + 32.0f, fahrenheit, 0.01f);

    // NaN fault injection
    mock_dht.set_nan_failure(true);
    ASSERT_TRUE(std::isnan(mock_dht.readTemperature()));
    ASSERT_TRUE(std::isnan(mock_dht.readHumidity()));
    ASSERT_FALSE(mock_dht.read());

    mock_dht.set_nan_failure(false);
    ASSERT_FALSE(std::isnan(mock_dht.readTemperature()));
}

// -----------------------------------------------------------------------------
// 7. Native Cooley-Tukey Radix-2 FFT Math Tests
// -----------------------------------------------------------------------------
TEST(MockLayerTest, FFT_PureSineWave25Hz_DominantPeak) {
    const uint16_t SAMPLES = 64;
    const double SAMPLING_FREQUENCY = 200.0; // 200 Hz
    double vReal[SAMPLES];
    double vImag[SAMPLES];

    // Generate 25 Hz sine wave: f = 25 Hz -> Bin index = 25 / (200 / 64) = 8
    for (uint16_t i = 0; i < SAMPLES; ++i) {
        double t = static_cast<double>(i) / SAMPLING_FREQUENCY;
        vReal[i] = 2.5 * std::sin(2.0 * M_PI * 25.0 * t);
        vImag[i] = 0.0;
    }

    arduinoFFT FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    double dominant_freq = FFT.majorPeak();
    // Delta f per bin = 200 / 64 = 3.125 Hz. Peak should resolve to 25.0 Hz (bin 8)
    ASSERT_NEAR(25.0, dominant_freq, 1.5);
}

TEST(MockLayerTest, FFT_PureSineWave50Hz_DominantPeak) {
    const uint16_t SAMPLES = 64;
    const double SAMPLING_FREQUENCY = 200.0;
    double vReal[SAMPLES];
    double vImag[SAMPLES];

    // Generate 50 Hz sine wave -> Bin index = 50 / (200 / 64) = 16
    for (uint16_t i = 0; i < SAMPLES; ++i) {
        double t = static_cast<double>(i) / SAMPLING_FREQUENCY;
        vReal[i] = 3.0 * std::sin(2.0 * M_PI * 50.0 * t);
        vImag[i] = 0.0;
    }

    arduinoFFT FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    double dominant_freq = FFT.majorPeak();
    ASSERT_NEAR(50.0, dominant_freq, 1.5);
}

TEST(MockLayerTest, FFT_InverseReconstruction) {
    const uint16_t SAMPLES = 32;
    double original[SAMPLES];
    double vReal[SAMPLES];
    double vImag[SAMPLES];

    for (uint16_t i = 0; i < SAMPLES; ++i) {
        original[i] = std::cos(2.0 * M_PI * i / 8.0) + 0.5 * std::sin(2.0 * M_PI * i / 4.0);
        vReal[i] = original[i];
        vImag[i] = 0.0;
    }

    arduinoFFT FFT(vReal, vImag, SAMPLES, 100.0);
    // Forward FFT
    FFT.compute(FFTDirection::Forward);
    // Inverse FFT
    FFT.compute(FFTDirection::Reverse);

    // Verify reconstruction matches original values within floating point epsilon
    for (uint16_t i = 0; i < SAMPLES; ++i) {
        ASSERT_NEAR(original[i], vReal[i], 1e-4);
        ASSERT_NEAR(0.0, vImag[i], 1e-4);
    }
}

TEST(MockLayerTest, FFT_FloatSpecialization) {
    const uint16_t SAMPLES = 64;
    const float SAMPLING_FREQ = 200.0f;
    float vReal[SAMPLES];
    float vImag[SAMPLES];

    for (uint16_t i = 0; i < SAMPLES; ++i) {
        float t = static_cast<float>(i) / SAMPLING_FREQ;
        vReal[i] = 1.8f * std::sin(2.0f * 3.14159265f * 25.0f * t);
        vImag[i] = 0.0f;
    }

    ArduinoFFT<float> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQ);
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    float dominant_freq = FFT.majorPeak();
    ASSERT_NEAR(25.0f, dominant_freq, 1.5f);
}

// =============================================================================
// Main Entry Point
// =============================================================================
int main(int argc, char** argv) {
    return RUN_ALL_TESTS(argc, argv);
}
