#pragma once
#include <cstdint>
#include <cstddef>
#include <deque>
#include <vector>
#include "Arduino.h"

// =============================================================================
// Mock TwoWire (I2C) Implementation for Host Native Testing
// =============================================================================
class TwoWire {
private:
    std::deque<uint8_t> rx_buffer;
    std::vector<uint8_t> tx_buffer;
    uint8_t transmitting_addr = 0;
    bool is_begun = false;
    uint32_t clock_frequency = 100000;
    uint32_t timeout_ms = 50;
    bool device_present = true;
    uint8_t last_status = 0;

public:
    TwoWire() = default;

    bool begin(int sda = -1, int scl = -1) {
        (void)sda;
        (void)scl;
        is_begun = true;
        return true;
    }

    bool begin(uint8_t address) {
        transmitting_addr = address;
        is_begun = true;
        return true;
    }

    void end() {
        is_begun = false;
    }

    void setClock(uint32_t freq) {
        clock_frequency = freq;
    }

    uint32_t getClock() const {
        return clock_frequency;
    }

    void setTimeOut(uint32_t ms) {
        timeout_ms = ms;
    }

    uint32_t getTimeOut() const {
        return timeout_ms;
    }

    void beginTransmission(uint8_t address) {
        transmitting_addr = address;
        tx_buffer.clear();
    }

    void beginTransmission(int address) {
        beginTransmission(static_cast<uint8_t>(address));
    }

    uint8_t endTransmission(bool sendStop = true) {
        (void)sendStop;
        if (!device_present) {
            last_status = 2; // NACK on address transmit
            return 2;
        }
        last_status = 0; // Success
        return 0;
    }

    size_t write(uint8_t data) {
        tx_buffer.push_back(data);
        return 1;
    }

    size_t write(int data) {
        return write(static_cast<uint8_t>(data));
    }

    size_t write(const uint8_t *data, size_t quantity) {
        if (!data) return 0;
        for (size_t i = 0; i < quantity; ++i) {
            tx_buffer.push_back(data[i]);
        }
        return quantity;
    }

    size_t write(const char *str) {
        if (!str) return 0;
        return write(reinterpret_cast<const uint8_t*>(str), std::strlen(str));
    }

    uint8_t requestFrom(uint8_t address, size_t quantity, bool sendStop = true) {
        (void)address;
        (void)sendStop;
        if (!device_present) return 0;
        // If buffer has fewer bytes than requested, fill with zeroes
        while (rx_buffer.size() < quantity) {
            rx_buffer.push_back(0);
        }
        return static_cast<uint8_t>(quantity);
    }

    uint8_t requestFrom(uint8_t address, uint8_t quantity) {
        return requestFrom(address, static_cast<size_t>(quantity), true);
    }

    uint8_t requestFrom(int address, int quantity) {
        return requestFrom(static_cast<uint8_t>(address), static_cast<size_t>(quantity), true);
    }

    uint8_t requestFrom(int address, int quantity, int sendStop) {
        return requestFrom(static_cast<uint8_t>(address), static_cast<size_t>(quantity), sendStop != 0);
    }

    int available() {
        return static_cast<int>(rx_buffer.size());
    }

    int read() {
        if (rx_buffer.empty()) return -1;
        uint8_t b = rx_buffer.front();
        rx_buffer.pop_front();
        return static_cast<int>(b);
    }

    int peek() {
        if (rx_buffer.empty()) return -1;
        return static_cast<int>(rx_buffer.front());
    }

    void flush() {}

    // Mock-specific inspection and fault injection helpers
    void mock_set_device_present(bool present) {
        device_present = present;
    }

    bool mock_is_device_present() const {
        return device_present;
    }

    void mock_inject_rx(const std::vector<uint8_t> &data) {
        for (uint8_t b : data) {
            rx_buffer.push_back(b);
        }
    }

    void mock_inject_rx_byte(uint8_t b) {
        rx_buffer.push_back(b);
    }

    const std::vector<uint8_t>& mock_get_tx() const {
        return tx_buffer;
    }

    void mock_clear_tx() {
        tx_buffer.clear();
    }

    void mock_clear_rx() {
        rx_buffer.clear();
    }

    void reset() {
        rx_buffer.clear();
        tx_buffer.clear();
        transmitting_addr = 0;
        is_begun = false;
        clock_frequency = 100000;
        timeout_ms = 50;
        device_present = true;
        last_status = 0;
    }
};

inline TwoWire Wire;
