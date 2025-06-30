#ifndef TCA9548A_H
#define TCA9548A_H

#include <Arduino.h>
#include <Wire.h>

class TCA9548A {
public:
    TCA9548A(uint8_t address = 0x70) : _address(address) {}

    void begin() {
        Wire.begin();
    }

    bool selectChannel(uint8_t channel) {
        if (channel > 7) return false;
        Wire.beginTransmission(_address);
        Wire.write(1 << channel);
        return Wire.endTransmission() == 0;
    }

    void disableAll() {
        Wire.beginTransmission(_address);
        Wire.write(0x00);  // Disable all channels
        Wire.endTransmission();
    }

private:
    uint8_t _address;
};

#endif
