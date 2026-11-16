#pragma once

#include <Arduino.h>
#include <Wire.h>

// SEN0322 protocol reference:
// https://github.com/DFRobot/DFRobot_OxygenSensor
// Split the documented 50/100 ms register waits across loop() calls.
// No calibration writes: use the sensor's stored calibration coefficient.
class OxygenSensor {
 public:
  double percent = NAN;
  const char* status = "starting";
  uint8_t address = 0;

  // True when a new measurement or error should be reported.
  bool update(uint32_t now) {
    if (now - phaseStarted < waitMs) return false;
    if (phase == 0) {
      // Probe using an actual register write; empty probes are unreliable
      // with the Arduino Mbed Wire implementation.
      if (!address) {
        for (uint8_t candidate : {0x73, 0x72, 0x71, 0x70}) {
          if (selectRegister(candidate, 0x0A)) {
            address = candidate;
            break;
          }
        }
        if (!address) return fail(now, "not connected");
      } else if (!selectRegister(address, 0x0A)) {
        return fail(now, "I2C write failed");
      }
      phase = 1;
      phaseStarted = now;
      waitMs = 50;
    } else if (phase == 1) {
      uint8_t bytes[2];
      if (!readBytes(bytes, 2)) return fail(now, "I2C short read");
      const uint16_t rawKey = bytes[0] | (uint16_t(bytes[1]) << 8);
      coefficient = rawKey ? rawKey / 1000.0 : 20.9 / 120.0;
      if (!selectRegister(address, 0x03)) return fail(now, "I2C write failed");
      phase = 2;
      phaseStarted = now;
      waitMs = 100;
    } else {
      uint8_t bytes[3];
      if (!readBytes(bytes, 3)) return fail(now, "I2C short read");
      const double value = coefficient *
          (bytes[0] + bytes[1] / 10.0 + bytes[2] / 100.0);
      if (!isfinite(value) || value < 0 || value > 25.0) {
        return fail(now, "out of range");
      }
      percent = value;
      status = "ok";
      phase = 0;
      // 150 ms register waits + 850 ms idle = approximately 1 s per sample.
      phaseStarted = now;
      waitMs = 850;
      return true;
    }
    return false;
  }

 private:
  uint8_t phase = 0;
  uint32_t phaseStarted = 0;
  uint32_t waitMs = 0;
  double coefficient = 0;

  bool selectRegister(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    return Wire.endTransmission() == 0;
  }

  bool readBytes(uint8_t* bytes, uint8_t count) {
    if (Wire.requestFrom(address, count) != count) {
      while (Wire.available()) Wire.read();
      return false;
    }
    for (uint8_t i = 0; i < count; ++i) bytes[i] = Wire.read();
    return true;
  }

  bool fail(uint32_t now, const char* reason) {
    percent = NAN;
    status = reason;
    address = 0;
    phase = 0;
    phaseStarted = now;
    waitMs = 1000;
    return true;
  }
};
