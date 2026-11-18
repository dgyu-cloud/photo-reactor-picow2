#pragma once

#include <stdint.h>

// Integer half-degrees keep knob increments exact.
struct TemperatureUi {
  constexpr TemperatureUi(int initialHalfC, int minHalfC, int maxHalfC)
      : appliedHalfC(initialHalfC), draftHalfC(initialHalfC),
        minHalfC_(minHalfC), maxHalfC_(maxHalfC) {}

  int appliedHalfC;
  int draftHalfC;
  bool editing = false;
  uint32_t blinkStartedMs = 0;

  constexpr void beginEdit(uint32_t now) {
    draftHalfC = appliedHalfC;
    editing = true;
    blinkStartedMs = now;
  }

  constexpr void turn(int steps, uint32_t now) {
    if (steps == 0) return;
    if (!editing) beginEdit(now);
    draftHalfC += steps;
    if (draftHalfC < minHalfC_) draftHalfC = minHalfC_;
    if (draftHalfC > maxHalfC_) draftHalfC = maxHalfC_;
  }

  constexpr bool confirm() {
    if (!editing) return false;
    appliedHalfC = draftHalfC;
    editing = false;
    return true;
  }

  constexpr double targetC() const { return appliedHalfC * 0.5; }
  constexpr bool targetVisible(uint32_t now) const {
    return !editing || ((now - blinkStartedMs) / 400) % 2 == 0;
  }

 private:
  const int minHalfC_;
  const int maxHalfC_;
};

// Decode both contacts; contact bounce retraces and cancels partial steps.
// A full quadrature cycle (four transitions) is one knob increment.
struct RotaryDecoder {
  uint8_t previous = 0;
  int8_t partial = 0;

  constexpr int update(uint8_t state) {
    const int8_t transitions[16] = {
       0, -1,  1,  0,  1,  0,  0, -1,
      -1,  0,  0,  1,  0,  1, -1,  0
    };
    if ((previous ^ state) == 3) partial = 0;
    partial += transitions[(previous << 2) | state];
    previous = state;
    if (partial >= 4) { partial = 0; return 1; }
    if (partial <= -4) { partial = 0; return -1; }
    return 0;
  }
};
