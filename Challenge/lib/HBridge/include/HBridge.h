#pragma once
// =============================================================================
// HBridge.h
// Authors     : Frida Sophia Chavez Juarez
// Mentor      : Oscar Vargas Perez
// Last updated: 2026-04-13
//
// Change: setup() now takes an optional jointLabel ("J3", "J4") so log lines
// identify which DC motor is acting. _label stored as a small char array.
// =============================================================================

#include <stdint.h>
#include "SimplePWM.h"

class HBridge {
public:
    HBridge();

    // jointLabel: short string shown in every log line, e.g. "J3" or "J4"
    void setup(uint8_t pin[2], uint8_t ch[2], TimerConfig* config,
               const char* jointLabel = "??");

    void setDuty(float duty);   // negative = forward, positive = reverse
    void setStop();

private:
    SimplePWM pwm[2];
    char      _label[8];
    bool      _stopped;
};