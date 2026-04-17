// =============================================================================
// Encoders.h
// Authors     : Frida Sophia Chavez Juarez
// Mentor      : Oscar Vargas Perez
// Last updated: 2026-04-14
// =============================================================================
#pragma once

#include "AS5600.h"
#include "QuadratureEncoder.h"
#include "MotorAngles.h"
#include "driver/gpio.h"

// Degrees per raw count from AS5600 (12-bit → 4096 counts per revolution)
//constexpr float AS5600_DEG_PER_COUNT = 360.0f / 4096.0f;

class Encoders
{
public:
    Encoders();

    // Initialise all encoders.
    // sda2/scl2 are accepted but unused (J2 now uses step-math).
    void setup(gpio_num_t sda,   gpio_num_t scl,
               gpio_num_t sda2, gpio_num_t scl2,
               uint8_t gpio_j3[2], float degPerEdge_j3,
               uint8_t gpio_j4[2], float degPerEdge_j4);

    // Returns all four joint angles.
    // j2 is always 0.0f — caller must fill it from StepperMotor::currentAngle().
    MotorAngles readAll();

    // Re-capture the J1 zero reference at the current physical position.
    void resetZero();

    // Reset quadrature counters for J3 and J4 to zero.
    void resetQuadrature();

private:
    // ── AS5600 (J1 only) ──────────────────────────────────────────────────
    AS5600_i2c _enc_j1;
    float      _offset_j1;   // raw angle captured at startup (= 0° reference)
    bool       _zeroed_j1;

    // ── Quadrature encoders (J3, J4) ──────────────────────────────────────
    QuadratureEncoder _enc_j3;
    QuadratureEncoder _enc_j4;

    // Helper: read raw angle, subtract offset, wrap to [-180, +180]
    float _readAS5600delta(AS5600_i2c& enc, float offset);
};