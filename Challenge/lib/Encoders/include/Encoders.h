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

    // ── Public accessors for diagnostics ───────────────────────────────────
    AS5600_i2c _enc_j1;       // AS5600 encoder object
    float      _offset_j1;    // Captured zero reference

private:
    // ── Quadrature encoders (J3, J4) ──────────────────────────────────────
    QuadratureEncoder _enc_j3;
    QuadratureEncoder _enc_j4;
    bool       _zeroed_j1;   // false until a successful read_ANGLE at startup

    // Capture the current AS5600 reading as the J1 zero reference.
    void  _captureJ1Zero();

    // Read raw angle, subtract offset, wrap to [-180, +180].
    float _readAS5600delta(AS5600_i2c& enc, float offset);
};