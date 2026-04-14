#pragma once

// =============================================================================
// Encoders.h
// Description : Unified encoder class for the robot arm.
//               Wraps two AS5600 magnetic encoders (for steppers j1/j2)
//               and two QuadratureEncoders (for DC motors j3/j4).
//               Single responsibility: read all four joints and return
//               their angles in degrees as a MotorAngles struct.
//               Knows nothing about motors, PID, or MQTT.
// Authors     : Frida Sophia Chavez Juarez
// Mentor      : Oscar Vargas Perez
// Last updated: 2026-04-11
// Dependencies: AS5600, QuadratureEncoder, motor_angles.h
// =============================================================================

#include "MotorAngles.h"
#include "AS5600.h"
#include "QuadratureEncoder.h"

// -----------------------------------------------------------------------------
// AS5600 raw range: 0–4095 counts for 0–360 degrees
// -----------------------------------------------------------------------------
static constexpr float AS5600_COUNTS_PER_REV = 4096.0f;
static constexpr float AS5600_DEG_PER_COUNT  = 360.0f / AS5600_COUNTS_PER_REV;

class Encoders {
public:
    Encoders();

    // -------------------------------------------------------------------------
    // setup()
    //   Call once before readAll().
    //
    //   sda_j1, scl_j1 : I2C pins for AS5600 on joint 1 (stepper 1)
    //   sda_j2, scl_j2 : I2C pins for AS5600 on joint 2 (stepper 2)
    //
    //   gpio_j3[2]     : A/B pins for quadrature encoder on joint 3 (DC 1)
    //   gpio_j4[2]     : A/B pins for quadrature encoder on joint 4 (DC 2)
    //
    //   degPerEdge_j3  : degrees per encoder edge for DC motor 1
    //   degPerEdge_j4  : degrees per encoder edge for DC motor 2
    // -------------------------------------------------------------------------
    void setup(gpio_num_t sda_j1, gpio_num_t scl_j1,
               gpio_num_t sda_j2, gpio_num_t scl_j2,
               uint8_t gpio_j3[2], float degPerEdge_j3,
               uint8_t gpio_j4[2], float degPerEdge_j4);

    // -------------------------------------------------------------------------
    // readAll()
    //   Reads all four encoders and returns angles in degrees.
    //   j1, j2 : from AS5600  (absolute magnetic, 0–360 deg)
    //   j3, j4 : from quadrature encoders (relative, unbounded)
    // -------------------------------------------------------------------------
    MotorAngles readAll();

    // -------------------------------------------------------------------------
    // resetQuadrature()
    //   Sets the quadrature encoder counts to zero (re-home DC motors).
    // -------------------------------------------------------------------------
    void resetQuadrature();

private:
    AS5600_i2c _enc_j1{I2C_NUM_0};
    AS5600_i2c _enc_j2{I2C_NUM_1};
    QuadratureEncoder  _enc_j3;   // quadrature encoder — DC motor 1
    QuadratureEncoder  _enc_j4;   // quadrature encoder — DC motor 2

    // Read one AS5600 and return angle in degrees (0–360)
    float _readAS5600(AS5600_i2c& enc);
};