// =============================================================================
// Encoders.cpp
// Description : Reads one AS5600 magnetic encoder (J1) and two QuadratureEncoders
//               (J3, J4), returning joint angles as a MotorAngles struct.
//
//               J2 is NOT read here — its angle comes from StepperMotor
//               step-counting math (see main.cpp / StepperMotor::currentAngle()).
//
//               AS5600 (J1):
//                 • At startup the raw reading is captured as _offset_j1.
//                 • Every subsequent read returns the signed delta from that
//                   offset, wrapped to [-180, +180] via the shortest-arc convention.
//                 • resetZero() re-captures the offset at the current position.
//
//               Quadrature (J3, J4):
//                 • Already accumulate a signed angle; no extra wrapping needed.
//                 • resetQuadrature() zeroes both counters.
//
// Authors     : Frida Sophia Chavez Juarez
// Mentor      : Oscar Vargas Perez
// Last updated: 2026-04-14
// =============================================================================
#include "Encoders.h"
#include "esp_log.h"
#include <cmath>

static const char* TAG = "Encoders";

// =============================================================================
// Constructor
// =============================================================================
Encoders::Encoders()
    : _offset_j1(0.0f), _zeroed_j1(false)
{}

// =============================================================================
// setup
//   sda2 / scl2 are kept in the signature so existing call-sites do not need
//   to change.  J2's I2C bus is simply not initialised anymore.
// =============================================================================
void Encoders::setup(gpio_num_t sda,      gpio_num_t scl,
                     gpio_num_t /*sda2*/, gpio_num_t /*scl2*/,
                     uint8_t gpio_j3[2],  float degPerEdge_j3,
                     uint8_t gpio_j4[2],  float degPerEdge_j4)
{
    // ── AS5600 for J1 ─────────────────────────────────────────────────────
    // i2c_driver_initialize now correctly stores i2c_num on the object before
    // the first read_ANGLE call below.
    _enc_j1.i2c_driver_initialize(sda, scl, I2C_NUM_0);

    // Capture startup position as the zero reference for J1.
    _captureJ1Zero();

    // ── QuadratureEncoder for J3 ──────────────────────────────────────────
    _enc_j3.setup(gpio_j3, degPerEdge_j3);

    // ── QuadratureEncoder for J4 ──────────────────────────────────────────
    _enc_j4.setup(gpio_j4, degPerEdge_j4);
}

// =============================================================================
// readAll
//   j1  — AS5600 delta from startup position, in [-180, +180]
//   j2  — always 0.0f; caller fills it from StepperMotor::currentAngle()
//   j3  — quadrature accumulated angle
//   j4  — quadrature accumulated angle
// =============================================================================
MotorAngles Encoders::readAll()
{
    MotorAngles angles;
    angles.j1 = _readAS5600delta(_enc_j1, _offset_j1);
    angles.j2 = 0.0f;   // filled by caller from StepperMotor::currentAngle()
    angles.j3 = _enc_j3.getAngle();
    angles.j4 = _enc_j4.getAngle();
    return angles;
}

// =============================================================================
// resetZero
//   BUG FIX: previously this was a no-op stub left over from the dual-encoder
//   refactor, with a wrong comment saying "J1 AS5600 is disabled".  J1 IS the
//   AS5600.  Now it correctly re-captures the zero reference.
// =============================================================================
void Encoders::resetZero()
{
    _captureJ1Zero();
}

// =============================================================================
// resetQuadrature
// =============================================================================
void Encoders::resetQuadrature()
{
    _enc_j3.setAngle(0.0f);
    _enc_j4.setAngle(0.0f);
    ESP_LOGI(TAG, "Quadrature encoders (J3, J4) reset to 0°");
}

// =============================================================================
// _captureJ1Zero  (private)
//   Reads the current AS5600 angle and stores it as _offset_j1.
//   Sets _zeroed_j1 = true on success, false on I2C failure.
// =============================================================================
void Encoders::_captureJ1Zero()
{
    uint16_t raw = 0;
    esp_err_t err = _enc_j1.read_ANGLE(raw);
    if (err == ESP_OK)
    {
        _offset_j1 = static_cast<float>(raw) * AS5600_DEG_PER_COUNT;
        _zeroed_j1 = true;
        ESP_LOGI(TAG, "AS5600 J1 — zero captured at %.2f°", _offset_j1);
    }
    else
    {
        _zeroed_j1 = false;
        ESP_LOGE(TAG, "AS5600 J1 — failed to read angle (err 0x%X). "
                      "Check SDA/SCL wiring and I2C address (0x36).", err);
    }
}

// =============================================================================
// _readAS5600delta  (private)
//   Returns the signed angular delta from the stored offset, wrapped to
//   [-180, +180] using the shortest-arc convention.
//   Returns 0.0f if the sensor has not been zeroed or an I2C read fails.
// =============================================================================
float Encoders::_readAS5600delta(AS5600_i2c& enc, float offset)
{
    if (!_zeroed_j1)
    {
        ESP_LOGW(TAG, "AS5600 J1 not zeroed — returning 0°");
        return 0.0f;
    }

    uint16_t raw = 0;
    esp_err_t err = enc.read_ANGLE(raw);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "AS5600 J1 read failed (err 0x%X)", err);
        return 0.0f;
    }

    float abs_deg = static_cast<float>(raw) * AS5600_DEG_PER_COUNT;  // 0 … 359.91°

    // Signed delta relative to zero reference, shortest arc.
    float delta = abs_deg - offset;
    delta = std::fmod(delta, 360.0f);        // collapse to (-360, +360)
    if (delta >  180.0f) delta -= 360.0f;    // map (180, 360] → (-180, 0]
    if (delta < -180.0f) delta += 360.0f;    // map [-360,-180) → [0, 180)

    return delta;   // guaranteed ∈ [-180, +180]
}