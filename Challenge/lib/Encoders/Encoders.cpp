// =============================================================================
// Encoders.cpp
// Description : Reads AS5600 (stepper J1 only) and QuadratureEncoder (DC motors),
//               returns joint angles as a MotorAngles struct.
//
//               J2 is NO LONGER read here — its angle comes from StepperMotor
//               step-counting math (see main.cpp).
//
//               AS5600 (J1):
//                 • At startup the raw reading is captured as _offset_j1.
//                 • Every subsequent read returns the signed delta from that
//                   offset, wrapped to [-360, +360].
//
//               Quadrature (J3, J4):
//                 • Already accumulate a signed angle; no extra wrapping needed.
//
// Authors     : Frida Sophia Chavez Juarez
// Mentor      : Oscar Vargas Perez
// Last updated: 2026-04-14
// =============================================================================

#include "Encoders.h"
#include "esp_log.h"
#include <cmath>

static const char* TAG = "Encoders";
constexpr uint8_t ENC_J1_ADDR = 0x36;

// =============================================================================
// Constructor
// =============================================================================
Encoders::Encoders()
    : _offset_j1(0.0f), _zeroed_j1(false)
{}

// =============================================================================
// setup
//   sda2 / scl2 are kept in the signature so existing call-sites (including
//   main_test_encoders.cpp) do not need to change.  J2's I2C bus is simply
//   not initialised anymore.
// =============================================================================
void Encoders::setup(gpio_num_t sda,    gpio_num_t scl,
                     gpio_num_t /*sda2*/, gpio_num_t /*scl2*/,
                     uint8_t gpio_j3[2], float degPerEdge_j3,
                     uint8_t gpio_j4[2], float degPerEdge_j4)
{
    // ── AS5600 for J1 ─────────────────────────────────────────────────────
    _enc_j1 = AS5600_i2c(sda, scl, I2C_NUM_0, ENC_J1_ADDR);

    // Auto-zero: capture the raw reading right now as the 0° reference
    uint16_t raw = 0;
    esp_err_t err = _enc_j1.read_ANGLE(raw);
    if (err == ESP_OK) {
        _offset_j1 = static_cast<float>(raw) * AS5600_DEG_PER_COUNT;
        _zeroed_j1 = true;
        ESP_LOGI(TAG, "J1 AS5600 zeroed — offset=%.2f°", _offset_j1);
    } else {
        _offset_j1 = 0.0f;
        _zeroed_j1 = false;
        ESP_LOGE(TAG, "J1 AS5600 zero-read failed: %s", esp_err_to_name(err));
    }

    // ── QuadratureEncoder for J3 ──────────────────────────────────────────
    _enc_j3.setup(gpio_j3, degPerEdge_j3);
    ESP_LOGI(TAG, "Quadrature J3 — A:%d B:%d deg/edge:%.4f",
             gpio_j3[0], gpio_j3[1], degPerEdge_j3);

    // ── QuadratureEncoder for J4 ──────────────────────────────────────────
    _enc_j4.setup(gpio_j4, degPerEdge_j4);
    ESP_LOGI(TAG, "Quadrature J4 — A:%d B:%d deg/edge:%.4f",
             gpio_j4[0], gpio_j4[1], degPerEdge_j4);
}

// =============================================================================
// readAll
//   j1  — AS5600 delta from startup position, in [-360, +360]
//   j2  — always 0.0f; caller fills it from StepperMotor::currentAngle()
//   j3  — quadrature accumulated angle
//   j4  — quadrature accumulated angle
// =============================================================================
MotorAngles Encoders::readAll()
{
    MotorAngles angles;
    angles.j1 = _readAS5600delta(_enc_j1, _offset_j1);
    angles.j2 = 0.0f;   // J2 supplied by caller from step-math
    angles.j3 = _enc_j3.getAngle();
    angles.j4 = _enc_j4.getAngle();
    return angles;
}

// =============================================================================
// resetZero — re-capture J1 zero reference at the current position
// =============================================================================
void Encoders::resetZero()
{
    uint16_t raw = 0;
    esp_err_t err = _enc_j1.read_ANGLE(raw);
    if (err == ESP_OK) {
        _offset_j1 = static_cast<float>(raw) * AS5600_DEG_PER_COUNT;
        ESP_LOGI(TAG, "J1 zero re-captured at %.2f°", _offset_j1);
    }
}

// =============================================================================
// resetQuadrature
// =============================================================================
void Encoders::resetQuadrature()
{
    _enc_j3.setAngle(0.0f);
    _enc_j4.setAngle(0.0f);
    ESP_LOGI(TAG, "Quadrature encoders reset to 0");
}

// =============================================================================
// _readAS5600delta  (private)
//   Returns the signed angular delta from the stored offset, wrapped to
//   [-360, +360].  Uses the shortest-path convention so a 1° move clockwise
//   from the zero point always returns +1 (not +359 or –359).
// =============================================================================
float Encoders::_readAS5600delta(AS5600_i2c& enc, float offset)
{
    uint16_t raw = 0;
    esp_err_t err = enc.read_ANGLE(raw);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "AS5600 read failed: %s", esp_err_to_name(err));
        return 0.0f;
    }

    float abs_deg = static_cast<float>(raw) * AS5600_DEG_PER_COUNT; // 0..359.91°

    // Compute signed delta and bring into (-360, +360]
    float delta = abs_deg - offset;
    delta = std::fmod(delta, 360.0f);   // result is in (-360, +360)

    // Prefer the shorter arc: map (180, 360) → (−180, 0) and vice-versa
    if (delta >  180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;

    return delta;   // guaranteed in [-180, +180] ⊂ [-360, +360]
}