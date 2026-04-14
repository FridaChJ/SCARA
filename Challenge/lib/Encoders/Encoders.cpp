// =============================================================================
// Encoders.cpp
// Description : Reads AS5600 (steppers) and QuadratureEncoder (DC motors),
//               returns all four joint angles as a MotorAngles struct.
// Authors     : Frida Sophia Chavez Juarez
// Mentor      : Oscar Vargas Perez
// Last updated: 2026-04-11
// =============================================================================

#include "Encoders.h"
#include "esp_log.h"

static const char* TAG = "Encoders";

// =============================================================================
// Constructor
// =============================================================================
Encoders::Encoders() {}

// =============================================================================
// setup
// =============================================================================
void Encoders::setup(gpio_num_t sda_j1, gpio_num_t scl_j1,
                     gpio_num_t sda_j2, gpio_num_t scl_j2,
                     uint8_t gpio_j3[2], float degPerEdge_j3,
                     uint8_t gpio_j4[2], float degPerEdge_j4)
{
    // ── AS5600 for stepper 1 (j1) ─────────────────────────────────────────
    _enc_j1 = AS5600_i2c(sda_j1, scl_j1, I2C_NUM_0);
    _enc_j2 = AS5600_i2c(sda_j2, scl_j2, I2C_NUM_1);

    // ── QuadratureEncoder for DC motor 1 (j3) ─────────────────────────────
    _enc_j3.setup(gpio_j3, degPerEdge_j3);
    ESP_LOGI(TAG, "Quadrature j3 initialized — A:%d B:%d deg/edge:%.4f",
             gpio_j3[0], gpio_j3[1], degPerEdge_j3);

    // ── QuadratureEncoder for DC motor 2 (j4) ─────────────────────────────
    _enc_j4.setup(gpio_j4, degPerEdge_j4);
    ESP_LOGI(TAG, "Quadrature j4 initialized — A:%d B:%d deg/edge:%.4f",
             gpio_j4[0], gpio_j4[1], degPerEdge_j4);
}

// =============================================================================
// readAll
// =============================================================================
MotorAngles Encoders::readAll()
{
    MotorAngles angles;
    angles.j1 = _readAS5600(_enc_j1);
    angles.j2 = _readAS5600(_enc_j2);
    angles.j3 = _enc_j3.getAngle();
    angles.j4 = _enc_j4.getAngle();
    return angles;
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
// _readAS5600 — private helper
// =============================================================================
float Encoders::_readAS5600(AS5600_i2c& enc)
{
    uint16_t raw = 0;
    esp_err_t err = enc.read_ANGLE(raw);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "AS5600 read failed: %d", err);
        return 0.0f;
    }
    return static_cast<float>(raw) * AS5600_DEG_PER_COUNT;
}