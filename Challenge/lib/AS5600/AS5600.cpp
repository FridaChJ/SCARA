// AS5600.cpp
#include "AS5600.h"

static const char* TAG = "AS5600";
static bool i2c_initialized[2] = {false, false};

// =============================================================================
// i2c_driver_initialize
// =============================================================================
void AS5600_i2c::i2c_driver_initialize(gpio_num_t sda, gpio_num_t scl, i2c_port_t port)
{
    // BUG FIX: store the port on the object so that subsequent read/write calls
    // use the correct I2C peripheral.  Previously the parameter name shadowed
    // the member, leaving this->i2c_num unchanged.
    this->i2c_num = port;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
    if (i2c_initialized[port])
    {
        ESP_LOGW(TAG, "I2C port %d already initialized, skipping", port);
        return;
    }
    i2c_config_t i2c_config = {
        .mode          = I2C_MODE_MASTER,
        .sda_io_num    = sda,
        .scl_io_num    = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master        = { .clk_speed = I2C_SPEED_HZ }
    };
#pragma GCC diagnostic pop

    ESP_ERROR_CHECK(i2c_param_config(port, &i2c_config));

    esp_err_t err = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        ESP_ERROR_CHECK(err);

    i2c_initialized[port] = true;
    ESP_LOGI(TAG, "I2C port %d initialised — SDA:%d SCL:%d addr:0x%02X @ %d Hz",
             port, sda, scl, i2c_address, I2C_SPEED_HZ);
}

// =============================================================================
// Register read / write primitives
// =============================================================================
esp_err_t AS5600_i2c::read_registr(AS5600_REG reg, uint8_t* value, uint8_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (uint8_t)reg, true);
    i2c_master_start(cmd);   // repeated start
    i2c_master_write_byte(cmd, (i2c_address << 1) | I2C_MASTER_READ, true);
    if (len > 1)
        i2c_master_read(cmd, value, len, I2C_MASTER_LAST_NACK);
    else
        i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t AS5600_i2c::read_registr(AS5600_REG reg, uint16_t& value)
{
    uint8_t data[2] = {0, 0};
    esp_err_t err = read_registr(reg, data, 2);
    if (err == ESP_OK)
        value = ((uint16_t)data[0] << 8) | data[1];   // AS5600 sends MSB first
    return err;
}

esp_err_t AS5600_i2c::read_registr(AS5600_REG reg, uint8_t& value)
{
    return read_registr(reg, &value, 1);
}

esp_err_t AS5600_i2c::write_registr(AS5600_REG reg, uint8_t* value, uint8_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (uint8_t)reg, true);
    i2c_master_write(cmd, value, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t AS5600_i2c::write_registr(AS5600_REG reg, uint16_t value)
{
    // BUG FIX: AS5600 expects big-endian (MSB first).
    // Casting a uint16_t pointer directly would send the low byte first on
    // little-endian ESP32, corrupting every 16-bit write (ZPOS, MPOS, MANG, CONF).
    uint8_t buf[2] = { (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return write_registr(reg, buf, 2);
}

esp_err_t AS5600_i2c::write_registr(AS5600_REG reg, uint8_t value)
{
    return write_registr(reg, &value, 1);
}

// =============================================================================
// Public API
// =============================================================================
esp_err_t AS5600_i2c::read_ZMCO     (uint8_t&  zmco)      { return read_registr(AS5600_REG_ZMCO,      zmco); }
esp_err_t AS5600_i2c::read_ZPOS     (uint16_t& zpos)      { return read_registr(AS5600_REG_ZPOS,      zpos); }
esp_err_t AS5600_i2c::write_ZPOS    (uint16_t  zpos)      { return write_registr(AS5600_REG_ZPOS,     zpos); }
esp_err_t AS5600_i2c::read_MPOS     (uint16_t& mpos)      { return read_registr(AS5600_REG_MPOS,      mpos); }
esp_err_t AS5600_i2c::write_MPOS    (uint16_t  mpos)      { return write_registr(AS5600_REG_MPOS,     mpos); }
esp_err_t AS5600_i2c::read_MANG     (uint16_t& mang)      { return read_registr(AS5600_REG_MANG,      mang); }
esp_err_t AS5600_i2c::write_MANG    (uint16_t  mang)      { return write_registr(AS5600_REG_MANG,     mang); }
esp_err_t AS5600_i2c::read_RAWANGLE (uint16_t& angle)     { return read_registr(AS5600_REG_RAWANGLE,  angle); }
esp_err_t AS5600_i2c::read_ANGLE    (uint16_t& angle)     { return read_registr(AS5600_REG_ANGLE,     angle); }
esp_err_t AS5600_i2c::read_AGC      (uint8_t&  agc)       { return read_registr(AS5600_REG_AGC,       agc); }
esp_err_t AS5600_i2c::read_MAGNITUDE(uint16_t& magnitude) { return read_registr(AS5600_REG_MAGNITUDE, magnitude); }
esp_err_t AS5600_i2c::Burn_Angle    ()                    { return write_registr(AS5600_REG_BURN,     (uint8_t)0x80); }
esp_err_t AS5600_i2c::Burn_Setting  ()                    { return write_registr(AS5600_REG_BURN,     (uint8_t)0x40); }

esp_err_t AS5600_i2c::read_STATUS(AS5600_STATUS& status)
    { return read_registr(AS5600_REG_STATUS, *(uint8_t*)&status); }

esp_err_t AS5600_i2c::write_CONF(AS5600_CONF conf)
{
    // Use the corrected big-endian write path (via uint16_t overload)
    return write_registr(AS5600_REG_CONF, *(uint16_t*)&conf);
}

AS5600_CONF AS5600_i2c::read_CONF(esp_err_t& err)
{
    uint16_t ret = 0;
    err = read_registr(AS5600_REG_CONF, ret);
    return AS5600_CONF(ret);
}

AS5600_CONF AS5600_i2c::read_CONF()
{
    esp_err_t err;
    return read_CONF(err);
}