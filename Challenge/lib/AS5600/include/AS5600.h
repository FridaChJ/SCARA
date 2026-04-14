// AS5600.h
#pragma once

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_SPEED_HZ        100000
#define I2C_AS5600_ADDRESS  0x36

typedef enum : uint8_t
{
    AS5600_REG_ZMCO     = 0x00,
    AS5600_REG_ZPOS     = 0x01,
    AS5600_REG_MPOS     = 0x03,
    AS5600_REG_MANG     = 0x05,
    AS5600_REG_CONF     = 0x07,
    AS5600_REG_RAWANGLE = 0x0C,
    AS5600_REG_ANGLE    = 0x0E,
    AS5600_REG_STATUS   = 0x0B,
    AS5600_REG_AGC      = 0x1A,
    AS5600_REG_MAGNITUDE= 0x1B,
    AS5600_REG_BURN     = 0xFF,
} AS5600_REG;

// =============================================================================
// Status
// =============================================================================
struct AS5600_STATUS
{
    AS5600_STATUS(uint8_t status) : _status(status) {}
    bool MagnetStrong()    { return _status & 0b00001000; }
    bool MagnetWeak()      { return _status & 0b00010000; }
    bool MagnetDetected()  { return _status & 0b00100000; }
    operator uint8_t() const { return _status; }
private:
    uint8_t _status;
};

// =============================================================================
// CONF enums
// =============================================================================
typedef enum : uint8_t {
    PM_NOM  = 0b00, PM_LPM1 = 0b01,
    PM_LPM2 = 0b10, PM_LPM3 = 0b11
} AS5600_CONF_PM;

typedef enum : uint8_t {
    HYST_OFF  = 0b00, HYST_1LSB = 0b01,
    HYST_2LSB = 0b10, HYST_3LSB = 0b11
} AS5600_CONF_HYST;

typedef enum : uint8_t {
    OUTS_ANALOG_FULL = 0b00,
    OUTS_ANALOG      = 0b01,
    OUTS_PWM         = 0b10,
} AS5600_CONF_OUTS;

typedef enum : uint8_t {
    PWMF_115 = 0b00, PWMF_230 = 0b01,
    PWMF_460 = 0b10, PWMF_920 = 0b11
} AS5600_CONF_PWMF;

typedef enum : uint8_t {
    SF_16 = 0b00, SF_8 = 0b01,
    SF_4  = 0b10, SF_2 = 0b11
} AS5600_CONF_SF;

typedef enum : uint8_t {
    FTH_0  = 0b000, FTH_6  = 0b001,
    FTH_7  = 0b010, FTH_9  = 0b011,
    FTH_18 = 0b100, FTH_21 = 0b101,
    FTH_24 = 0b110, FTH_10 = 0b111,
} AS5600_CONF_FTH;

const char* CONF_PM_String  (AS5600_CONF_PM   pm);
const char* CONF_HYST_String(AS5600_CONF_HYST hyst);
const char* CONF_OUTS_String(AS5600_CONF_OUTS outs);
const char* CONF_PWMF_String(AS5600_CONF_PWMF pwmf);
const char* CONF_SF_String  (AS5600_CONF_SF   sf);
const char* CONF_FTH_String (AS5600_CONF_FTH  fth);

// =============================================================================
// CONF struct
// =============================================================================
struct AS5600_CONF
{
    AS5600_CONF(uint16_t conf) : _conf(conf) {}

    AS5600_CONF_PM   PowerMode()      { return (AS5600_CONF_PM)  (_conf & 0x03); }
    AS5600_CONF_HYST Hysteresis()     { return (AS5600_CONF_HYST)((_conf >> 2) & 0x03); }
    AS5600_CONF_OUTS OutputStage()    { return (AS5600_CONF_OUTS)((_conf >> 4) & 0x03); }
    AS5600_CONF_PWMF PWMFrequency()   { return (AS5600_CONF_PWMF)((_conf >> 6) & 0x03); }
    AS5600_CONF_SF   SlowFilter()     { return (AS5600_CONF_SF)  ((_conf >> 8) & 0x03); }
    AS5600_CONF_FTH  FastFilterThreshold() { return (AS5600_CONF_FTH)((_conf >> 10) & 0x07); }

    void SetPowerMode(AS5600_CONF_PM pm)
        { _conf = (_conf & ~0x03) | (pm & 0x03); }
    void SetHysteresis(AS5600_CONF_HYST hyst)
        { _conf = (_conf & ~(0x03 << 2)) | ((hyst & 0x03) << 2); }
    void SetOutputStage(AS5600_CONF_OUTS outs)
        { _conf = (_conf & ~(0x03 << 4)) | ((outs & 0x03) << 4); }
    void SetPWMFrequency(AS5600_CONF_PWMF pwmf)
        { _conf = (_conf & ~(0x03 << 6)) | ((pwmf & 0x03) << 6); }
    void SetSlowFilter(AS5600_CONF_SF sf)
        { _conf = (_conf & ~(0x03 << 8)) | ((sf & 0x03) << 8); }
    void SetFastFilterThreshold(AS5600_CONF_FTH fth)
        { _conf = (_conf & ~(0x07 << 10)) | ((fth & 0x07) << 10); }

private:
    uint16_t _conf;
};

// =============================================================================
// AS5600_i2c class
// =============================================================================
class AS5600_i2c
{
public:
    // Full constructor: configures + installs the I2C driver for this port
    AS5600_i2c(gpio_num_t sda, gpio_num_t scl, i2c_port_t i2c_num = I2C_NUM_0)
        : i2c_num(i2c_num)
    {
        // NOTE: i2c_driver_initialize now does param_config AND driver_install
        // in the correct order, using the correct port number
        i2c_driver_initialize(sda, scl);
    }

    // Lightweight constructor: assumes driver already installed externally
    AS5600_i2c(i2c_port_t i2c_num) : i2c_num(i2c_num) {}

    // Default constructor: required for use as a class member before setup
    AS5600_i2c() : i2c_num(I2C_NUM_0) {}

    esp_err_t read_ZMCO    (uint8_t  &zmco);
    esp_err_t read_ZPOS    (uint16_t &zpos);
    esp_err_t write_ZPOS   (uint16_t  zpos);
    esp_err_t read_MPOS    (uint16_t &mpos);
    esp_err_t write_MPOS   (uint16_t  mpos);
    esp_err_t read_MANG    (uint16_t &mang);
    esp_err_t write_MANG   (uint16_t  mang);
    AS5600_CONF read_CONF  (esp_err_t &err);
    AS5600_CONF read_CONF  ();
    esp_err_t write_CONF   (AS5600_CONF conf);
    esp_err_t read_RAWANGLE(uint16_t &angle);
    esp_err_t read_ANGLE   (uint16_t &angle);
    esp_err_t read_STATUS  (AS5600_STATUS &status);
    esp_err_t read_AGC     (uint8_t  &agc);
    esp_err_t read_MAGNITUDE(uint16_t &magnitude);
    esp_err_t Burn_Angle   ();
    esp_err_t Burn_Setting ();

private:
    i2c_port_t i2c_num;

    // FIX: now takes i2c_num into account and installs driver in correct order
    void i2c_driver_initialize(gpio_num_t sda, gpio_num_t scl);

    esp_err_t read_registr (AS5600_REG reg, uint8_t *value, uint8_t len);
    esp_err_t read_registr (AS5600_REG reg, uint16_t &value);
    esp_err_t read_registr (AS5600_REG reg, uint8_t  &value);
    esp_err_t write_registr(AS5600_REG reg, uint8_t *value, uint8_t len);
    esp_err_t write_registr(AS5600_REG reg, uint16_t value);
    esp_err_t write_registr(AS5600_REG reg, uint8_t  value);
};