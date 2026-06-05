#ifndef GRIPPER_H
#define GRIPPER_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

class Gripper {
public:
    struct Config {
        gpio_num_t       gpio         = GPIO_NUM_4;
        ledc_mode_t      speed_mode   = LEDC_LOW_SPEED_MODE;
        ledc_timer_t     timer        = LEDC_TIMER_3;    // last free (0=DC, 1=J1, 2=J2)
        ledc_channel_t   channel      = LEDC_CHANNEL_6;  // free (0=steppers, 2-5=DC motors)
        ledc_timer_bit_t resolution   = LEDC_TIMER_14_BIT;
        uint32_t         freq_hz      = 50;
        float            min_us       = 500.0f;
        float            max_us       = 2500.0f;
        float            full_deg     = 180.0f;
        float            angle_closed = 0.0f;
        float            angle_open   = 35.0f;
    };

    Gripper();                            // uses default Config
    explicit Gripper(const Config& cfg);  // custom Config
    
    esp_err_t begin();           // configure LEDC, then move to closed
    void  open();                // move to angle_open
    void  close();               // move to angle_closed
    void  setAngle(float deg);   // arbitrary (clamped) angle
    bool  isOpen()       const { return _is_open; }
    float currentAngle() const { return _current_deg; }

private:
    void writeAngle(float deg);

    Config   _cfg;
    uint32_t _period_ticks = 0;
    float    _current_deg  = 0.0f;
    bool     _is_open      = false;
};

#endif // GRIPPER_H