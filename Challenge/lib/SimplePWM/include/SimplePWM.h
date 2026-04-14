#ifndef _SIMPLEPWM_H
#define _SIMPLEPWM_H


// Replace with:
#include "driver/ledc.h"        // try first

struct TimerConfig
{
    const ledc_timer_t timer = LEDC_TIMER_0;
    uint32_t frequency = 25000;
    const ledc_timer_bit_t bit_resolution = LEDC_TIMER_10_BIT;
    const ledc_mode_t mode = LEDC_LOW_SPEED_MODE;
};

class SimplePWM
{
public:
    SimplePWM();
    void setup(const uint8_t pin, const uint8_t channel, TimerConfig *timer_config, bool invert = false);
    void setDigitalLevel(uint32_t digital_level, uint32_t hpoint = 0);
    void setDuty(float duty);
    void setFrequency(uint32_t frequency);
    static void init_ledc_once();   // call once before any setup()
    gpio_num_t gpio_num;

private:
    ledc_channel_t _channel;
    TimerConfig *_timer_config;
    uint32_t _max_digital_level, _last_level;
};

#endif // _SIMPLEPWM_H