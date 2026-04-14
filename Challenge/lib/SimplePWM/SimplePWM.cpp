// =============================================================================
// SimplePWM.cpp
// Authors     : Frida Sophia Chavez Juarez
// Mentor      : Oscar Vargas Perez
// Last updated: 2026-04-13
//
// FIX: Removed ledc_fade_func_install() entirely.
//      That call installs an ISR on core 1 which conflicts with the TG1
//      interrupt watchdog and causes TG1WDT_SYS_RST every boot.
//      Fade is not needed — replaced ledc_set_duty_and_update() with
//      ledc_set_duty() + ledc_update_duty() which require no ISR at all.
// =============================================================================

#include "SimplePWM.h"

SimplePWM::SimplePWM() {}

void SimplePWM::setup(const uint8_t pin, const uint8_t channel,
                      TimerConfig* timer_config, bool invert)
{
    gpio_num           = (gpio_num_t)pin;
    _channel           = (ledc_channel_t)channel;
    _timer_config      = timer_config;
    _max_digital_level = (1 << timer_config->bit_resolution) - 1;

    // Configure the hardware timer only once per unique (timer, mode) pair
    static uint32_t configured_timers = 0;
    uint32_t timer_bit = (uint32_t)(timer_config->timer) * 2
                       + (uint32_t)(timer_config->mode);

    if (!(configured_timers & (1u << timer_bit))) {
        ledc_timer_config_t ledc_timer = {
            .speed_mode      = timer_config->mode,
            .duty_resolution = timer_config->bit_resolution,
            .timer_num       = timer_config->timer,
            .freq_hz         = timer_config->frequency,
            .clk_cfg         = LEDC_AUTO_CLK,
            .deconfigure     = false,
        };
        esp_err_t err = ledc_timer_config(&ledc_timer);
        if (err != ESP_OK)
            printf("Timer config failed! Timer=%d Mode=%d err=%d\n",
                   timer_config->timer, timer_config->mode, err);
        else
            configured_timers |= (1u << timer_bit);
    }

    // Configure the channel — no fade ISR needed
    ledc_channel_config_t ledc_channel = {
        .gpio_num   = gpio_num,
        .speed_mode = timer_config->mode,
        .channel    = _channel,
        .intr_type  = LEDC_INTR_DISABLE,   // no interrupt
        .timer_sel  = timer_config->timer,
        .duty       = 0,
        .hpoint     = 0,
        .flags      = { .output_invert = invert },
    };
    _last_level = 0;

    esp_err_t err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK)
        printf("Channel config failed! GPIO=%d ch=%d err=%d\n", pin, channel, err);

    printf("PWM Setup: GPIO=%d, Channel=%d, Timer=%d, Mode=%d\n",
           pin, channel, timer_config->timer, timer_config->mode);
}

void SimplePWM::setDigitalLevel(uint32_t digital_level, uint32_t hpoint)
{
    if (digital_level != _last_level) {
        if (digital_level > _max_digital_level)
            digital_level = _max_digital_level;

        // No fade ISR needed — set duty directly and update
        ledc_set_duty(_timer_config->mode, _channel, digital_level);
        ledc_update_duty(_timer_config->mode, _channel);

        _last_level = digital_level;
    }
}

void SimplePWM::setDuty(float duty_percentage)
{
    uint32_t digital_level =
        (uint32_t)(duty_percentage * _max_digital_level / 100.0f);
    setDigitalLevel(digital_level, 0);
}

void SimplePWM::setFrequency(uint32_t frequency)
{
    esp_err_t err = ledc_set_freq(
        _timer_config->mode, _timer_config->timer, frequency);
    if (err != ESP_OK)
        printf("setFreq failed: ch=%d mode=%d timer=%d err=%d\n",
               _channel, _timer_config->mode, _timer_config->timer, err);
}