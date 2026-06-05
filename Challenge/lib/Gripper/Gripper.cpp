#include "Gripper.h"
#include "esp_log.h"

static const char* TAG = "Gripper";

Gripper::Gripper()                  : _cfg{}    {}
Gripper::Gripper(const Config& cfg) : _cfg(cfg) {}

esp_err_t Gripper::begin()
{
    _period_ticks = (1u << _cfg.resolution);

    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode      = _cfg.speed_mode;
    timer_conf.duty_resolution = _cfg.resolution;
    timer_conf.timer_num       = _cfg.timer;
    timer_conf.freq_hz         = _cfg.freq_hz;
    timer_conf.clk_cfg         = LEDC_AUTO_CLK;
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) return err;

    ledc_channel_config_t ch_conf = {};
    ch_conf.gpio_num   = _cfg.gpio;
    ch_conf.speed_mode = _cfg.speed_mode;
    ch_conf.channel    = _cfg.channel;
    ch_conf.intr_type  = LEDC_INTR_DISABLE;
    ch_conf.timer_sel  = _cfg.timer;
    ch_conf.duty       = 0;
    ch_conf.hpoint     = 0;
    err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK) return err;

    close();  // safe known state on boot
    ESP_LOGI(TAG, "ready on GPIO%d (timer %d, ch %d)",
             _cfg.gpio, _cfg.timer, _cfg.channel);
    return ESP_OK;
}

void Gripper::writeAngle(float deg)
{
    if (deg < 0.0f)            deg = 0.0f;
    if (deg > _cfg.full_deg)   deg = _cfg.full_deg;

    float us = _cfg.min_us + (deg / _cfg.full_deg) * (_cfg.max_us - _cfg.min_us);
    float period_us = 1000000.0f / _cfg.freq_hz;          // 20000 us @ 50 Hz
    uint32_t duty = (uint32_t)((us / period_us) * _period_ticks);

    ledc_set_duty(_cfg.speed_mode, _cfg.channel, duty);
    ledc_update_duty(_cfg.speed_mode, _cfg.channel);

    _current_deg = deg;
}

void Gripper::setAngle(float deg) { writeAngle(deg); }
void Gripper::open()  { writeAngle(_cfg.angle_open);   _is_open = true;  }
void Gripper::close() { writeAngle(_cfg.angle_closed); _is_open = false; }