#ifndef __QUADRATUREENCODER_H__
#define __QUADRATUREENCODER_H__

#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_attr.h>

class QuadratureEncoder
{
public:
    QuadratureEncoder();
    ~QuadratureEncoder();
    void setup(uint8_t gpio_num[], const float degrees_per_edge, const int64_t timeout_us = 100000);
    float getAngle();
    float getSpeed();
    int8_t getDirection();
    void setAngle(float angle);

private:
    void IRAM_ATTR handler();
    gpio_num_t _gpio_num[2];
    float _degrees_per_edge;
    volatile float _speed;
    volatile int64_t _counts, _timeout_us;
    volatile int64_t _prev_micros, _delta_micros;
    volatile uint8_t _states;
    volatile int8_t _direction;
    const int8_t _edge_lut[16] = {0, 0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, 0};
};

#endif // __QUADRATUREENCODER_H__