#ifndef _SIMPLEGPIO_H
#define _SIMPLEGPIO_H

#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_attr.h>

#define GPI GPIO_MODE_INPUT
#define GPO GPIO_MODE_OUTPUT
#define GPIO GPIO_MODE_INPUT_OUTPUT

class SimpleGPIO
{
public:
    SimpleGPIO();
    ~SimpleGPIO();
    void setup(uint8_t gpio_num, gpio_mode_t mode, gpio_pull_mode_t pull = GPIO_FLOATING);
    void set(uint32_t state);
    int get();
    void addInterrupt(gpio_int_type_t intr_type, gpio_isr_t handler, int64_t debounce_time_us = 0);
    void setInterruptFlag();
    bool interruptAvailable();
    int64_t getElapsedTime();
    int getEdge();

private:
    gpio_num_t _gpio_num;
    uint32_t _debounce_time_us;
    volatile int64_t _prev_time, _elapsed_time_us;
    volatile int _interrupt_flag, _state, _prev_state;
};

#endif // _SIMPLEGPIO_H