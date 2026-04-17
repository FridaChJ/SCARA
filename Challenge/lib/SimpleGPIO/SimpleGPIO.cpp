#include "SimpleGPIO.h"

SimpleGPIO::SimpleGPIO()
{
}

SimpleGPIO::~SimpleGPIO()
{
    gpio_reset_pin(_gpio_num);
}

void SimpleGPIO::setup(uint8_t gpio_num, gpio_mode_t mode, gpio_pull_mode_t pull)
{
    if(mode!=GPIO_MODE_DISABLE)
    {
        _gpio_num = (gpio_num_t)gpio_num;
        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = mode;
        io_conf.pin_bit_mask = (1ULL << gpio_num);
        gpio_config(&io_conf);
        gpio_set_pull_mode(_gpio_num, pull);
    }
}

void SimpleGPIO::set(uint32_t state)
{
    gpio_set_level(_gpio_num, state);
}

int SimpleGPIO::get()
{
    return gpio_get_level(_gpio_num);
}

void SimpleGPIO::addInterrupt(gpio_int_type_t interrupt_type, gpio_isr_t handler, int64_t debounce_time_us)
{
    _debounce_time_us = debounce_time_us;
    gpio_set_intr_type(_gpio_num, interrupt_type);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(_gpio_num, handler, (void *)_gpio_num);
    _prev_state = get();
}

void SimpleGPIO::setInterruptFlag()
{
    volatile int64_t current_time = esp_timer_get_time();
    _elapsed_time_us = current_time - _prev_time;
    if (_elapsed_time_us >= _debounce_time_us)
    {
        _prev_time = current_time;
        _interrupt_flag = 1;
        _prev_state = _state;
        _state = get();
    }
}

bool SimpleGPIO::interruptAvailable()
{
    if (_interrupt_flag)
    {
        _interrupt_flag = 0;
        return true;
    }
    return false;
}

int64_t SimpleGPIO::getElapsedTime()
{
    return _elapsed_time_us;
}

int SimpleGPIO::getEdge()
{
    if (_prev_state > _state)
        return 0; // Falling edge
    else if (_prev_state < _state)
        return 1; // Rising edge
    else
        return 2; // No change
}