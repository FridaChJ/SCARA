#ifndef __STEPPER_H__
#define __STEPPER_H__

#include "SimplePWM.h"
#include "SimpleGPIO.h"
#include "math.h"


class Stepper
{
public:
    Stepper();
    void setup(uint8_t pins[2], uint8_t pwm_channel, TimerConfig* timer_config, int32_t step_per_rev);
    void setSpeed(float rpm);
    bool directionget();

private:
    SimpleGPIO dir;
    SimplePWM step;
    float _step_per_rev;
};
#endif // __STEPPER_H__