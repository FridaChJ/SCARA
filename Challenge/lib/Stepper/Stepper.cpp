#include "Stepper.h"

Stepper::Stepper()
{
}

void Stepper::setup(uint8_t pins[2], uint8_t pwm_channel, TimerConfig* timer_config, int32_t step_per_rev)
{
    step.setup(pins[0], pwm_channel, timer_config);
    dir.setup(pins[1], GPIO_MODE_OUTPUT);
    _step_per_rev = step_per_rev;
    step.setDuty(0);
}

void Stepper::setSpeed(float rpm)
{
    if (rpm > 0)
    {
        dir.set(1);
        printf("Direction set to %s\n", dir.get() ? "TRUE" : "FALSE");
    }
    else
    {
        dir.set(0);
        printf("Direction set to %s\n", dir.get() ? "TRUE" : "FALSE");
    }
    int32_t freq = fabs(rpm) * (float)_step_per_rev/60.0f;
    if (freq < 4)
        step.setDuty(0);
    else
    {
        step.setDuty(10);
        step.setFrequency(freq);
    }
}

bool Stepper::directionget()
{
    return dir.get();
}

