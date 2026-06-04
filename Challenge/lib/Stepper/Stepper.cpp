#include "include/Stepper.h"

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

// Stepper.cpp — fix order
void Stepper::setSpeed(float rpm)
{
    if (rpm > 0)
        dir.set(1);
    else
        dir.set(0);

    printf("Direction set to %s\n", dir.get() ? "TRUE" : "FALSE");

    int32_t freq = fabs(rpm) * (float)_step_per_rev / 60.0f;

    if (freq < 4)
    {
        step.setDuty(0);
    }
    else
    {
        step.setFrequency(freq);  // ← FIRST
        step.setDuty(0.5f);         // ← SECOND
    }
}
bool Stepper::directionget()
{
    return dir.get();
}

