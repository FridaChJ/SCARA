#pragma once
// =============================================================================
// DriverStepper.h
// Authors     : Frida Sophia Chavez Juarez
// Mentor      : Oscar Vargas Perez
// Last updated: 2026-04-13
// =============================================================================

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_attr.h"

static constexpr int FULL_STEPS_PER_REV = 200;

class StepperMotor {
public:
    // jointLabel: short string shown in every log line, e.g. "J1" or "J2"
    StepperMotor(int stepPin, int dirPin,
                 int enPin       = -1,
                 int limitPin    = -1,
                 const char* jointLabel = "??");
    ~StepperMotor();

    void begin();

    void setMicrostepDivisor(uint16_t divisor);
    void setMaxSpeed(float degPerSec);
    void setAcceleration(float degPerSec2);
    void setMinPulseWidth(uint16_t us);
    void setDirInverted(bool inverted);
    void setEnableActiveLevel(uint8_t level);
    void setLimitActiveLevel(uint8_t level);

    void enable();
    void disable();
    bool isEnabled() const;

    void moveTo(float degrees);
    void moveBy(float degrees);
    void home(int8_t direction = -1, float backoffDeg = 5.0f);
    void emergencyStop();
    void softStop();

    bool  isMoving()        const;
    bool  isHoming()        const;
    bool  limitPressed()    const;
    float currentAngle()    const;
    long  currentPosition() const;
    float currentSpeed()    const;

    bool onTimer();
    void update();

private:
    int _stepPin, _dirPin, _enPin, _limitPin;
    char _label[8];

    gptimer_handle_t _timer;

    uint16_t _microstepDiv;
    int      _stepsPerRev;
    float    _maxSpeedSPS;
    float    _accelSPS2;
    uint16_t _minPulseUs;
    bool     _dirInverted;
    uint8_t  _enActiveLevel;
    uint8_t  _limitActiveLevel;

    volatile long  _currentPos;
    volatile long  _targetPos;
    volatile float _currentSpeedSPS;
    volatile bool  _moving;
    bool           _enabled;
    volatile long  _stepsToGo;
    long           _totalSteps;
    long           _accelSteps;
    long           _decelStart;
    volatile float _stepInterval;
    float          _minInterval;
    float          _accelSPS2_cache;

    volatile bool     _stopRequested;
    volatile bool     _timerRunning;
    volatile uint32_t _nextIntervalUs;
    volatile bool     _needsIntervalUpdate;

    uint32_t _minIntervalUs;
    uint32_t _accelSPS2_integ;

    enum class HomingState { IDLE, SEEKING, BACKING_OFF };
    HomingState _homingState;
    int8_t      _homingDir;
    float       _homingBackoff;

    void  _planMove(long steps);
    void  _setDirection(bool forward);
    void  _pulse();
    float _stepsToAngle(long steps) const;
    long  _angleToSteps(float deg)  const;
    void  _startTimer(float intervalUs);
    void  _stopTimer();

    static bool IRAM_ATTR _timerCallback(gptimer_handle_t timer,
                                          const gptimer_alarm_event_data_t* edata,
                                          void* user_ctx);
};