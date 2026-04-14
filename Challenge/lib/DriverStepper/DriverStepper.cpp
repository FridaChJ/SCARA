// =============================================================================
// DriverStepper.cpp
// Authors     : Frida Sophia Chavez Juarez
// Mentor      : Oscar Vargas Perez
// Last updated: 2026-04-13
//
// Change v5→v6: joint label ("J1"/"J2") added to constructor and stored so
// every log line identifies which stepper is acting.
// All previous fixes (v1–v5) retained.
// =============================================================================

#include "DriverStepper.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include <cmath>
#include <cstdlib>
#include <cstring>

static const char* TAG = "Stepper";

// =============================================================================
// isqrt32 — pure integer square root, IRAM-safe, zero FPU
// =============================================================================
static IRAM_ATTR uint32_t isqrt32(uint32_t n)
{
    if (n == 0) return 0;
    uint32_t x = n;
    uint32_t y = (x + 1) >> 1;
    while (y < x) { x = y; y = (x + n / x) >> 1; }
    return x;
}

// =============================================================================
// Constructor
// =============================================================================
StepperMotor::StepperMotor(int stepPin, int dirPin, int enPin, int limitPin,
                           const char* jointLabel)
    : _stepPin(stepPin), _dirPin(dirPin), _enPin(enPin), _limitPin(limitPin)
    , _timer(nullptr)
    , _microstepDiv(1), _stepsPerRev(FULL_STEPS_PER_REV)
    , _maxSpeedSPS(0.0f), _accelSPS2(0.0f)
    , _minPulseUs(5), _dirInverted(false)
    , _enActiveLevel(0), _limitActiveLevel(0)
    , _currentPos(0), _targetPos(0)
    , _currentSpeedSPS(0.0f), _moving(false), _enabled(false)
    , _stepsToGo(0), _totalSteps(0)
    , _accelSteps(0), _decelStart(0)
    , _stepInterval(0.0f), _minInterval(0.0f)
    , _accelSPS2_cache(0.0f)
    , _stopRequested(false), _timerRunning(false)
    , _nextIntervalUs(0), _needsIntervalUpdate(false)
    , _minIntervalUs(0), _accelSPS2_integ(0)
    , _homingState(HomingState::IDLE), _homingDir(-1), _homingBackoff(5.0f)
{
    strncpy(_label, jointLabel ? jointLabel : "??", sizeof(_label) - 1);
    _label[sizeof(_label) - 1] = '\0';

    setMicrostepDivisor(1);
    setMaxSpeed(360.0f);
    setAcceleration(720.0f);
}

StepperMotor::~StepperMotor()
{
    _stopTimer();
    if (_timer) {
        gptimer_disable(_timer);
        gptimer_del_timer(_timer);
    }
}

// =============================================================================
// begin
// =============================================================================
void StepperMotor::begin()
{
    gpio_config_t io = {};
    io.mode         = GPIO_MODE_OUTPUT;
    io.pull_up_en   = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_DISABLE;
    io.pin_bit_mask = (1ULL << _stepPin) | (1ULL << _dirPin);
    if (_enPin >= 0) io.pin_bit_mask |= (1ULL << _enPin);
    gpio_config(&io);

    gpio_set_level((gpio_num_t)_stepPin, 0);
    gpio_set_level((gpio_num_t)_dirPin,  0);
    if (_enPin >= 0) disable();

    if (_limitPin >= 0) {
        gpio_config_t lim = {};
        lim.mode         = GPIO_MODE_INPUT;
        lim.pull_up_en   = GPIO_PULLUP_ENABLE;
        lim.pull_down_en = GPIO_PULLDOWN_DISABLE;
        lim.intr_type    = GPIO_INTR_DISABLE;
        lim.pin_bit_mask = (1ULL << _limitPin);
        gpio_config(&lim);
    }

    gptimer_config_t cfg = {};
    cfg.clk_src       = GPTIMER_CLK_SRC_DEFAULT;
    cfg.direction     = GPTIMER_COUNT_UP;
    cfg.resolution_hz = 1000000;
    gptimer_new_timer(&cfg, &_timer);

    gptimer_event_callbacks_t cbs = {};
    cbs.on_alarm = _timerCallback;
    gptimer_register_event_callbacks(_timer, &cbs, this);
    gptimer_enable(_timer);

    ESP_LOGI(TAG, "[%s] begin — STEP:%d  DIR:%d  EN:%d",
             _label, _stepPin, _dirPin, _enPin);
}

// =============================================================================
// Configuration
// =============================================================================
void StepperMotor::setMicrostepDivisor(uint16_t divisor)
{
    _microstepDiv = (divisor < 1) ? 1 : divisor;
    _stepsPerRev  = FULL_STEPS_PER_REV * _microstepDiv;
}

void StepperMotor::setMaxSpeed(float degPerSec)
{
    if (degPerSec <= 0.0f) return;
    _maxSpeedSPS   = (degPerSec / 360.0f) * _stepsPerRev;
    _minInterval   = 1e6f / _maxSpeedSPS;
    _minIntervalUs = (uint32_t)_minInterval;
}

void StepperMotor::setAcceleration(float degPerSec2)
{
    if (degPerSec2 <= 0.0f) return;
    _accelSPS2       = (degPerSec2 / 360.0f) * _stepsPerRev;
    _accelSPS2_cache = _accelSPS2;
    _accelSPS2_integ = (uint32_t)_accelSPS2;
}

void StepperMotor::setMinPulseWidth(uint16_t us)       { _minPulseUs       = us;       }
void StepperMotor::setDirInverted(bool inverted)       { _dirInverted      = inverted; }
void StepperMotor::setEnableActiveLevel(uint8_t level) { _enActiveLevel    = level;    }
void StepperMotor::setLimitActiveLevel(uint8_t level)  { _limitActiveLevel = level;    }

// =============================================================================
// Enable / disable
// =============================================================================
void StepperMotor::enable()
{
    if (_enPin >= 0)
        gpio_set_level((gpio_num_t)_enPin, _enActiveLevel);
    _enabled = true;
    ESP_LOGI(TAG, "[%s] ENABLE — EN pin:%d = %d", _label, _enPin, _enActiveLevel);
}

void StepperMotor::disable()
{
    if (_enPin >= 0)
        gpio_set_level((gpio_num_t)_enPin, _enActiveLevel == 0 ? 1 : 0);
    _enabled = false;
    ESP_LOGI(TAG, "[%s] DISABLE — EN pin:%d = %d",
             _label, _enPin, _enActiveLevel == 0 ? 1 : 0);
}

bool StepperMotor::isEnabled() const { return _enabled; }

// =============================================================================
// Motion commands
// =============================================================================
void StepperMotor::moveTo(float degrees)
{
    long target = _angleToSteps(degrees);
    long delta  = target - _currentPos;
    if (delta == 0) return;
    _targetPos = target;
    _setDirection(delta > 0);
    _planMove(std::abs(delta));
}

void StepperMotor::moveBy(float degrees)
{
    long delta = _angleToSteps(degrees);
    if (delta == 0) return;
    _targetPos = _currentPos + delta;
    _setDirection(delta > 0);
    _planMove(std::abs(delta));
}

void StepperMotor::home(int8_t direction, float backoffDeg)
{
    if (_limitPin < 0) return;
    _homingDir     = (direction >= 0) ? 1 : -1;
    _homingBackoff = backoffDeg;
    _homingState   = HomingState::SEEKING;
    moveBy(_homingDir * 3600.0f);
}

void StepperMotor::emergencyStop()
{
    _stopTimer();
    _moving          = false;
    _currentSpeedSPS = 0.0f;
    _stepsToGo       = 0;
    _targetPos       = _currentPos;
    _homingState     = HomingState::IDLE;
    ESP_LOGI(TAG, "[%s] EMERGENCY STOP", _label);
}

void StepperMotor::softStop()
{
    if (!_moving) return;
    float speedAbs   = std::fabs(_currentSpeedSPS);
    long  decelSteps = (long)((speedAbs * speedAbs) / (2.0f * _accelSPS2));
    if (decelSteps < 1) decelSteps = 1;
    bool forward = (_targetPos > _currentPos);
    _targetPos  = _currentPos + (forward ? decelSteps : -decelSteps);
    _stepsToGo  = decelSteps;
    _totalSteps = _accelSteps + decelSteps;
    _decelStart = _accelSteps;
}

// =============================================================================
// Status
// =============================================================================
bool  StepperMotor::isMoving()        const { return _moving; }
bool  StepperMotor::isHoming()        const { return _homingState != HomingState::IDLE; }
float StepperMotor::currentAngle()    const { return _stepsToAngle(_currentPos); }
long  StepperMotor::currentPosition() const { return _currentPos; }
float StepperMotor::currentSpeed()    const {
    return (_currentSpeedSPS / (float)_stepsPerRev) * 360.0f;
}
bool StepperMotor::limitPressed() const
{
    if (_limitPin < 0) return false;
    return gpio_get_level((gpio_num_t)_limitPin) == _limitActiveLevel;
}

// =============================================================================
// update — task context only
// =============================================================================
void StepperMotor::update()
{
    switch (_homingState) {
    case HomingState::IDLE: break;
    case HomingState::SEEKING:
        if (limitPressed()) {
            emergencyStop();
            _homingState = HomingState::BACKING_OFF;
            moveBy(-_homingDir * _homingBackoff);
        }
        break;
    case HomingState::BACKING_OFF:
        if (!_moving) {
            _currentPos  = 0;
            _targetPos   = 0;
            _homingState = HomingState::IDLE;
        }
        break;
    }

    if (_needsIntervalUpdate && _moving) {
        uint32_t interval    = _nextIntervalUs;
        _needsIntervalUpdate = false;
        _moving = false;
        _stopTimer();
        _moving = true;
        _startTimer((float)interval);
    }
}

// =============================================================================
// onTimer — ISR context, fully FPU-free
// =============================================================================
bool IRAM_ATTR StepperMotor::onTimer()
{
    if (_stopRequested || !_moving || _stepsToGo == 0) {
        _moving       = false;
        _timerRunning = false;
        return false;
    }

    if (_homingState == HomingState::SEEKING && _limitPin >= 0) {
        if (gpio_get_level((gpio_num_t)_limitPin) == _limitActiveLevel) {
            _moving       = false;
            _timerRunning = false;
            return false;
        }
    }

    _pulse();

    if ((_targetPos - _currentPos) > 0) _currentPos = _currentPos + 1;
    else                                 _currentPos = _currentPos - 1;
    _stepsToGo = _stepsToGo - 1;

    long stepsDone = _totalSteps - _stepsToGo;
    uint32_t newIntervalUs;

    if (stepsDone <= _accelSteps) {
        uint32_t arg   = (uint32_t)((uint64_t)2 * _accelSPS2_integ * (uint32_t)stepsDone);
        uint32_t speed = isqrt32(arg);
        if (speed == 0) speed = 1;
        newIntervalUs = 1000000UL / speed;
    } else if (stepsDone >= _decelStart) {
        uint32_t stepsLeft = (uint32_t)(_stepsToGo + 1);
        uint32_t arg       = (uint32_t)((uint64_t)2 * _accelSPS2_integ * stepsLeft);
        uint32_t speed     = isqrt32(arg);
        if (speed == 0) speed = 1;
        newIntervalUs = 1000000UL / speed;
    } else {
        newIntervalUs = _minIntervalUs;
    }

    uint32_t minUs = (uint32_t)(_minPulseUs * 2);
    if (newIntervalUs < minUs)          newIntervalUs = minUs;
    if (newIntervalUs < _minIntervalUs) newIntervalUs = _minIntervalUs;

    _nextIntervalUs      = newIntervalUs;
    _needsIntervalUpdate = true;
    return true;
}

// =============================================================================
// _planMove — task context only
// =============================================================================
void StepperMotor::_planMove(long steps)
{
    if (steps == 0) return;

    _stopTimer();
    _moving = false;

    _totalSteps      = steps;
    long accelNeeded = (long)((_maxSpeedSPS * _maxSpeedSPS) / (2.0f * _accelSPS2));

    if (2 * accelNeeded >= steps) {
        _accelSteps = steps / 2;
        _decelStart = steps - _accelSteps;
    } else {
        _accelSteps = accelNeeded;
        _decelStart = steps - accelNeeded;
    }

    _stepsToGo           = steps;
    _currentSpeedSPS     = 0.0f;
    _needsIntervalUpdate = false;

    float firstInterval = 0.676f * std::sqrt(2.0f / _accelSPS2) * 1e6f;
    if (firstInterval < (float)(_minPulseUs * 2))
        firstInterval = (float)(_minPulseUs * 2);
    _stepInterval = firstInterval;

    int dirLevel = (int)gpio_get_level((gpio_num_t)_dirPin);
    ESP_LOGI(TAG, "[%s] moveTo — STEP:%d  DIR:%d=%d  steps:%ld  target:%.1f°",
             _label, _stepPin, _dirPin, dirLevel, steps,
             _stepsToAngle(_targetPos));

    _moving = true;
    _startTimer(firstInterval);
}

// =============================================================================
// Internal helpers
// =============================================================================
void StepperMotor::_setDirection(bool forward)
{
    bool level = forward ^ _dirInverted;
    gpio_set_level((gpio_num_t)_dirPin, level ? 1 : 0);
}

void StepperMotor::_pulse()
{
    gpio_set_level((gpio_num_t)_stepPin, 1);
    ets_delay_us(_minPulseUs);
    gpio_set_level((gpio_num_t)_stepPin, 0);
}

float StepperMotor::_stepsToAngle(long steps) const {
    return (float)steps * (360.0f / (float)_stepsPerRev);
}

long StepperMotor::_angleToSteps(float deg) const {
    return (long)std::roundf(deg * (float)_stepsPerRev / 360.0f);
}

void StepperMotor::_startTimer(float intervalUs)
{
    _stopRequested = false;
    _timerRunning  = true;

    gptimer_alarm_config_t alarm = {};
    alarm.alarm_count                = (uint64_t)intervalUs;
    alarm.reload_count               = 0;
    alarm.flags.auto_reload_on_alarm = true;
    gptimer_set_alarm_action(_timer, &alarm);
    gptimer_start(_timer);
}

void StepperMotor::_stopTimer()
{
    if (!_timer) return;
    _stopRequested = true;
    _timerRunning  = false;
    gptimer_stop(_timer);
    _stopRequested = false;
}

bool IRAM_ATTR StepperMotor::_timerCallback(gptimer_handle_t timer,
                                              const gptimer_alarm_event_data_t* edata,
                                              void* user_ctx)
{
    StepperMotor* self = static_cast<StepperMotor*>(user_ctx);
    return self->onTimer();
}