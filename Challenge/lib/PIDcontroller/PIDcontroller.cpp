// =============================================================================
// PIDController.cpp
// Description : Pure math PID implementation. No hardware dependencies.
// Authors     : Frida Sophia Chavez Juarez
// Mentor      : Oscar Vargas Perez
// Last updated: 2026-04-11
// =============================================================================
 
#include "PIDcontroller.h"
#include <cmath>
 
// =============================================================================
// Constructor
// =============================================================================
PIDController::PIDController(float kp, float ki, float kd)
    : _kp(kp), _ki(ki), _kd(kd),
      _integral(0.0f), _prevError(0.0f)
{}
 
// =============================================================================
// compute
// =============================================================================
float PIDController::compute(float error, float dt)
{
    if (dt <= 0.0f) return 0.0f;   // guard: avoid division by zero
 
    // Proportional term
    float p = _kp * error;
 
    // Integral term with anti-windup clamp
    _integral += error * dt;
    if      (_integral >  INTEGRAL_LIMIT) _integral =  INTEGRAL_LIMIT;
    else if (_integral < -INTEGRAL_LIMIT) _integral = -INTEGRAL_LIMIT;
    float i = _ki * _integral;
 
    // Derivative term (computed on error, not on measurement,
    // to avoid derivative kick when the target changes suddenly)
    float derivative = (error - _prevError) / dt;
    float d = _kd * derivative;
 
    _prevError = error;
 
    return p + i + d;
}
 
// =============================================================================
// reset
// =============================================================================
void PIDController::reset()
{
    _integral  = 0.0f;
    _prevError = 0.0f;
}
 
// =============================================================================
// setGains
// =============================================================================
void PIDController::setGains(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}