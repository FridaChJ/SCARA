#pragma once

// =============================================================================
// PIDController.h
// Description : Pure math class. Receives an error value and returns a control
//               signal. No knowledge of motors, hardware, or MQTT.
//               One instance per motor joint.
// Authors     : Frida Sophia Chavez Juarez
// Mentor      : Oscar Vargas Perez
// Last updated: 2026-04-11
// =============================================================================

class PIDController {
public:
    // -------------------------------------------------------------------------
    // Constructor
    //   kp : proportional gain
    //   ki : integral gain
    //   kd : derivative gain
    // -------------------------------------------------------------------------
    PIDController(float kp, float ki, float kd);

    // -------------------------------------------------------------------------
    // compute()
    //   error : target_angle - current_angle  (degrees)
    //   dt    : elapsed time since last call  (seconds)
    //   Returns a signed control signal.
    //     Positive → move in positive angle direction
    //     Negative → move in negative angle direction
    //   Magnitude is used by MotorManager to derive steps or PWM.
    // -------------------------------------------------------------------------
    float compute(float error, float dt);

    // -------------------------------------------------------------------------
    // reset()
    //   Clears integral and previous error.
    //   Call after an e-stop or when switching to a new target abruptly.
    // -------------------------------------------------------------------------
    void reset();

    // -------------------------------------------------------------------------
    // setGains() — allows runtime tuning without recompiling
    // -------------------------------------------------------------------------
    void setGains(float kp, float ki, float kd);

private:
    float _kp;
    float _ki;
    float _kd;

    float _integral;
    float _prevError;

    // Anti-windup: integral is clamped to this magnitude
    static constexpr float INTEGRAL_LIMIT = 1000.0f;
};