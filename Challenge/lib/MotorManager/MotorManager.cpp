// =============================================================================
// MotorManager.cpp
// Authors     : Frida Sophia Chavez Juarez
// Mentor      : Oscar Vargas Perez
// Last updated: 2026-04-13
//
// Change: update() now returns bool — true when ALL four joints are within
// DEADBAND_DEG of their targets. main.cpp uses this to clear TARGET_READY_BIT
// so the PID loop (and the setStop spam) stops once the robot is at position.
// =============================================================================

#include "MotorManager.h"
#include <cmath>

MotorManager::MotorManager(
    StepperMotor&  stepper1,
    StepperMotor&  stepper2,
    HBridge&       dc1,
    HBridge&       dc2,
    PIDController& pid_j1,
    PIDController& pid_j2,
    PIDController& pid_j3,
    PIDController& pid_j4,
    float          maxDuty
)
    : _stepper1(stepper1), _stepper2(stepper2)
    , _dc1(dc1),           _dc2(dc2)
    , _pid_j1(pid_j1),     _pid_j2(pid_j2)
    , _pid_j3(pid_j3),     _pid_j4(pid_j4)
    , _maxDuty(maxDuty)
{}

// Returns true when every joint is within DEADBAND_DEG — main.cpp clears
// TARGET_READY_BIT on that condition to stop the PID loop.
bool MotorManager::update(const MotorAngles& target,
                          const MotorAngles& feedback,
                          float dt)
{
    bool j1_done = false;
    bool j2_done = false;
    bool j3_done = false;
    bool j4_done = false;

    // ── J1 : Stepper 1 ───────────────────────────────────────────────────────
    {
        float error = target.j1 - feedback.j1;
        if (std::fabs(error) > DEADBAND_DEG) {
            _pid_j1.compute(error, dt);
            _stepper1.moveTo(target.j1);
        } else {
            j1_done = true;
        }
    }

    // ── J2 : Stepper 2 ───────────────────────────────────────────────────────
    {
        float error = target.j2 - feedback.j2;
        if (std::fabs(error) > DEADBAND_DEG) {
            _pid_j2.compute(error, dt);
            _stepper2.moveTo(target.j2);
        } else {
            j2_done = true;
        }
    }

    // ── J3 : DC Motor 1 ──────────────────────────────────────────────────────
    {
        float error = target.j3 - feedback.j3;
        if (std::fabs(error) > DEADBAND_DEG) {
            float signal = _pid_j3.compute(error, dt);
            if (signal >  _maxDuty) signal =  _maxDuty;
            if (signal < -_maxDuty) signal = -_maxDuty;
            _dc1.setDuty(signal);
        } else {
            // Only stop + reset once, not every tick — done flag prevents re-call
            _dc1.setStop();
            _pid_j3.reset();
            j3_done = true;
        }
    }

    // ── J4 : DC Motor 2 ──────────────────────────────────────────────────────
    {
        float error = target.j4 - feedback.j4;
        if (std::fabs(error) > DEADBAND_DEG) {
            float signal = _pid_j4.compute(error, dt);
            if (signal >  _maxDuty) signal =  _maxDuty;
            if (signal < -_maxDuty) signal = -_maxDuty;
            _dc2.setDuty(signal);
        } else {
            _dc2.setStop();
            _pid_j4.reset();
            j4_done = true;
        }
    }

    return j1_done && j2_done && j3_done && j4_done;
}

void MotorManager::reset()
{
    _pid_j1.reset();
    _pid_j2.reset();
    _pid_j3.reset();
    _pid_j4.reset();
    _dc1.setStop();
    _dc2.setStop();
}