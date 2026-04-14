#pragma once

// =============================================================================
// MotorManager.h
// Description : Central coordinator for all four motors.
//               Receives target angles and encoder feedback, computes error
//               per joint, runs each PIDController, and sends the correction
//               signal to the appropriate motor driver.
//
//               Motor interface contract (must match driver classes):
//                 j1, j2 (steppers) → StepperMotor::moveTo(float degrees)
//                 j3, j4 (DC)       → HBridge::setDuty(float duty -100..+100)
//
//               Knows nothing about MQTT, JSON, or kinematics.
// Authors     : Frida Sophia Chavez Juarez
// Mentor      : Oscar Vargas Perez
// Last updated: 2026-04-11
// Dependencies: motor_angles.h, PIDController, DriverStepper, HBridge
// =============================================================================

#include "MotorAngles.h"
#include "PIDcontroller.h"
#include "DriverStepper.h"
#include "HBridge.h"

class MotorManager {
public:
    // -------------------------------------------------------------------------
    // Constructor
    //   All objects must be created in app_main and outlive MotorManager.
    //   maxDuty : maximum absolute duty cycle sent to HBridge (0–100.0)
    //             limits DC motor speed during PID correction
    // -------------------------------------------------------------------------
    MotorManager(
        StepperMotor&  stepper1,
        StepperMotor&  stepper2,
        HBridge&       dc1,
        HBridge&       dc2,
        PIDController& pid_j1,
        PIDController& pid_j2,
        PIDController& pid_j3,
        PIDController& pid_j4,
        float          maxDuty = 80.0f
    );

    // -------------------------------------------------------------------------
    // update()
    //   Call once per control loop tick.
    //   target   : desired joint angles from Brain PC (degrees)
    //   feedback : current joint angles from encoders  (degrees)
    //   dt       : elapsed time since last call        (seconds)
    // -------------------------------------------------------------------------
    bool update(const MotorAngles& target,
                const MotorAngles& feedback,
                float dt);

    // -------------------------------------------------------------------------
    // reset()
    //   Clears all PID states and stops DC motors.
    //   Call after e-stop or long pause.
    // -------------------------------------------------------------------------
    void reset();

    // -------------------------------------------------------------------------
    // Deadband — errors smaller than this (degrees) are ignored.
    // Prevents jitter from encoder noise near the target.
    // -------------------------------------------------------------------------
    static constexpr float DEADBAND_DEG = 0.5f;

private:
    StepperMotor&  _stepper1;
    StepperMotor&  _stepper2;
    HBridge&       _dc1;
    HBridge&       _dc2;

    PIDController& _pid_j1;
    PIDController& _pid_j2;
    PIDController& _pid_j3;
    PIDController& _pid_j4;

    float _maxDuty;
};