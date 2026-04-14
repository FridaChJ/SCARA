#pragma once

// =============================================================================
// MotorAngles
// Description : Shared data structure used across EncoderReader, MotorManager,
//               and main to carry the four joint angles.
//
//               j1 → stepper 1   (TB6600 driver)
//               j2 → stepper 2   (TB6600 driver)
//               j3 → DC motor 1  (H-bridge)
//               j4 → DC motor 2  (H-bridge)
//
//               Units : degrees, range -360.0 to 360.0
// =============================================================================

struct MotorAngles {
    float j1 = 0.0f;
    float j2 = 0.0f;
    float j3 = 0.0f;
    float j4 = 0.0f;
};