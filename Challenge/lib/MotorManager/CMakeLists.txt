# =============================================================================
# CMakeLists.txt — MotorManager component
# Framework : ESP-IDF (no Arduino)
# =============================================================================

idf_component_register(
    SRCS
        "MotorManager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        PIDcontroller   # PIDController
        DriverStepper   # StepperMotor
        HBridge          # HBridge
        MotorAngles     # MotorAngles struct
)