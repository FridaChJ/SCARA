# =============================================================================
# CMakeLists.txt — main component
# Framework : ESP-IDF (no Arduino)
# =============================================================================

idf_component_register(
    SRCS
        "main.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        MotorAngles     # MotorAngles struct
        MqttClient      # MqttClient
        Encoders         # Encoders (AS5600 + QuadratureEncoder)
        DriverStepper   # StepperMotor
        HBridge          # HBridge
        SimplePWM       # SimplePWM (needed for TimerConfig)
        PIDcontroller   # PIDController
        MotorManager    # MotorManager
        freertos         # vTaskDelayUntil
        log          # ESP_LOGI / ESP_LOGE
        json            # cJSON — ESP-IDF built-in
)