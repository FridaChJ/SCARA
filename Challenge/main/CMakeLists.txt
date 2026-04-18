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
        Stepper
        SimpleGPIO
        MotorAngles     # MotorAngles struct
        MqttClient      # MqttClient
        Encoders         # Encoders (AS5600 + QuadratureEncoder)
        HBridge          # HBridge
        SimplePWM       # SimplePWM (needed for TimerConfig)
        freertos         # vTaskDelayUntil
        log          # ESP_LOGI / ESP_LOGE
        json            # cJSON — ESP-IDF built-in
        esp_timer       # esp_timer_get_time
)