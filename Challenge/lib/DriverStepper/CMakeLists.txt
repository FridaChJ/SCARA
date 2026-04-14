# =============================================================================
# CMakeLists.txt — DriverStepper component
# Framework : ESP-IDF 
# =============================================================================

idf_component_register(
    SRCS
        "DriverStepper.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        driver      # gpio, gptimer
        esp_rom     # ets_delay_us (IRAM pulse timing)
        log
)