idf_component_register(
    SRCS "SimplePWM.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        driver
        esp_driver_ledc
)
