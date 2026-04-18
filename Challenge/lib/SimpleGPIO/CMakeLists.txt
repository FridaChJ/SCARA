idf_component_register(
    SRCS "SimpleGPIO.cpp"
    INCLUDE_DIRS "include"
    REQUIRES 
        driver
        esp_timer
        esp_common
)