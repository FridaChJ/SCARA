idf_component_register(
    SRCS "Stepper.cpp"
    INCLUDE_DIRS "include"
    REQUIRES 
        SimplePWM 
        driver
        Encoders
)