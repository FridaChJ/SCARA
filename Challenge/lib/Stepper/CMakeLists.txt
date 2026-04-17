idf_component_register(
    SRCS "Stepper.cpp"
    INCLUDE_DIRS "."
    REQUIRES 
        SimplePWM 
        driver
)