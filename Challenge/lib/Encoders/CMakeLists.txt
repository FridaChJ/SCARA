# =============================================================================
# CMakeLists.txt — Encoders component
# Framework : ESP-IDF (no Arduino)
# =============================================================================

idf_component_register(
    SRCS
        "Encoders.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        AS5600              # AS5600_i2c class
        QuadratureEncoder  # QuadratureEncoder class
        MotorAngles        # MotorAngles struct
        driver              # i2c, gpio
        log
)