# =============================================================================
# CMakeLists.txt — PIDController component
# Framework : ESP-IDF (no Arduino)
# =============================================================================
 
idf_component_register(
    SRCS
        "PIDcontroller.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        # No ESP-IDF components needed — pure math, no hardware dependencies
)