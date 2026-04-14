# =============================================================================
# CMakeLists.txt — HBridge component
# Framework : ESP-IDF 
# =============================================================================

idf_component_register(
    SRCS
        "HBridge.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        SimplePWM   # SimplePWM component
)