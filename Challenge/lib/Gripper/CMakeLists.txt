# =============================================================================
# CMakeLists.txt — Gripper component
# =============================================================================

idf_component_register(
    SRCS "Gripper.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver
)