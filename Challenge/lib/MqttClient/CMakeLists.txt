idf_component_register(
    SRCS        "MqttClient.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        mqtt
        esp_event
)