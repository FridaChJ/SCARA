// =============================================================================
// MqttClient.h
// Description : ESP32-S3 MQTT wrapper (ESP-IDF only).
// Authors     : Frida Sophia Chavez Juarez
// Mentor      : Oscar Vargas Perez
// Last updated: 2026-04-13
// =============================================================================
#pragma once

#include <string>
#include <functional>
#include "mqtt_client.h"

class MQTTClient
{
public:
    MQTTClient();

    void setCallback(std::function<void(std::string, std::string)> cb);

    // uri      e.g. "mqtt://192.168.1.10:1883"
    // username / password — leave empty if broker has no auth
    void init(const std::string& uri,
              const std::string& username = "",
              const std::string& password = "");

    void start();
    void stop();
    void subscribe(const std::string& topic);
    void publish(const std::string& topic, const std::string& message);

    // Block until the broker connection is established.
    // Logs a retry message every `log_every_ms` milliseconds.
    void waitUntilConnected(uint32_t log_every_ms = 2000);

    bool isConnected() const { return connected_; }

private:
    static void event_handler(void*            handler_args,
                               esp_event_base_t base,
                               int32_t          event_id,
                               void*            event_data);

    esp_mqtt_client_handle_t                        client_;
    std::function<void(std::string, std::string)>   message_callback_;
    std::string                                     broker_uri_;
    std::string                                     username_;
    std::string                                     password_;
    volatile bool                                   connected_;
};