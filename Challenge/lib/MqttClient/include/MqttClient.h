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
#include <mutex>
#include <condition_variable>
#include <deque>
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
    bool waitUntilConnected(uint32_t timeout_ms = 0, uint32_t log_every_ms = 2000);

    bool isConnected() const { return connected_; }

    // Blocking read: waits up to `timeout_ms` milliseconds for a message
    // on `topic`. Returns true and fills `out_message` on success.
    bool readMessage(const std::string& topic, std::string& out_message, uint32_t timeout_ms = 5000);

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
    // Message queue & synchronization for blocking reads
    std::mutex                                      msg_mutex_;
    std::condition_variable                         msg_cv_;
    std::deque<std::pair<std::string, std::string>> message_queue_;
};