// =============================================================================
// MqttClient.cpp
// Description : ESP32-S3 MQTT wrapper implementation (ESP-IDF only).
// Authors     : Frida Sophia Chavez Juarez
// Mentor      : Oscar Vargas Perez
// Last updated: 2026-04-13
// =============================================================================

#include "MqttClient.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <chrono>

static const char* TAG = "MQTTClient";

// =============================================================================
// Constructor
// =============================================================================
MQTTClient::MQTTClient()
    : client_(nullptr), connected_(false)
{
}

// =============================================================================
// setCallback
// =============================================================================
void MQTTClient::setCallback(std::function<void(std::string, std::string)> cb)
{
    message_callback_ = cb;
}

// =============================================================================
// init
// =============================================================================
void MQTTClient::init(const std::string& uri,
                      const std::string& username,
                      const std::string& password)
{
    broker_uri_ = uri;
    username_   = username;
    password_   = password;

    esp_mqtt_client_config_t config = {};
    config.broker.address.uri = broker_uri_.c_str();

    if (!username_.empty()) {
        config.credentials.username = username_.c_str();
    }
    if (!password_.empty()) {
        config.credentials.authentication.password = password_.c_str();
    }

    client_ = esp_mqtt_client_init(&config);

    if (client_ == nullptr) {
        ESP_LOGE(TAG, "esp_mqtt_client_init() failed — out of memory?");
        return;
    }

    esp_mqtt_client_register_event(
        client_,
        MQTT_EVENT_ANY,
        event_handler,
        this
    );

    ESP_LOGI(TAG, "Initialized — broker: %s  user: %s",
             broker_uri_.c_str(),
             username_.empty() ? "(none)" : username_.c_str());
}

// =============================================================================
// start
// =============================================================================
void MQTTClient::start()
{
    if (client_ == nullptr) {
        ESP_LOGE(TAG, "start() called before init()");
        return;
    }
    ESP_LOGI(TAG, "Starting MQTT client — attempting to connect to %s ...", broker_uri_.c_str());
    esp_mqtt_client_start(client_);
}

// =============================================================================
// waitUntilConnected
// =============================================================================
bool MQTTClient::waitUntilConnected(uint32_t timeout_ms, uint32_t log_every_ms)
{
    if (client_ == nullptr) {
        ESP_LOGE(TAG, "waitUntilConnected() called before init()");
        return false;
    }

    const TickType_t poll   = pdMS_TO_TICKS(100);
    const uint32_t   period = (log_every_ms > 0) ? (log_every_ms / 100) : 20;   // how many 100 ms ticks per log
    uint32_t         ticks  = 0;
    TickType_t       waited = 0;
    const TickType_t max_wait = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

    while (!connected_ && waited < max_wait) {
        vTaskDelay(poll);
        waited += poll;
        ticks++;
        if (ticks % period == 0) {
            ESP_LOGW(TAG, "Not connected yet — retrying... (broker: %s)", broker_uri_.c_str());
        }
    }

    if (connected_) {
        ESP_LOGI(TAG, "Connected to broker!");
        return true;
    }

    ESP_LOGE(TAG, "MQTT broker connection timed out after %lu ms", (unsigned long)timeout_ms);
    return false;
}

// =============================================================================
// stop
// =============================================================================
void MQTTClient::stop()
{
    if (client_) {
        esp_mqtt_client_stop(client_);
        ESP_LOGI(TAG, "Client stopped");
    }
}

// =============================================================================
// subscribe
// =============================================================================
void MQTTClient::subscribe(const std::string& topic)
{
    if (client_ == nullptr) {
        ESP_LOGE(TAG, "subscribe() called before init()");
        return;
    }
    int msg_id = esp_mqtt_client_subscribe(client_, topic.c_str(), 0);
    ESP_LOGI(TAG, "Subscribed to '%s'  (msg_id=%d)", topic.c_str(), msg_id);
}

// =============================================================================
// publish
// =============================================================================
void MQTTClient::publish(const std::string& topic, const std::string& message)
{
    if (client_ == nullptr) {
        ESP_LOGE(TAG, "publish() called before init()");
        return;
    }
    int msg_id = esp_mqtt_client_publish(
        client_,
        topic.c_str(),
        message.c_str(),
        0,   // length 0 → ESP-IDF uses strlen()
        0,   // QoS 0
        0    // retain = false
    );
    ESP_LOGD(TAG, "Published to '%s'  (msg_id=%d)  payload: %s",
             topic.c_str(), msg_id, message.c_str());
}

// =============================================================================
// event_handler  (static)
// =============================================================================
void MQTTClient::event_handler(void*            handler_args,
                               esp_event_base_t base,
                               int32_t          event_id,
                               void*            event_data)
{
    MQTTClient*             self  = static_cast<MQTTClient*>(handler_args);
    esp_mqtt_event_handle_t event = static_cast<esp_mqtt_event_handle_t>(event_data);

    switch (event_id) {

        case MQTT_EVENT_CONNECTED:
            self->connected_ = true;
            ESP_LOGI(TAG, "✔ Connected to broker");
            break;

        case MQTT_EVENT_DISCONNECTED:
            self->connected_ = false;
            ESP_LOGW(TAG, "✘ Disconnected from broker — will retry automatically");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "Subscription confirmed (msg_id=%d)", event->msg_id);
            break;

        case MQTT_EVENT_DATA: {
            std::string topic(event->topic,  event->topic_len);
            std::string message(event->data, event->data_len);

            ESP_LOGI(TAG, "▶ Message | topic: %s | payload: %s",
                     topic.c_str(), message.c_str());

            if (self->message_callback_) {
                self->message_callback_(topic, message);
            }
            // Store message in queue for blocking reads
            {
                std::lock_guard<std::mutex> lk(self->msg_mutex_);
                self->message_queue_.emplace_back(topic, message);
            }
            self->msg_cv_.notify_all();
            break;
        }

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "✘ MQTT error — check broker address and credentials");
            break;

        default:
            break;
    }
}

// =============================================================================
// readMessage
// =============================================================================
bool MQTTClient::readMessage(const std::string& topic, std::string& out_message, uint32_t timeout_ms)
{
    std::unique_lock<std::mutex> lk(msg_mutex_);
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

    for (;;) {
        // Search for a queued message matching the topic
        for (auto it = message_queue_.begin(); it != message_queue_.end(); ++it) {
            if (it->first == topic) {
                out_message = it->second;
                message_queue_.erase(it);
                return true;
            }
        }

        if (timeout_ms == 0) {
            return false;
        }

        if (msg_cv_.wait_until(lk, deadline) == std::cv_status::timeout) {
            // One last check in case a message arrived exactly before timeout
            for (auto it = message_queue_.begin(); it != message_queue_.end(); ++it) {
                if (it->first == topic) {
                    out_message = it->second;
                    message_queue_.erase(it);
                    return true;
                }
            }
            return false;
        }
        // Otherwise, loop and search again
    }
}