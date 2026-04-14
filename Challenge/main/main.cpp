// =============================================================================
// main.cpp  —  ESP32-S3 Controller
// Authors     : Frida Sophia Chavez Juarez
// Mentor      : Oscar Vargas Perez
// Last updated: 2026-04-13
//
// Change vs previous version:
//   - StepperMotor and HBridge constructors/setup now receive joint labels
//     ("J1","J2","J3","J4") so every log line identifies which motor is acting.
//   - TARGET_READY_BIT is cleared when MotorManager::update() returns true,
//     meaning ALL joints are within deadband. This stops the PID loop (and
//     the setStop spam) once the robot has reached the target position.
// =============================================================================

#include <string>
#include <cstring>
#include <cstdlib>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include "cJSON.h"

#include "MotorAngles.h"
#include "MqttClient.h"
#include "Encoders.h"
#include "DriverStepper.h"
#include "HBridge.h"
#include "SimplePWM.h"
#include "PIDcontroller.h"
#include "MotorManager.h"

static const char* TAG = "main";

#define WIFI_SSID      "Friii"
#define WIFI_PASSWORD  "12345678"

static const std::string BROKER_URI         = "mqtt://192.168.100.14:1883";
static const std::string BROKER_USER        = "";
static const std::string BROKER_PASS        = "";
static const std::string TOPIC_MOTOR_ANGLES = "robot/motor_angles";
static const std::string TOPIC_ENCODERS     = "robot/encoders";

static const TickType_t LOOP_TICKS = pdMS_TO_TICKS(20);   // 50 Hz

// ── Pin definitions ──────────────────────────────────────────────────────────
static const int S1_STEP = 4,  S1_DIR = 5,  S1_EN = 6;
static const int S2_STEP = 7,  S2_DIR = 8,  S2_EN = 9;

static uint8_t DC1_PINS[2] = {10, 11};
static uint8_t DC1_CH[2]   = {0,  1};
static uint8_t DC2_PINS[2] = {12, 13};
static uint8_t DC2_CH[2]   = {2,  3};

static const gpio_num_t ENC_J1_SDA = GPIO_NUM_8,  ENC_J1_SCL = GPIO_NUM_9;
static const gpio_num_t ENC_J2_SDA = GPIO_NUM_14, ENC_J2_SCL = GPIO_NUM_15;

static uint8_t ENC_J3_PINS[2] = {34, 35};
static uint8_t ENC_J4_PINS[2] = {36, 39};

static const float DEG_PER_EDGE_J3 = 0.1f;
static const float DEG_PER_EDGE_J4 = 0.1f;

// ── PID gains ────────────────────────────────────────────────────────────────
static const float J1_KP = 1.0f, J1_KI = 0.0f, J1_KD = 0.0f;
static const float J2_KP = 1.0f, J2_KI = 0.0f, J2_KD = 0.0f;
static const float J3_KP = 1.0f, J3_KI = 0.0f, J3_KD = 0.0f;
static const float J4_KP = 1.0f, J4_KI = 0.0f, J4_KD = 0.0f;
static const float MAX_DUTY = 80.0f;

// ── Shared state ─────────────────────────────────────────────────────────────
// TARGET_READY_BIT: set when a new MQTT target arrives, cleared when all
// joints converge (MotorManager::update() returns true).
#define TARGET_READY_BIT  BIT0

static EventGroupHandle_t s_ctrl_events = nullptr;
static portMUX_TYPE       s_angle_mux   = portMUX_INITIALIZER_UNLOCKED;
static MotorAngles         s_target      = {};

// ── WiFi ─────────────────────────────────────────────────────────────────────
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group = nullptr;
static int s_retry_num = 0;

static void wifi_event_handler(void* arg, esp_event_base_t base,
                               int32_t id, void* data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* event =
            (wifi_event_sta_disconnected_t*) data;
        if (s_retry_num < 10) {
            s_retry_num++;
            ESP_LOGI(TAG, "[WiFi] Retry %d/10 (reason %d)…",
                     s_retry_num, event->reason);
            esp_wifi_connect();
        } else {
            ESP_LOGE(TAG, "[WiFi] Failed after 10 retries. Last reason: %d",
                     event->reason);
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    }
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) data;
        ESP_LOGI(TAG, "[WiFi] Connected — IP: " IPSTR,
                 IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static bool wifi_init_and_wait(void)
{
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_log_level_set("wifi",               ESP_LOG_NONE);
    esp_log_level_set("wifi_init",          ESP_LOG_NONE);
    esp_log_level_set("phy_init",           ESP_LOG_NONE);
    esp_log_level_set("phy",                ESP_LOG_NONE);
    esp_log_level_set("esp_netif_handlers", ESP_LOG_NONE);
    esp_log_level_set("pp",                 ESP_LOG_NONE);
    esp_log_level_set("net80211",           ESP_LOG_NONE);

    esp_event_handler_instance_t instance_any_id, instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid,     WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, WIFI_PASSWORD, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode  = WIFI_AUTH_WPA_PSK;
    wifi_config.sta.pmf_cfg.capable     = true;
    wifi_config.sta.pmf_cfg.required    = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "[WiFi] Connecting to '%s'…", WIFI_SSID);
    esp_wifi_connect();

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE, portMAX_DELAY);

    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGE(TAG, "[WiFi] Could not connect to '%s'. "
                      "Check SSID, password, and that the AP is reachable.", WIFI_SSID);
        return false;
    }
    return true;
}

// ── MQTT callback ─────────────────────────────────────────────────────────────
static void onMessage(std::string topic, std::string payload)
{
    if (topic != TOPIC_MOTOR_ANGLES) return;

    cJSON* doc = cJSON_Parse(payload.c_str());
    if (!doc) { ESP_LOGE(TAG, "[MQTT] Bad JSON: %s", payload.c_str()); return; }

    MotorAngles tmp = s_target;
    cJSON* j1 = cJSON_GetObjectItem(doc, "j1");
    cJSON* j2 = cJSON_GetObjectItem(doc, "j2");
    cJSON* j3 = cJSON_GetObjectItem(doc, "j3");
    cJSON* j4 = cJSON_GetObjectItem(doc, "j4");
    if (cJSON_IsNumber(j1)) tmp.j1 = static_cast<float>(j1->valuedouble);
    if (cJSON_IsNumber(j2)) tmp.j2 = static_cast<float>(j2->valuedouble);
    if (cJSON_IsNumber(j3)) tmp.j3 = static_cast<float>(j3->valuedouble);
    if (cJSON_IsNumber(j4)) tmp.j4 = static_cast<float>(j4->valuedouble);
    cJSON_Delete(doc);

    portENTER_CRITICAL(&s_angle_mux);
    s_target = tmp;
    portEXIT_CRITICAL(&s_angle_mux);

    xEventGroupSetBits(s_ctrl_events, TARGET_READY_BIT);

    ESP_LOGI(TAG, "[MQTT] New target → J1:%.1f°  J2:%.1f°  J3:%.1f°  J4:%.1f°",
             tmp.j1, tmp.j2, tmp.j3, tmp.j4);
}

// ── Hardware init task (CPU1) ─────────────────────────────────────────────────
#define HW_READY_BIT  BIT1
#define HW_FAIL_BIT   BIT2

struct HwBundle {
    StepperMotor*  s1;
    StepperMotor*  s2;
    HBridge*       dc1;
    HBridge*       dc2;
    Encoders*      enc;
    PIDController* pid_j1;
    PIDController* pid_j2;
    PIDController* pid_j3;
    PIDController* pid_j4;
    MotorManager*  mgr;
};
static HwBundle g_hw = {};

static void hw_init_task(void* /*arg*/)
{
    ESP_LOGI(TAG, "[HW] Hardware init starting on CPU1…");

    // Steppers — pass joint label so logs show [J1] and [J2]
    g_hw.s1 = new StepperMotor(S1_STEP, S1_DIR, S1_EN, -1, "J1");
    g_hw.s2 = new StepperMotor(S2_STEP, S2_DIR, S2_EN, -1, "J2");
    g_hw.s1->begin();
    g_hw.s2->begin();
    g_hw.s1->enable();
    g_hw.s2->enable();
    vTaskDelay(pdMS_TO_TICKS(10));

    // H-bridges — pass joint label so logs show [J3] and [J4]
    TimerConfig dcTimerCfg = {
        .timer          = LEDC_TIMER_0,
        .frequency      = 25000,
        .bit_resolution = LEDC_TIMER_10_BIT,
        .mode           = LEDC_LOW_SPEED_MODE
    };
    g_hw.dc1 = new HBridge();
    g_hw.dc2 = new HBridge();
    g_hw.dc1->setup(DC1_PINS, DC1_CH, &dcTimerCfg, "J3");
    vTaskDelay(pdMS_TO_TICKS(10));
    g_hw.dc2->setup(DC2_PINS, DC2_CH, &dcTimerCfg, "J4");
    vTaskDelay(pdMS_TO_TICKS(10));

    // Encoders
    g_hw.enc = new Encoders();
    g_hw.enc->setup(ENC_J1_SDA, ENC_J1_SCL,
                    ENC_J2_SDA, ENC_J2_SCL,
                    ENC_J3_PINS, DEG_PER_EDGE_J3,
                    ENC_J4_PINS, DEG_PER_EDGE_J4);
    vTaskDelay(pdMS_TO_TICKS(10));

    // PID controllers
    g_hw.pid_j1 = new PIDController(J1_KP, J1_KI, J1_KD);
    g_hw.pid_j2 = new PIDController(J2_KP, J2_KI, J2_KD);
    g_hw.pid_j3 = new PIDController(J3_KP, J3_KI, J3_KD);
    g_hw.pid_j4 = new PIDController(J4_KP, J4_KI, J4_KD);

    // Motor manager
    g_hw.mgr = new MotorManager(*g_hw.s1, *g_hw.s2,
                                 *g_hw.dc1, *g_hw.dc2,
                                 *g_hw.pid_j1, *g_hw.pid_j2,
                                 *g_hw.pid_j3, *g_hw.pid_j4,
                                 MAX_DUTY);

    ESP_LOGI(TAG, "[HW] Hardware ready.");
    xEventGroupSetBits(s_ctrl_events, HW_READY_BIT);
    vTaskDelete(NULL);
}

// =============================================================================
// app_main
// =============================================================================
extern "C" void app_main(void)
{
    esp_log_level_set("ledc",           ESP_LOG_NONE);
    esp_log_level_set("gpio",           ESP_LOG_NONE);
    esp_log_level_set("I2C",            ESP_LOG_NONE);
    esp_log_level_set("i2c",            ESP_LOG_NONE);
    esp_log_level_set("task_wdt",       ESP_LOG_NONE);
    esp_log_level_set("MQTT_CLIENT",    ESP_LOG_NONE);
    esp_log_level_set("mqtt_client",    ESP_LOG_NONE);
    esp_log_level_set("transport_base", ESP_LOG_NONE);
    esp_log_level_set("esp-tls",        ESP_LOG_NONE);
    esp_log_level_set("transport",      ESP_LOG_NONE);
    esp_log_level_set("outbox",         ESP_LOG_NONE);

    ESP_LOGI(TAG, "=== Controller booting ===");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_ctrl_events = xEventGroupCreate();

    // ── WiFi ─────────────────────────────────────────────────────────────────
    if (!wifi_init_and_wait()) {
        ESP_LOGE(TAG, "[WiFi] No network — halting.");
        vTaskSuspend(NULL);
    }

    // ── MQTT ─────────────────────────────────────────────────────────────────
    MQTTClient mqttClient;
    mqttClient.setCallback(onMessage);
    mqttClient.init(BROKER_URI, BROKER_USER, BROKER_PASS);

    ESP_LOGI(TAG, "[MQTT] Connecting to %s…", BROKER_URI.c_str());
    mqttClient.start();

    for (int mqttRetry = 0; !mqttClient.isConnected(); ++mqttRetry) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (mqttRetry % 4 == 0)
            ESP_LOGI(TAG, "[MQTT] Waiting for broker… (%d s)", mqttRetry / 2);
        if (mqttRetry >= 40) {
            ESP_LOGE(TAG, "[MQTT] Broker unreachable at %s after 20 s. "
                          "Check IP, port, and that the broker is running.",
                     BROKER_URI.c_str());
            vTaskSuspend(NULL);
        }
    }
    ESP_LOGI(TAG, "[MQTT] Connected — listening on '%s'",
             TOPIC_MOTOR_ANGLES.c_str());
    mqttClient.subscribe(TOPIC_MOTOR_ANGLES);

    // ── Hardware init on CPU1 ─────────────────────────────────────────────────
    ESP_LOGI(TAG, "[HW] Starting hardware init on CPU1…");
    xTaskCreatePinnedToCore(hw_init_task, "hw_init", 8192, NULL, 5, NULL, 1);
    xEventGroupWaitBits(s_ctrl_events, HW_READY_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "[HW] Hardware ready — entering control loop at 50 Hz");
    ESP_LOGI(TAG, "[HW] Waiting for first target on '%s'",
             TOPIC_MOTOR_ANGLES.c_str());

    // =========================================================================
    // Control loop — 50 Hz, CPU0
    // =========================================================================
    TickType_t lastWake = xTaskGetTickCount();
    const float dt      = LOOP_TICKS / static_cast<float>(configTICK_RATE_HZ);

    while (true) {

        // 1. Read encoders
        MotorAngles feedback = g_hw.enc->readAll();

        // 2. Service stepper acceleration state machines
        g_hw.s1->update();
        g_hw.s2->update();

        // 3. Run PID if a target is active
        EventBits_t bits = xEventGroupGetBits(s_ctrl_events);
        if (bits & TARGET_READY_BIT) {
            MotorAngles localTarget;
            portENTER_CRITICAL(&s_angle_mux);
            localTarget = s_target;
            portEXIT_CRITICAL(&s_angle_mux);

            // update() returns true when ALL joints are within deadband.
            // Clear the bit then so the PID loop stops until a new target arrives.
            bool allDone = g_hw.mgr->update(localTarget, feedback, dt);
            if (allDone) {
                xEventGroupClearBits(s_ctrl_events, TARGET_READY_BIT);
                ESP_LOGI(TAG, "[CTRL] All joints at target — motors idle.");
            }
        }

        // 4. Publish encoder angles (no terminal log — noisy)
        cJSON* doc = cJSON_CreateObject();
        cJSON_AddNumberToObject(doc, "j1", feedback.j1);
        cJSON_AddNumberToObject(doc, "j2", feedback.j2);
        cJSON_AddNumberToObject(doc, "j3", feedback.j3);
        cJSON_AddNumberToObject(doc, "j4", feedback.j4);

        char* payload = cJSON_PrintUnformatted(doc);
        if (payload) {
            mqttClient.publish(TOPIC_ENCODERS, std::string(payload));
            cJSON_free(payload);
        }
        cJSON_Delete(doc);

        vTaskDelayUntil(&lastWake, LOOP_TICKS);
    }
}