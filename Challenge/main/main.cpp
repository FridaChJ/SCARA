// =============================================================================
// main.cpp  —  ESP32-S3 Controller
// Authors     : Frida Sophia Chavez Juarez
// Mentor      : Oscar Vargas Perez
// Last updated: 2026-04-17
//
// Simplified control — no MotorManager, no PID:
//   • Receives target angles via MQTT (topic: robot/motor_angles)
//   • Publishes encoder feedback via MQTT (topic: robot/encoders)
//   • DC Motor 1 (J3) and DC Motor 2 (J4) — run simultaneously:
//       - Positive target → spin RIGHT at up to 60% duty
//       - Negative target → spin LEFT  at up to 60% duty
//       - Proportional slowdown inside SLOW_ZONE degrees of target
//       - Stops when within STOP_ZONE degrees of target
// =============================================================================


#include "definitions.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_task_wdt.h"
#include "MqttClient.h"
#include "Encoders.h"
#include "MotorAngles.h"
#include "HBridge.h"
#include "SimplePWM.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include <cmath>
#include "include/Stepper.h"
#include "SimpleGPIO.h"
// ── Stepper J1 ─────────────────────────────────────────────────────────


#define INTERRUPT_PIN   GPIO_NUM_1    // same as S1_STEP pin
#define step_per_rev    200
static uint8_t STEPPERPINS[2] = {1, 2};




static TimerConfig timer = {
    .timer          = LEDC_TIMER_1,        // TIMER_0 is used by DC motors, use TIMER_1
    .frequency      = 1000,
    .bit_resolution = LEDC_TIMER_10_BIT,
    .mode           = LEDC_LOW_SPEED_MODE
};


static Stepper    Smotor;
static SimpleGPIO interrupt_pin;


static const char* TAG = "main";
static const TickType_t LOOP_TICKS = pdMS_TO_TICKS(20); // 50 Hz
static volatile float  s_j1_position  = 0.0f;   // tracks current stepper position (degrees)
static volatile bool   s_j1_direction = true;    // true = CW, false = CCW
static volatile long   s_j1_steps     = 0;       // total step count


static float ShortPath(float current, float target) {
    float error = target - current;
    if (error >  180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;
    return error;
}


static void IRAM_ATTR j1_step_handler(void* arg) {
    float step_degrees = 360.0f / step_per_rev;
    if (s_j1_direction) s_j1_position += step_degrees;
    else                s_j1_position -= step_degrees;
    if (s_j1_position >= 360.0f) s_j1_position -= 360.0f;
    if (s_j1_position <    0.0f) s_j1_position += 360.0f;
    s_j1_steps = s_j1_steps + 1;;
}
// =============================================================================
// Motor tuning constants  — adjust these to taste
// =============================================================================
static constexpr float MAX_DUTY_J3  = 60.0f;  // full-speed PWM duty (%) for J3
static constexpr float MAX_DUTY_J4  = 60.0f;  // full-speed PWM duty (%) for J4
static constexpr float SLOW_ZONE    = 15.0f;  // degrees from target → start slowing
static constexpr float STOP_ZONE    =  1.5f;  // degrees from target → stop completely
static constexpr float MIN_DUTY_J3  = 20.0f;  // minimum duty while still moving (%) J3
static constexpr float MIN_DUTY_J4  = 20.0f;  // minimum duty while still moving (%) J4


// =============================================================================
// Shared State
// =============================================================================
static EventGroupHandle_t s_ctrl_events = nullptr;
static portMUX_TYPE       s_angle_mux   = portMUX_INITIALIZER_UNLOCKED;
static MotorAngles         s_target      = {};


// =============================================================================
// WiFi
// =============================================================================
static EventGroupHandle_t s_wifi_event_group = nullptr;
static int s_retry_num = 0;


static void wifi_event_handler(void* arg, esp_event_base_t base,
                                int32_t id, void* data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*)data;
        if (s_retry_num < 10) {
            s_retry_num++;
            ESP_LOGI(TAG, "[WiFi] Retry %d/10 (reason %d)...", s_retry_num, event->reason);
            esp_wifi_connect();
        } else {
            ESP_LOGE(TAG, "[WiFi] Failed after 10 retries. Last reason: %d", event->reason);
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)data;
        ESP_LOGI(TAG, "[WiFi] Connected — IP: " IPSTR, IP2STR(&event->ip_info.ip));
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
    strncpy((char*)wifi_config.sta.ssid,     WIFI_SSID,     sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, WIFI_PASSWORD, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA_PSK;
    wifi_config.sta.pmf_cfg.capable    = true;
    wifi_config.sta.pmf_cfg.required   = false;


    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());


    ESP_LOGI(TAG, "[WiFi] Connecting to '%s'...", WIFI_SSID);
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


// =============================================================================
// MQTT Callback
// =============================================================================
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


// =============================================================================
// Hardware  (global — initialised in hw_init_task)
// =============================================================================
static HBridge*  g_dc1  = nullptr;   // J3 — DC Motor 1
static HBridge*  g_dc2  = nullptr;   // J4 — DC Motor 2
static Encoders* g_enc  = nullptr;


static void hw_init_task(void* /*arg*/)
{
    ESP_LOGI(TAG, "[HW] Hardware init starting on CPU1...");


    // ── Shared LEDC timer for both DC motors ──────────────────────────────
    TimerConfig dcTimerCfg = {
        .timer          = LEDC_TIMER_0,
        .frequency      = 25000,
        .bit_resolution = LEDC_TIMER_10_BIT,
        .mode           = LEDC_LOW_SPEED_MODE
    };


    // ── H-bridge for DC motor 1 (J3) ─────────────────────────────────────
    g_dc1 = new HBridge();
    g_dc1->setup(DC1_PINS, DC1_CH, &dcTimerCfg, "J3");
    vTaskDelay(pdMS_TO_TICKS(10));


    // ── H-bridge for DC motor 2 (J4) ─────────────────────────────────────
    g_dc2 = new HBridge();
    g_dc2->setup(DC2_PINS, DC2_CH, &dcTimerCfg, "J4");
    vTaskDelay(pdMS_TO_TICKS(10));


    // ── Encoders ─────────────────────────────────────────────────────────
    g_enc = new Encoders();
    g_enc->setup(ENC_I2C_SDA,  ENC_I2C_SCL,
                 ENC2_I2C_SDA, ENC2_I2C_SCL,
                 ENC_J3_PINS,  DEG_PER_EDGE_J3,
                 ENC_J4_PINS,  DEG_PER_EDGE_J4);
    vTaskDelay(pdMS_TO_TICKS(10));


    ESP_LOGI(TAG, "[HW] Hardware ready.");
    xEventGroupSetBits(s_ctrl_events, HW_READY_BIT);
    vTaskDelete(NULL);


    // ── Stepper motor (J1) ────────────────────────────────────────────────
    Smotor.setup(STEPPERPINS, 0, &timer, step_per_rev);
    interrupt_pin.setup(INTERRUPT_PIN, GPIO_MODE_INPUT);
    interrupt_pin.addInterrupt(GPIO_INTR_POSEDGE, &j1_step_handler);
}


// =============================================================================
// computeDuty — proportional slowdown, same sign as error
//
//   |error| >= SLOW_ZONE               →  sign * maxDuty
//   |error| in (STOP_ZONE, SLOW_ZONE)  →  linearly scaled minDuty..maxDuty
//   |error| <= STOP_ZONE               →  0  (stopped)
//
// Returned value is SIGNED: positive = spin right, negative = spin left.
// =============================================================================
static float computeDuty(float error, float maxDuty, float minDuty)
{
    float absErr = (error >= 0.0f) ? error : -error;
    float sign   = (error >= 0.0f) ? 1.0f : -1.0f;


    if (absErr <= STOP_ZONE) {
        return 0.0f;                                          // inside dead-band → stop
    }
    if (absErr >= SLOW_ZONE) {
        return sign * maxDuty;                                // far away → full speed
    }
    // Linear scale between minDuty and maxDuty inside the slow zone
    float fraction = (absErr - STOP_ZONE) / (SLOW_ZONE - STOP_ZONE);
    float duty     = minDuty + fraction * (maxDuty - minDuty);
    return sign * duty;
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


    // ── WiFi ─────────────────────────────────────────────────────────────
    if (!wifi_init_and_wait()) {
        ESP_LOGE(TAG, "[WiFi] No network — halting.");
        vTaskSuspend(NULL);
    }


    // ── MQTT ─────────────────────────────────────────────────────────────
    MQTTClient mqttClient;
    mqttClient.setCallback(onMessage);
    mqttClient.init(BROKER_URI, BROKER_USER, BROKER_PASS);


    ESP_LOGI(TAG, "[MQTT] Connecting to %s...", BROKER_URI.c_str());
    mqttClient.start();


    for (int mqttRetry = 0; !mqttClient.isConnected(); ++mqttRetry) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (mqttRetry % 4 == 0)
            ESP_LOGI(TAG, "[MQTT] Waiting for broker... (%d s)", mqttRetry / 2);
        if (mqttRetry >= 40) {
            ESP_LOGE(TAG, "[MQTT] Broker unreachable at %s after 20 s.",
                     BROKER_URI.c_str());
            vTaskSuspend(NULL);
        }
    }
    ESP_LOGI(TAG, "[MQTT] Connected — listening on '%s'", TOPIC_MOTOR_ANGLES.c_str());
    mqttClient.subscribe(TOPIC_MOTOR_ANGLES);


    // ── Hardware init on CPU1 ─────────────────────────────────────────────
    xTaskCreatePinnedToCore(hw_init_task, "hw_init", 8192, NULL, 5, NULL, 1);
    xEventGroupWaitBits(s_ctrl_events, HW_READY_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "[HW] Hardware ready — entering control loop at 50 Hz");


    // =========================================================================
    // Control loop — 50 Hz, CPU0
    // =========================================================================
    TickType_t lastWake  = xTaskGetTickCount();
    bool       j3_active = false;   // true while we are driving toward a J3 target
    bool       j4_active = false;   // true while we are driving toward a J4 target


    while (true)
    {
        // 1. Read encoders
        MotorAngles feedback = g_enc->readAll();


        // 2. Check whether a target is pending
        EventBits_t bits = xEventGroupGetBits(s_ctrl_events);
        if (bits & TARGET_READY_BIT)
        {
            MotorAngles localTarget;
            portENTER_CRITICAL(&s_angle_mux);
            localTarget = s_target;
            portEXIT_CRITICAL(&s_angle_mux);
            // ── J1 (Stepper Motor 1) ──────────────────────────────────────────────
            {
                float error = ShortPath(s_j1_position, localTarget.j1);
                float rpm   = error;                          // proportional
                if (rpm >  30.0f) rpm =  30.0f;              // clamp to ±30 RPM
                if (rpm < -30.0f) rpm = -30.0f;


                s_j1_direction = (rpm > 0.0f);


                if (fabsf(error) <= 1.9f) {
                    Smotor.setSpeed(0.0f);
                    ESP_LOGI(TAG, "[J1] Target reached — pos:%.2f°  target:%.2f°",
                            s_j1_position, localTarget.j1);
                } else {
                    Smotor.setSpeed(rpm);
                    ESP_LOGI(TAG, "[J1] pos:%.2f°  target:%.2f°  error:%.2f°  rpm:%.1f",
                            s_j1_position, localTarget.j1, error, rpm);
                }
            }


            // ── J3 (DC Motor 1) ───────────────────────────────────────────
            {
                float error = localTarget.j3 - feedback.j3;
                float duty  = computeDuty(error, MAX_DUTY_J3, MIN_DUTY_J3);


                if (duty == 0.0f)
                {
                    g_dc1->setStop();


                    if (j3_active) {
                        ESP_LOGI(TAG, "[J3] Target reached — feedback:%.2f°  target:%.2f°",
                                 feedback.j3, localTarget.j3);
                        j3_active = false;
                    }
                }
                else
                {
                    g_dc1->setDuty(duty);
                    j3_active = true;


                    ESP_LOGI(TAG, "[J3] target:%.2f°  feedback:%.2f°  error:%.2f°  duty:%.1f%%",
                             localTarget.j3, feedback.j3, error, duty);
                }
            }


            // ── J4 (DC Motor 2) ───────────────────────────────────────────
            {
                float error = localTarget.j4 - feedback.j4;
                float duty  = computeDuty(error, MAX_DUTY_J4, MIN_DUTY_J4);


                if (duty == 0.0f)
                {
                    g_dc2->setStop();


                    if (j4_active) {
                        ESP_LOGI(TAG, "[J4] Target reached — feedback:%.2f°  target:%.2f°",
                                 feedback.j4, localTarget.j4);
                        j4_active = false;
                    }
                }
                else
                {
                    g_dc2->setDuty(duty);
                    j4_active = true;


                    ESP_LOGI(TAG, "[J4] target:%.2f°  feedback:%.2f°  error:%.2f°  duty:%.1f%%",
                             localTarget.j4, feedback.j4, error, duty);
                }
            }


            // Clear TARGET_READY_BIT only when BOTH motors have reached their targets
            if (!j3_active && !j4_active) {
                xEventGroupClearBits(s_ctrl_events, TARGET_READY_BIT);
            }
        }
        else
        {
            // No active target — make sure both motors are stopped
            g_dc1->setStop();
            g_dc2->setStop();
            j3_active = false;
            j4_active = false;
        }


        // 3. Publish all encoder angles to broker
        cJSON* doc = cJSON_CreateObject();
        char buf[16];
        snprintf(buf, sizeof(buf), "%.2f", s_j1_position); cJSON_AddRawToObject(doc, "j1", buf);
        snprintf(buf, sizeof(buf), "%.2f", feedback.j2); cJSON_AddRawToObject(doc, "j2", buf);
        snprintf(buf, sizeof(buf), "%.2f", feedback.j3); cJSON_AddRawToObject(doc, "j3", buf);
        snprintf(buf, sizeof(buf), "%.2f", feedback.j4); cJSON_AddRawToObject(doc, "j4", buf);
        char* payload = cJSON_PrintUnformatted(doc);
        if (payload) {
            mqttClient.publish(TOPIC_ENCODERS, std::string(payload));
            cJSON_free(payload);
        }
        cJSON_Delete(doc);


        vTaskDelayUntil(&lastWake, LOOP_TICKS);
    }
}

