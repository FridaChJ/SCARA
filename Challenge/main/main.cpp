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

// =================================INCLUDES=================================
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
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include "driver/i2c_master.h"
#include "AS5600.h"
#include "driver/i2c.h"
#include "freertos/event_groups.h"
// =================================GLOBALS=================================
//static const char* TAG = "MAIN";
//static const char* TAG = "MotorCtrl";
typedef enum {
    STATE_INIT,
    STATE_IDLE,
    STATE_EXECUTE_CYCLE
} RobotState;
//=====Wifi 
static const char* TAG = "WiFi";
static EventGroupHandle_t s_wifi_event_group = nullptr;
static int s_retry_count = 0;
//=====
static Encoders* g_enc  = nullptr;  // global encoder instance used by init_hardware()
static RobotState state = STATE_INIT;
static volatile bool s_new_message = false;
static std::string   s_payload     = "";       // ← add this line
static std::string   s_color       = "";
static float         s_j1_target   = 0.0f;
static float         s_j3_target   = 0.0f;
//================Stepper J1
#define STEP_PER_REV    6400  // full microstep resolution for J1 stepper

static uint8_t    STEPPERPINS[2]   = {S1_STEP, S1_DIR};
static TimerConfig s_stepper_timer = {
    .timer          = LEDC_TIMER_1,
    .frequency      = 1000,
    .bit_resolution = LEDC_TIMER_11_BIT,  // 12 bits para mayor resolución (4096 cuentas)
    .mode           = LEDC_LOW_SPEED_MODE
};
static Stepper Smotor;

//================DC motors
static HBridge* g_dc1 = nullptr;   // J3
static HBridge* g_dc2 = nullptr;   // J4

//================Encoder pointer (matches old code pattern)
//static Encoders* g_enc = nullptr;

static float computeDuty(float error, float maxDuty, float minDuty);
static float clampf(float value, float min_value, float max_value);
//static SemaphoreHandle_t g_i2c_mutex = nullptr;

//================================================================================
//=================================HELPER FUNCTIONS===============================
// =================================Move DC Motors================================
//J3 
static float g_j3_current_angle = 0.0f;  // updated after every move

static void controlDcJoint(const char* tag,
                           HBridge*    motor,
                           float       target,
                           float       feedback,
                           float       max_duty,
                           float       min_duty,
                           bool*       active)
{
    float error    = target - feedback;
    float abs_error = fabsf(error);
    float duty     = 0.0f;

    if (abs_error <= STOP_ZONE)
    {
        // Inside deadband — stop
        duty = 0.0f;
    }
    else if (abs_error <= SLOW_ZONE)
    {
        // Approaching target — run at minimum duty
        duty = min_duty;
    }
    else
    {
        // Far from target — run at full duty
        duty = max_duty;
    }

    // Apply sign: positive error → forward, negative error → reverse
    float signed_duty = (error >= 0.0f) ? -duty : duty;

    if (duty == 0.0f)
    {
        motor->setStop();
        if (*active)
        {
            ESP_LOGI(tag, "[%s] Target reached — feedback:%.2f°  target:%.2f°",
                     tag, feedback, target);
            *active = false;
        }
    }
    else
    {
        motor->setDuty(signed_duty);
        *active = true;
        ESP_LOGI(tag, "[%s] target:%.2f°  feedback:%.2f°  error:%.2f°  duty:%.1f%%",
                 tag, target, feedback, error, signed_duty);
    }
}

static void moveJ3(float delta_angle)
{
    float target_angle = g_j3_current_angle + delta_angle;

    printf("[J3] Moving %.2f° from %.2f° → target %.2f°\n",
           delta_angle, g_j3_current_angle, target_angle);

    // Print constants so we can verify values
    ESP_LOGI("J3", "MAX_DUTY=%.4f  MIN_DUTY=%.4f  SLOW_ZONE=%.2f  STOP_ZONE=%.2f",
             MAX_DUTY_J3, MIN_DUTY_J3, SLOW_ZONE, STOP_ZONE);

    bool j3_active = true;
    int  iteration = 0;

    while (j3_active)
    {
        //xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
        MotorAngles feedback = g_enc->readAll();
        //xSemaphoreGive(g_i2c_mutex);
        float error = target_angle - feedback.j3;

        ESP_LOGI("J3", "[%03d] j3=%.2f° target=%.2f° error=%.2f° active=%d",
                 iteration++, feedback.j3, target_angle, error, j3_active);

        controlDcJoint("J3",
                       g_dc1,
                       target_angle,
                       feedback.j3,
                       MAX_DUTY_J3,
                       MIN_DUTY_J3,
                       &j3_active);

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    g_j3_current_angle = target_angle;
    printf("[J3] Reached %.2f° — ready for next command.\n", target_angle);
}



// =================================Move Steppers =================================
//J1 
float readEncoderRaw()
{
    uint16_t raw = 0;
    //xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
    g_enc->_enc_j1.read_RAWANGLE(raw);
    //  xSemaphoreGive(g_i2c_mutex);
    return (raw * 360.0f) / 4096.0f;
}

static inline float clampf(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}


//J2 (uP AND DOWN)


// =================================FUNCTIONS-STATE-INIT=================================
//================Init Hardware

void init_hardware()
{
    ESP_LOGI(TAG, "=== init_hardware START ===");

    // ── 1. Stepper motors J1 + J2 (no EN pin — always enabled via hardware) ─
    gpio_config_t step_cfg = {
        .pin_bit_mask = (1ULL << S1_STEP) | (1ULL << S1_DIR)
                      | (1ULL << S2_STEP) | (1ULL << S2_DIR),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&step_cfg));
    gpio_set_level((gpio_num_t)S1_STEP, 0);
    gpio_set_level((gpio_num_t)S1_DIR,  0);
    gpio_set_level((gpio_num_t)S2_STEP, 0);
    gpio_set_level((gpio_num_t)S2_DIR,  0);
    ESP_LOGI(TAG, "Stepper J1 — STEP:%d DIR:%d", S1_STEP, S1_DIR);
    ESP_LOGI(TAG, "Stepper J2 — STEP:%d DIR:%d", S2_STEP, S2_DIR);

    // ── 2. Limit switch J2 (active LOW, internal pull-up) ──────────────────
    gpio_config_t lim_cfg = {
        .pin_bit_mask = (1ULL << S2_LIMIT_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&lim_cfg));
    ESP_LOGI(TAG, "Limit switch J2 — GPIO:%d (pull-up, active LOW)", S2_LIMIT_PIN);

    // ── 5. Encoders (AS5600 J1 on I2C0, quadrature J3+J4) ──────────────────
    g_enc = new Encoders();
    g_enc->setup(
        ENC_I2C_SDA,  ENC_I2C_SCL,      // J1 AS5600  (I2C port 0)
        ENC2_I2C_SDA, ENC2_I2C_SCL,     // J2 unused  (kept for API compat)
        ENC_J3_PINS,  DEG_PER_EDGE_J3,  // J3 quadrature
        ENC_J4_PINS,  DEG_PER_EDGE_J4   // J4 quadrature
    );
    ESP_LOGI(TAG, "Encoders initialised");
    // ── 6. DC motors (HBridge) ────────────────────────────────────────────
    static TimerConfig dcTimerCfg = {
        .timer          = LEDC_TIMER_0,
        .frequency      = 25000,
        .bit_resolution = LEDC_TIMER_10_BIT,
        .mode           = LEDC_LOW_SPEED_MODE
    };
    g_dc1 = new HBridge();
    g_dc1->setup(DC1_PINS, DC1_CH, &dcTimerCfg, "J3");
    g_dc2 = new HBridge();
    g_dc2->setup(DC2_PINS, DC2_CH, &dcTimerCfg, "J4");
    ESP_LOGI(TAG, "HBridge J3/J4 ready");

    // ── 7. Stepper J1 ─────────────────────────────────────────────────────
    Smotor.setup(STEPPERPINS, 0, &s_stepper_timer, STEP_PER_REV);
    ESP_LOGI(TAG, "Stepper J1 ready (6400 microsteps/rev)");

    ESP_LOGI(TAG, "=== init_hardware DONE ===");
}
//================Wifi
static const char* reason_to_str(wifi_err_reason_t reason)
{
    switch (reason) {
        case WIFI_REASON_AUTH_FAIL: return "AUTH_FAIL";
        case WIFI_REASON_NO_AP_FOUND: return "NO_AP_FOUND";
        case WIFI_REASON_HANDSHAKE_TIMEOUT: return "HANDSHAKE_TIMEOUT";
        case WIFI_REASON_ASSOC_FAIL: return "ASSOC_FAIL";
        default: return "UNKNOWN";
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi started, connecting...");
        esp_wifi_connect();

    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {

        wifi_event_sta_disconnected_t* disconn =
            (wifi_event_sta_disconnected_t*) event_data;

        ESP_LOGE(TAG, "Disconnected! Reason: %d (%s)",
                 disconn->reason,
                 reason_to_str((wifi_err_reason_t)disconn->reason));

        if (s_retry_count < WIFI_MAX_RETRIES) {
            esp_wifi_connect();
            s_retry_count++;
            ESP_LOGW(TAG, "Retrying... (%d/%d)", s_retry_count, WIFI_MAX_RETRIES);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }

    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {

        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;

        ESP_LOGI(TAG, "GOT IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "MASK  : " IPSTR, IP2STR(&event->ip_info.netmask));
        ESP_LOGI(TAG, "GW    : " IPSTR, IP2STR(&event->ip_info.gw));

        s_retry_count = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}


bool wifi_init_sta()
{
    s_wifi_event_group = xEventGroupCreate();

    // Init TCP/IP stack and default STA netif
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // Default WiFi config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                         &wifi_event_handler, nullptr,
                                                         &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                         &wifi_event_handler, nullptr,
                                                         &instance_got_ip));

    // Set credentials
    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid,     WIFI_SSID,     sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, WIFI_PASSWORD, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to SSID: %s", WIFI_SSID);

    // Block until connected or failed
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE, pdFALSE,
                                            portMAX_DELAY);

    // Cleanup handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to %s", WIFI_SSID);
        return true;
    }

    ESP_LOGE(TAG, "Connection failed");
    return false;
}
//==============MQTT
MQTTClient g_mqtt;
void init_mqtt()
{
    ESP_LOGI(TAG, "=== init_mqtt START ===");

    // ── Register message callback BEFORE start() so no message is missed ──
    g_mqtt.setCallback([](std::string topic, std::string payload) {
        if (topic == TOPIC_MOTOR_ANGLES) {
            // Payload is stored raw here; parse_message() will decode it.
            // We only flag that a new message arrived — keeps the ISR-like
            // callback short and the parsing logic in one place.
            s_new_message = true;
            ESP_LOGI(TAG, "New message on '%s': %s", topic.c_str(), payload.c_str());
        }
    });

    // ── Initialise client (configures broker URI + credentials) ───────────
    g_mqtt.init(BROKER_URI, BROKER_USER, BROKER_PASS);

    // ── Start client (spawns internal ESP-IDF MQTT task, begins connecting) ─
    g_mqtt.start();

    // ── Block here until broker handshake completes ────────────────────────
    // Logs a warning every 2 s so you can see it waiting on the serial monitor.
    g_mqtt.waitUntilConnected(2000);

    ESP_LOGI(TAG, "=== init_mqtt DONE ===");
}
void subscribe_topics()
{
    ESP_LOGI(TAG, "=== subscribe_topics START ===");

    // The robot only needs to RECEIVE on TOPIC_MOTOR_ANGLES.
    // TOPIC_ENCODERS and TOPIC_SYSTEM_STATUS are publish-only (robot → PC),
    // so we do NOT subscribe to them here.
    g_mqtt.subscribe(TOPIC_MOTOR_ANGLES);

    ESP_LOGI(TAG, "Subscribed to: %s", TOPIC_MOTOR_ANGLES.c_str());
    ESP_LOGI(TAG, "=== subscribe_topics DONE ===");
}

// =================================FUNCTIONS-STATE-IDLE=================================
//================Parse Message
bool parse_message()
{
    ESP_LOGI(TAG, "Parsing payload: %s", s_payload.c_str());

    // ── Locate "color" ────────────────────────────────────────────────────
    // Simple manual parse — no cJSON heap allocation needed for this small payload.
    auto extract_string = [&](const std::string& key) -> std::string {
        // Looks for  "key":"value"
        std::string search = "\"" + key + "\":\"";
        size_t start = s_payload.find(search);
        if (start == std::string::npos) return "";
        start += search.size();
        size_t end = s_payload.find('"', start);
        if (end == std::string::npos) return "";
        return s_payload.substr(start, end - start);
    };

    auto extract_float = [&](const std::string& key) -> float {
        std::string search = "\"" + key + "\":";
        size_t start = s_payload.find(search);
        if (start == std::string::npos) return NAN;
        start += search.size();
        const char* ptr = s_payload.c_str() + start;
        char* end = nullptr;
        float val = strtof(ptr, &end);
        if (end == ptr) return NAN;   // no valid number found
        return val;
    };

    std::string color  = extract_string("color");
    float       j1     = extract_float("j1");
    float       j3     = extract_float("j3");

    // ── Validate ──────────────────────────────────────────────────────────
    if (color.empty()) {
        ESP_LOGW(TAG, "parse_message: missing or invalid 'color' field");
        return false;
    }
    if (std::isnan(j1)) {
        ESP_LOGW(TAG, "parse_message: missing or invalid 'j1' field");
        return false;
    }
    if (std::isnan(j3)) {
        ESP_LOGW(TAG, "parse_message: missing or invalid 'j3' field");
        return false;
    }

    // ── Commit ────────────────────────────────────────────────────────────
    s_color     = color;
    s_j1_target = j1;
    s_j3_target = j3;

    ESP_LOGI(TAG, "Parsed OK — color:%s  j1:%.2f°  j3:%.2f°",
             s_color.c_str(), s_j1_target, s_j3_target);
    return true;
}

// =================================FUNCTIONS-STATE-EXECUTE-CYCLE=================================
static volatile bool g_j1_done = false;
static volatile bool g_j3_done = false;

static void moveJ1ClosedLoop(float delta_angle)
{
    const float MAX_RPM        = 18.0f;
    const float MIN_RPM        = 2.0f;
    const float SLOWDOWN_ANGLE = 15.0f;
    const float STOP_DEADBAND  = 1.0f;

    float previous_raw_angle = readEncoderRaw();
    float current_position   = 0.0f;
    float target_position    = delta_angle;

    int64_t prev_time = esp_timer_get_time();
    const int64_t dt_us = 10000;

    ESP_LOGI(TAG, "[J1] start=%.2f° delta=%.2f°", previous_raw_angle, delta_angle);

    while (true)
    {
        int64_t now = esp_timer_get_time();
        if (now - prev_time < dt_us) continue;
        prev_time = now;

        // ── Update position ───────────────────────────────────────────────
        float raw  = readEncoderRaw();
        float step = raw - previous_raw_angle;
        if      (step >  180.0f) step -= 360.0f;
        else if (step < -180.0f) step += 360.0f;
        current_position   += step;
        previous_raw_angle  = raw;

        float error = target_position - current_position;

        // ── Stop ──────────────────────────────────────────────────────────
        if (fabsf(error) <= STOP_DEADBAND)
        {
            Smotor.setSpeed(0.0f);
            ESP_LOGI(TAG, "[J1] Done — pos=%.2f° target=%.2f°",
                     current_position, target_position);
            return;
        }

        // ── Speed: full or ramp ───────────────────────────────────────────
        float rpm;
        if (fabsf(error) >= SLOWDOWN_ANGLE)
        {
            rpm = MAX_RPM;                          // full speed
        }
        else
        {
            // Linear ramp: MAX_RPM → MIN_RPM as error shrinks to deadband
            float t = (fabsf(error) - STOP_DEADBAND) / (SLOWDOWN_ANGLE - STOP_DEADBAND);
            rpm = MIN_RPM + t * (MAX_RPM - MIN_RPM);
        }

        // Apply direction
        rpm = (error > 0.0f) ? rpm : -rpm;

        Smotor.setSpeed(-rpm);   // negate for wiring polarity

        ESP_LOGI(TAG, "[J1] pos=%.2f° err=%.2f° rpm=%.2f",
                 current_position, error, -rpm);
    }
}

static void moveJ1OpenLoop(float delta_angle)
{
    const float RPM_FAST       = 15.0f;
    const float RPM_SLOW       = 4.0f;
    const float SLOWDOWN_DEG   = 15.0f;   // switch to slow speed this many degrees before target

    const float DEG_PER_STEP   = 360.0f / (float)STEP_PER_REV;  // 0.05625°
    const float STEPS_PER_DEG  = (float)STEP_PER_REV / 360.0f;  // 17.78 steps/°

    if (fabsf(delta_angle) < DEG_PER_STEP) {
        ESP_LOGI(TAG, "[J1-OL] delta too small — skipping");
        return;
    }

    float direction = (delta_angle > 0.0f) ? 1.0f : -1.0f;
    float abs_delta = fabsf(delta_angle);

    // Split into fast and slow segments
    float fast_deg = 0.0f;
    float slow_deg = 0.0f;

    if (abs_delta > SLOWDOWN_DEG) {
        fast_deg = abs_delta - SLOWDOWN_DEG;
        slow_deg = SLOWDOWN_DEG;
    } else {
        slow_deg = abs_delta;
    }

    // Convert degrees to milliseconds at each speed
    // freq (Hz) = rpm * steps_per_rev / 60
    // time (ms) = (steps / freq) * 1000 = (deg * steps_per_deg / freq) * 1000
    float freq_fast = RPM_FAST * (float)STEP_PER_REV / 60.0f;
    float freq_slow = RPM_SLOW * (float)STEP_PER_REV / 60.0f;

    uint32_t fast_ms = (uint32_t)((fast_deg * STEPS_PER_DEG / freq_fast) * 1000.0f);
    uint32_t slow_ms = (uint32_t)((slow_deg * STEPS_PER_DEG / freq_slow) * 1000.0f);

    ESP_LOGI(TAG, "[J1-OL] delta=%.2f°  fast=%.2f°(%lums)  slow=%.2f°(%lums)",
             delta_angle, fast_deg, fast_ms, slow_deg, slow_ms);

    // ── Fast segment ─────────────────────────────────────────────────────
    if (fast_ms > 0) {
        Smotor.setSpeed(direction * RPM_FAST);
        vTaskDelay(pdMS_TO_TICKS(fast_ms));
    }

    // ── Slow segment ─────────────────────────────────────────────────────
    Smotor.setSpeed(direction * RPM_SLOW);
    vTaskDelay(pdMS_TO_TICKS(slow_ms));

    // ── Stop ─────────────────────────────────────────────────────────────
    Smotor.setSpeed(0.0f);

    ESP_LOGI(TAG, "[J1-OL] Done — estimated position delta=%.2f°", delta_angle);
}
//================Step 1: Move J1 & J3 to pickup position
static void taskJ1(void* arg)
{
    float delta = *(float*)arg;
    moveJ1OpenLoop(delta);
    g_j1_done = true;
    vTaskDelete(NULL);
}

static void taskJ3(void* arg)
{
    float delta = *(float*)arg;
    moveJ3(delta);
    g_j3_done = true;
    vTaskDelete(NULL);
}

static void moveJ1J3Parallel(float j1_delta, float j3_delta)
{
    g_j1_done = false;
    g_j3_done = false;

    static float j1_arg, j3_arg;
    j1_arg = j1_delta;
    j3_arg = j3_delta;

    ESP_LOGI(TAG, "[PARALLEL] J1=%.2f°  J3=%.2f°", j1_delta, j3_delta);

    xTaskCreate(taskJ1, "J1_task", 4096, &j1_arg, 5, NULL);
    xTaskCreate(taskJ3, "J3_task", 4096, &j3_arg, 5, NULL);

    while (!g_j1_done || !g_j3_done)
        vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "[PARALLEL] Both joints reached target.");
}



//================Step 2: Lower arm (J2)
//================Step 3: Close gripper
//================Step 4:  Raise arm (J2)
//================Step 5: Move J1 & J3 to place position
//================Step 6: Lower arm
//================Step 7: Open gripper
//================Step 8: Raise arm

//================Execute cycle (called from main loop when in STATE_EXECUTE_CYCLE)
/*
bool execute_cycle(std::string color, float j1_target, float j3_target)
{
    // Internal step tracker — persists across calls
    static uint8_t step = 0;

    switch (step)
    {
        //Step 1: Move J1 + J3 to pickup position (from MQTT) 
        case 0:
            if (move_to_pickup(j1_target, j3_target))
            {
                ESP_LOGI(TAG, "[CYCLE] Step 1 done — at pickup position");
                step = 1;   // advance to next step when ready
            }
            return false;

        //Step 2 … N: to be added 
        // case 1:
        //     ...

        default:
            // Cycle complete — reset step counter for next cycle
            ESP_LOGI(TAG, "[CYCLE] Cycle complete");
            step = 0;
            return true;
    }
}
*/
// =================================TEST================================
void test_encoders()
{
    // Read raw AS5600 value for diagnostics
    uint16_t raw = 0;
    g_enc->_enc_j1.read_RAWANGLE(raw);
    float raw_deg = static_cast<float>(raw) * (360.0f / 4096.0f);  // 12-bit encoder
    
    MotorAngles angles = g_enc->readAll();

    ESP_LOGI(TAG, "──────────────────────────────────────────");
    ESP_LOGI(TAG, "  AS5600 RAW       : %u (%.2f°)", raw, raw_deg);
    //ESP_LOGI(TAG, "  AS5600 offset    : %.2f°", g_encoders._offset_j1);
    ESP_LOGI(TAG, "  J1 (delta)       : %.2f°", angles.j1);    
    ESP_LOGI(TAG, "  J3 (quadrature)  : %.2f°", angles.j3);
    ESP_LOGI(TAG, "  J4 (quadrature)  : %.2f°", angles.j4);
    ESP_LOGI(TAG, "──────────────────────────────────────────");
}

void i2c_scan()
{
    ESP_LOGI(TAG, "=== I2C Scan on SDA:%d SCL:%d ===", ENC_I2C_SDA, ENC_I2C_SCL);

    // ── Re-init only if not already installed ─────────────────────────────
    // init_hardware() already called i2c_driver_initialize(), so we just
    // use the existing driver directly.
    int found = 0;
    for (uint8_t addr = 0x01; addr < 0x7F; addr++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);

        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "  Found device at 0x%02X", addr);
            found++;
        }
    }

    if (found == 0)
        ESP_LOGE(TAG, "  No I2C devices found — check wiring SDA:%d SCL:%d",
                 ENC_I2C_SDA, ENC_I2C_SCL);
    else
        ESP_LOGI(TAG, "  Scan complete — %d device(s) found", found);
}

//DC motor control test (J3)
static constexpr float DEAD_BAND_DEG   =  1.5f;   // stop when |error| < this
static constexpr float DECEL_START_DEG = 15.0f;   // begin ramp-down inside this window

void moveJ3ToAngle(HBridge&           bridge,std::function<float()> getAngle,float              targetDeg,uint32_t           pollMs = 20)
{
    ESP_LOGI(TAG, "moveJ3ToAngle → target: %.2f°", targetDeg);

    while (true)
    {
        float current = getAngle();
        float error   = targetDeg - current;

        ESP_LOGI(TAG, "  current: %.2f°  error: %.2f°", current, error);

        // ── 1. Stop condition ────────────────────────────────────────────────
        if (fabsf(error) < DEAD_BAND_DEG)
        {
            bridge.setStop();
            ESP_LOGI(TAG, "  Target reached — STOP");
            break;
        }

        // ── 2. Deceleration ramp ─────────────────────────────────────────────
        // Inside DECEL_START_DEG the duty scales linearly from DUTY_MAX → DUTY_MIN
        float abserr = fabsf(error);
        float duty;
        if (abserr >= DECEL_START_DEG)
        {
            duty = MAX_DUTY_J3;
        }
        else
        {
            // Linear interpolation: full speed → min speed over the window
            float t = abserr / DECEL_START_DEG;          // 1.0 far, 0.0 at target
            duty = MIN_DUTY_J3 + t * (MAX_DUTY_J3 - MIN_DUTY_J3);
        }

        // ── 3. Direction ─────────────────────────────────────────────────────
        // setDuty() in your HBridge: negative → FWD, positive → REV
        // Flip the sign below if your encoder counts in the opposite direction.
        float signedDuty = (error > 0) ? -duty : duty;
        bridge.setDuty(signedDuty);

        vTaskDelay(pdMS_TO_TICKS(pollMs));
    }
}

#include "driver/uart.h"
#define UART_PORT UART_NUM_0

void readAngleFromUART(float& result)
{
    char buf[32] = {};
    int  idx     = 0;

    // Flush stale bytes
    uint8_t flush;
    while (uart_read_bytes(UART_NUM_0, &flush, 1, pdMS_TO_TICKS(10)) > 0) {}

    ESP_LOGI(TAG, "Waiting for input...");

    while (idx < (int)sizeof(buf) - 1)
    {
        uint8_t c = 0;
        int got = uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(500)); // 500ms timeout

        if (got <= 0)
        {
            // No byte for 500ms — treat as end of input
            if (idx > 0) break;
            // Nothing typed yet — keep waiting
            continue;
        }

        if (c == '\n' || c == '\r')
        {
            if (idx > 0) break;
            continue;   // skip leading terminators
        }
        else if (c == 0x08 || c == 0x7F)  // backspace
        {
            if (idx > 0) idx--;
        }
        else
        {
            buf[idx++] = (char)c;
            printf("%c", c);
            fflush(stdout);
        }
    }

    buf[idx] = '\0';
    printf("\n");

    char* end = nullptr;
    float val = strtof(buf, &end);
    result = (end != buf) ? val : 0.0f;

    ESP_LOGI(TAG, "UART parsed: '%s' → %.4f", buf, result);
}

struct AS5600Reading {
    uint16_t raw;
    float angle_deg;
    bool magnet_detected;
    esp_err_t err;
};
/*
AS5600Reading read_as5600() {
    AS5600Reading result = {0, 0.0f, false, ESP_OK};
    
    // Leer ángulo RAW de 12 bits
    result.err = g_enc->_enc_j1.read_RAWANGLE(result.raw);
    if (result.err == ESP_OK) {
        result.angle_deg = (result.raw * 360.0f) / 4096.0f;
    } else {
        ESP_LOGE(TAG, "Error leyendo AS5600 RAWANGLE");
        return result;
    }
    
    // Leer status del imán
    AS5600_STATUS status = AS5600_STATUS(0);
    result.err = g_enc->_enc_j1.read_STATUS(status);
    result.magnet_detected = status.MagnetDetected();
    
    if (!result.magnet_detected) {
        ESP_LOGW(TAG, "Imán no detectado en AS5600");
    }
    
    return result;
}

static void debugJ1(void)
{
    ESP_LOGI(TAG, "=== DEBUG J1 START ===");

    // ── TEST 1: Can we stop the motor? ────────────────────────────────────
    ESP_LOGI(TAG, "[TEST 1] Calling setSpeed(0) — motor should NOT move");
    Smotor.setSpeed(0.0f);
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "[TEST 1] Done — did motor move? (check visually)");

    // ── TEST 2: Does the encoder change when motor is stopped? ────────────
    ESP_LOGI(TAG, "[TEST 2] Encoder stability — 10 readings with motor stopped");
    for (int i = 0; i < 10; i++) {
        float r = readEncoderRaw();
        ESP_LOGI(TAG, "  [%d] raw=%.3f°", i, r);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // ── TEST 3: Does setSpeed(X) actually spin, and setSpeed(0) stop it? ──
    ESP_LOGI(TAG, "[TEST 3] setSpeed(5) for 1s, then setSpeed(0)");
    Smotor.setSpeed(5.0f);
    vTaskDelay(pdMS_TO_TICKS(1000));
    Smotor.setSpeed(0.0f);
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "[TEST 3] Done");

    // ── TEST 4: Does the encoder track motion? ────────────────────────────
    ESP_LOGI(TAG, "[TEST 4] setSpeed(5) — watch encoder move for 2s");
    float before = readEncoderRaw();
    Smotor.setSpeed(5.0f);
    for (int i = 0; i < 20; i++) {
        float r = readEncoderRaw();
        ESP_LOGI(TAG, "  [%d] raw=%.3f°  delta=%.3f°", i, r, r - before);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    Smotor.setSpeed(0.0f);
    ESP_LOGI(TAG, "[TEST 4] Done — encoder should have changed");

    // ── TEST 5: Wraparound — spin past 0/360 boundary ─────────────────────
    ESP_LOGI(TAG, "[TEST 5] setSpeed(-5) for 2s — watch for wraparound");
    before = readEncoderRaw();
    Smotor.setSpeed(-5.0f);
    for (int i = 0; i < 20; i++) {
        float r = readEncoderRaw();
        ESP_LOGI(TAG, "  [%d] raw=%.3f°  delta=%.3f°", i, r, r - before);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    Smotor.setSpeed(0.0f);
    ESP_LOGI(TAG, "[TEST 5] Done");

    ESP_LOGI(TAG, "=== DEBUG J1 DONE ===");
}

static void debugJ1ClosedLoop(float delta_angle)
{
    const float Kp             = 0.4f;
    const float MAX_RPM        = 18.0f;
    const float MIN_RPM        = 3.0f;
    const float SLOWDOWN_ANGLE = 8.0f;
    const float STOP_DEADBAND  = 0.5f;
    const int   SETTLE_COUNT   = 5;

    float previous_raw_angle = readEncoderRaw();
    float current_position   = 0.0f;
    float target_position    = delta_angle;
    int   settle_counter     = 0;
    int   iteration          = 0;

    int64_t prev_time = esp_timer_get_time();
    const int64_t dt_us = 10000;

    ESP_LOGI(TAG, "=== CLOSED LOOP DEBUG ===");
    ESP_LOGI(TAG, "  initial_raw=%.3f°  target_delta=%.3f°", previous_raw_angle, delta_angle);

    while (true)
    {
        int64_t now = esp_timer_get_time();
        if (now - prev_time < dt_us) continue;
        prev_time = now;
        iteration++;

        // ── Encoder ──────────────────────────────────────────────────────
        float raw  = readEncoderRaw();
        float step = raw - previous_raw_angle;
        if      (step >  180.0f) step -= 360.0f;
        else if (step < -180.0f) step += 360.0f;
        current_position   += step;
        previous_raw_angle  = raw;

        float error = target_position - current_position;

        // ── Compute RPM (before any decisions) ───────────────────────────
        float rpm = 0.0f;
        if (fabsf(error) > STOP_DEADBAND)
        {
            rpm = Kp * error;
            rpm = clampf(rpm, -MAX_RPM, MAX_RPM);

            if (fabsf(error) < SLOWDOWN_ANGLE) {
                float t   = fabsf(error) / SLOWDOWN_ANGLE;
                float mag = MIN_RPM + t * (MAX_RPM - MIN_RPM);
                rpm = (rpm >= 0.0f) ? mag : -mag;
            }
        }

        float rpm_sent = -rpm;   // inverted as per your fix

        // ── Log EVERYTHING ───────────────────────────────────────────────
        ESP_LOGI(TAG, "[%03d] raw=%.3f° step=%.3f° pos=%.3f° target=%.3f° "
                      "error=%.3f° rpm_calc=%.3f rpm_sent=%.3f settle=%d",
                 iteration, raw, step, current_position, target_position,
                 error, rpm, rpm_sent, settle_counter);

        // ── Drive ────────────────────────────────────────────────────────
        Smotor.setSpeed(rpm_sent);

        // ── Settle check ─────────────────────────────────────────────────
        if (fabsf(error) <= STOP_DEADBAND) {
            settle_counter++;
            ESP_LOGI(TAG, "  >> Inside deadband — settle=%d/%d", settle_counter, SETTLE_COUNT);
            if (settle_counter >= SETTLE_COUNT) {
                Smotor.setSpeed(0.0f);
                ESP_LOGI(TAG, "  >> SETTLED — final pos=%.3f° target=%.3f°",
                         current_position, target_position);
                return;
            }
        } else {
            settle_counter = 0;
        }

        // ── Safety exit after 500 iterations (~5s) ───────────────────────
        if (iteration >= 500) {
            Smotor.setSpeed(0.0f);
            ESP_LOGI(TAG, "  >> TIMEOUT — pos=%.3f° target=%.3f° error=%.3f°",
                     current_position, target_position, error);
            return;
        }
    }
}

static void debugSpeedRange(void)
{
    ESP_LOGI(TAG, "=== SPEED RANGE TEST ===");

    float speeds[] = { 5.0f, 10.0f, 15.0f, 18.0f, 20.0f, 25.0f, 30.0f, -5.0f, -10.0f, -18.0f, -30.0f };
    int n = sizeof(speeds) / sizeof(speeds[0]);

    for (int i = 0; i < n; i++)
    {
        float spd = speeds[i];
        float before = readEncoderRaw();

        ESP_LOGI(TAG, "[TEST] setSpeed(%.1f) for 1s ...", spd);
        Smotor.setSpeed(spd);
        vTaskDelay(pdMS_TO_TICKS(1000));
        Smotor.setSpeed(0.0f);
        vTaskDelay(pdMS_TO_TICKS(500));   // coast to stop

        float after = readEncoderRaw();
        float moved = after - before;
        if      (moved >  180.0f) moved -= 360.0f;
        else if (moved < -180.0f) moved += 360.0f;

        ESP_LOGI(TAG, "  before=%.3f°  after=%.3f°  moved=%.3f°  %s",
                 before, after, moved,
                 fabsf(moved) < 0.5f ? "*** DID NOT MOVE ***" : "OK");

        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGI(TAG, "=== SPEED RANGE TEST DONE ===");
}

void debugUART(void)
{
    ESP_LOGI(TAG, "=== UART RAW BYTE DEBUG ===");
    ESP_LOGI(TAG, "Type something and press Enter...");

    for (int i = 0; i < 20; i++)   // capture up to 20 bytes
    {
        uint8_t c = 0;
        int got = uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(5000));
        if (got > 0) {
            ESP_LOGI(TAG, "  byte[%02d]: dec=%3d  hex=0x%02X  char='%c'",
                     i, c, c, (c >= 32 && c < 127) ? c : '?');
        } else {
            ESP_LOGW(TAG, "  byte[%02d]: TIMEOUT — no byte received", i);
            break;
        }
    }
}

static void testJ3EncoderWhileMoving(void)
{
    ESP_LOGI(TAG, "=== J3 ENCODER WHILE MOVING ===");

    // Spin J3 at full duty
    g_dc1->setDuty(MAX_DUTY_J3);

    for (int i = 0; i < 30; i++) {
        MotorAngles fb = g_enc->readAll();
        ESP_LOGI(TAG, "[%02d] j3=%.3f°", i, fb.j3);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    g_dc1->setStop();
    ESP_LOGI(TAG, "=== DONE ===");
}

static void testJ3Hardware(void)
{
    ESP_LOGI(TAG, "=== J3 HARDWARE TEST ===");

    // TEST 1: g_dc1 both directions
    ESP_LOGI(TAG, "[TEST 1] g_dc1 setDuty(0.8) for 2s");
    g_dc1->setDuty(0.8f);
    vTaskDelay(pdMS_TO_TICKS(2000));
    g_dc1->setStop();
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "[TEST 1] g_dc1 setDuty(-0.8) for 2s");
    g_dc1->setDuty(-0.8f);
    vTaskDelay(pdMS_TO_TICKS(2000));
    g_dc1->setStop();
    vTaskDelay(pdMS_TO_TICKS(1000));

    // TEST 2: g_dc2 both directions
    ESP_LOGI(TAG, "[TEST 2] g_dc2 setDuty(0.8) for 2s");
    g_dc2->setDuty(0.8f);
    vTaskDelay(pdMS_TO_TICKS(2000));
    g_dc2->setStop();
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "[TEST 2] g_dc2 setDuty(-0.8) for 2s");
    g_dc2->setDuty(-0.8f);
    vTaskDelay(pdMS_TO_TICKS(2000));
    g_dc2->setStop();
    vTaskDelay(pdMS_TO_TICKS(1000));

    // TEST 3: whichever moved J3 — check encoder tracks it
    ESP_LOGI(TAG, "[TEST 3] Encoder tracking — watch j3 while spinning");
    g_dc1->setDuty(0.8f);   // change to g_dc2 if that's the one that moved J3
    for (int i = 0; i < 20; i++) {
        MotorAngles fb = g_enc->readAll();
        ESP_LOGI(TAG, "  [%02d] j3=%.3f°  j4=%.3f°", i, fb.j3, fb.j4);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    g_dc1->setStop();

    ESP_LOGI(TAG, "=== J3 HARDWARE TEST DONE ===");
}
static void testJ3Pins(void)
{
    ESP_LOGI(TAG, "=== J3 PIN BRUTE FORCE TEST ===");
    ESP_LOGI(TAG, "DC1_PINS[0]=%d DC1_PINS[1]=%d", DC1_PINS[0], DC1_PINS[1]);
    ESP_LOGI(TAG, "DC2_PINS[0]=%d DC2_PINS[1]=%d", DC2_PINS[0], DC2_PINS[1]);
    ESP_LOGI(TAG, "DC1_CH[0]=%d DC1_CH[1]=%d", DC1_CH[0], DC1_CH[1]);
    ESP_LOGI(TAG, "DC2_CH[0]=%d DC2_CH[1]=%d", DC2_CH[0], DC2_CH[1]);

    // Manually toggle every DC pin so you can probe with a multimeter
    // or watch which wire on the H-bridge goes high
    uint8_t all_pins[] = { DC1_PINS[0], DC1_PINS[1], DC2_PINS[0], DC2_PINS[1] };
    const char* names[] = { "DC1_PINS[0]", "DC1_PINS[1]", "DC2_PINS[0]", "DC2_PINS[1]" };

    for (int i = 0; i < 4; i++)
    {
        ESP_LOGI(TAG, "[PIN TEST] Driving %s (GPIO %d) HIGH for 3s — does J3 move?",
                 names[i], all_pins[i]);

        gpio_set_direction((gpio_num_t)all_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)all_pins[i], 1);
        vTaskDelay(pdMS_TO_TICKS(3000));
        gpio_set_level((gpio_num_t)all_pins[i], 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "=== PIN TEST DONE ===");
}

static void testJ3DirectPWM(void)
{
    ESP_LOGI(TAG, "=== DIRECT PWM TEST ON GPIO 10/11 ===");

    // Bypass HBridge entirely — configure LEDC directly on DC1 pins
    ledc_timer_config_t t = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num       = LEDC_TIMER_3,        // fresh timer, no conflicts
        .freq_hz         = 1000,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&t);

    ledc_channel_config_t ch0 = {
        .gpio_num   = 10,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_6,           // fresh channels
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_3,
        .duty       = 512,                      // 50% of 1024
        .hpoint     = 0
    };
    ledc_channel_config(&ch0);

    ledc_channel_config_t ch1 = {
        .gpio_num   = 11,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_7,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_3,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&ch1);

    ESP_LOGI(TAG, "PWM on GPIO10 ch6 duty=512, GPIO11 ch7 duty=0 — J3 should spin for 3s");
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Reverse
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_7, 512);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_7);

    ESP_LOGI(TAG, "Reversed — J3 should spin other direction for 3s");
    vTaskDelay(pdMS_TO_TICKS(3000));

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_7, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_7);

    ESP_LOGI(TAG, "=== DIRECT PWM TEST DONE ===");
}
*/

// ===========================================================================
// =================================MAIN======================================
// ===========================================================================

extern "C" void app_main(void)
{
    state = STATE_INIT;

    while (true)
    {
        if (state == STATE_INIT)
        {
            // NVS flash is required by WiFi driver
            ESP_ERROR_CHECK(nvs_flash_init());

            if (!wifi_init_sta()) {
                ESP_LOGE("main", "WiFi failed — halting");
                return;
            }
            //open_gripper();
            init_mqtt();
            subscribe_topics();
            init_hardware();
            state = STATE_IDLE;
        }
        /*else if (state == STATE_IDLE)
        {
            if (s_new_message)
            {
                s_new_message = false;   // clear flag immediately before parsing
                if (parse_message())
                {
                    state = STATE_EXECUTE_CYCLE;
                }
                else
                {
                    ESP_LOGW(TAG, "Bad message ignored — staying IDLE");
                }
            }
        }
        else if (state == STATE_EXECUTE_CYCLE)
        {
            if (execute_cycle(s_color, s_j1_target, s_j3_target))
            {
                state = STATE_IDLE;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));   // yield to FreeRTOS — never busy-spin*/
    }
}

static EventGroupHandle_t s_ctrl_events = nullptr;
static portMUX_TYPE       s_angle_mux   = portMUX_INITIALIZER_UNLOCKED;



/*
extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1500));
    init_hardware();
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);

    g_j3_current_angle = g_enc->readAll().j3;
    g_dc1->setStop();

    ESP_LOGI(TAG, "=== J1+J3 PARALLEL TEST ===");
    ESP_LOGI(TAG, "Format: j1,j3  (e.g. '30,45')");

    while (true)
    {
        char buf[32] = {};
        int  idx     = 0;

        printf("\nEnter j1,j3 angles: ");
        fflush(stdout);

        while (idx == 0) {
            uint8_t c = 0;
            if (uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(10000)) > 0)
                buf[idx++] = (char)c;
        }

        while (idx < (int)sizeof(buf) - 1) {
            uint8_t c = 0;
            if (uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(500)) <= 0) break;
            if (c == '\r' || c == '\n') break;
            buf[idx++] = (char)c;
        }
        buf[idx] = '\0';

        char* sep = strchr(buf, ',');
        if (sep == nullptr) {
            ESP_LOGW(TAG, "Bad format — expected 'j1,j3' e.g. '30,45'");
            continue;
        }

        *sep = '\0';
        char* end1 = nullptr;
        char* end2 = nullptr;
        float j1 = strtof(buf,   &end1);
        float j3 = strtof(sep+1, &end2);

        if (end1 == buf || end2 == sep+1) {
            ESP_LOGW(TAG, "Parse failed — try again");
            continue;
        }

        ESP_LOGI(TAG, "Moving J1=%.2f°  J3=%.2f° in parallel...", j1, j3);
        moveJ1J3Parallel(j1, j3);
        ESP_LOGI(TAG, "Done. Ready for next command.");
    }
}*/

