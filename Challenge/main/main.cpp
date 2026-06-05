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
#include "Gripper.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include <cmath>
#include <vector>
#include <sstream>
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

#include "driver/uart.h"
#define UART_PORT UART_NUM_0


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

//================Stepper J2 (Z-Axis / Lead Screw)
static TimerConfig timer_z = {
    .timer          = LEDC_TIMER_2, // Usamos el Timer 2 para no chocar con J1 y J3
    .frequency      = 10000,
    .bit_resolution = LEDC_TIMER_11_BIT,
    .mode           = LEDC_LOW_SPEED_MODE
};
static Stepper Zmotor;
static Gripper g_gripper;
static SimpleGPIO limit_switch;
static SimpleGPIO interrupt_pin;

// Parámetros Mecánicos J2
constexpr int32_t J2_STEPS_PER_REV = 1600;
constexpr float J2_MM_PER_REV = 8.0f;
constexpr float J2_MM_PER_STEP = J2_MM_PER_REV / (float)J2_STEPS_PER_REV;
constexpr float Z_LIMIT_TOP = 0.0f;       // HOME
constexpr float Z_LIMIT_BOTTOM = -90.0f;  // PISO

// Odometría Volátil (ISR)
volatile int32_t current_z_steps = 0;
volatile bool current_z_direction = true;
volatile bool is_homing = false;
float current_z_position = 0.0f;

//================DC motors
static HBridge* g_dc1 = nullptr;   // J3
static HBridge* g_dc2 = nullptr;   // J4

//================Encoder pointer (matches old code pattern)
//static Encoders* g_enc = nullptr;

static float computeDuty(float error, float maxDuty, float minDuty);
static float clampf(float value, float min_value, float max_value);
//static SemaphoreHandle_t g_i2c_mutex = nullptr;

//=============Handlers para interrupciones del stepper en Z================
void IRAM_ATTR step_handler_z(void *arg) {
    if (current_z_direction) current_z_steps++;
    else current_z_steps--;
}

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

static inline float clampf(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}


//J2 (uP AND DOWN)
//J2 (UP AND DOWN)
bool moveJ2_Z_Axis(float target_mm) {
    if (!is_homing) {
        target_mm = clampf(target_mm, Z_LIMIT_BOTTOM, Z_LIMIT_TOP);
    }
    
    ESP_LOGI("J2_Z", "Moviendo eje Z a %.2f mm", target_mm);

    const float Kp = 12.0f;
    const float MAX_RPM = 150.0f;
    const float SLOWDOWN_DIST = 3.0f;
    const float STOP_DEADBAND = 0.05f;
    const float MIN_STABLE_RPM = 12.0f;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10)); 

        current_z_position = current_z_steps * J2_MM_PER_STEP;
        
        bool switch_is_active = (limit_switch.get() == 0); 
        float error = target_mm - current_z_position;

        // ── MANEJO DEL LIMIT SWITCH (Polling Seguro) ──
        if (switch_is_active) {
            if (is_homing) {
                Zmotor.setSpeed(0.0f);
                current_z_steps = 0;
                current_z_position = 0.0f;
                is_homing = false;
                ESP_LOGI("J2_Z", "¡Homing completado! Z = 0.0 mm");
                return true; 
            } 
            else if (error > 0) {
                Zmotor.setSpeed(0.0f);
                ESP_LOGW("J2_Z", "Alerta: Switch presionado. Subida bloqueada.");
                return false; 
            }
        }

        // ── Condición de Paro Normal ──
        if (!is_homing && fabsf(error) <= STOP_DEADBAND) {
            Zmotor.setSpeed(0.0f);
            ESP_LOGI("J2_Z", "Target alcanzado: %.2f mm", current_z_position);
            return true;
        }

        // ── Cálculo de Velocidad ──
        float u = Kp * error;
        if (fabsf(error) < SLOWDOWN_DIST) u *= fabsf(error) / SLOWDOWN_DIST;
        u = clampf(u, -MAX_RPM, MAX_RPM);
        
        if (fabsf(u) < MIN_STABLE_RPM) {
            u = (u > 0) ? MIN_STABLE_RPM : -MIN_STABLE_RPM;
        }

        current_z_direction = (u > 0);
        Zmotor.setSpeed(u);
    }
}

// Macros rápidas para facilitar tu programación SCARA
void Z_home() {
    is_homing = true;
    moveJ2_Z_Axis(90.0f); // Forzar la subida al origen
}

void Z_pick() {
    moveJ2_Z_Axis(Z_LIMIT_BOTTOM); // Ir al piso
}

// =================================FUNCTIONS-STATE-INIT=================================
//================Init Hardware

void init_hardware()
{
    gpio_install_isr_service(0); 
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

    // ── 8 Stepper J2 (Z-Axis Lead Screw) ──────────────────────────────
    static uint8_t J2_PINS[2] = {S2_STEP, S2_DIR};
    Zmotor.setup(J2_PINS, 1, &timer_z, J2_STEPS_PER_REV);
    gpio_pulldown_dis((gpio_num_t)INTERRUPT_PIN);
    gpio_pulldown_dis((gpio_num_t)S2_LIMIT_PIN);
    
    interrupt_pin.setup(INTERRUPT_PIN, GPIO_MODE_INPUT);
    interrupt_pin.addInterrupt(GPIO_INTR_POSEDGE, step_handler_z);

    limit_switch.setup(S2_LIMIT_PIN, GPIO_MODE_INPUT);

    // <-- AQUÍ borramos el limit_switch.addInterrupt(...)
    ESP_LOGI(TAG, "Stepper J2 (Z-Axis) ready");

    // ── 9. Gripper servo (LEDC timer 3, independent of motors) ─────────────
    ESP_ERROR_CHECK(g_gripper.begin());   // configures LEDC, boots to CLOSED
    ESP_LOGI(TAG, "Gripper ready");

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

    ESP_LOGE(TAG, "Connection failed after %d retries", WIFI_MAX_RETRIES);
    return false;
}
//==============MQTT
MQTTClient g_mqtt;
bool init_mqtt()
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
    if (!g_mqtt.waitUntilConnected(10000, 2000)) {
        ESP_LOGE(TAG, "Unable to connect to MQTT broker at %s", BROKER_URI.c_str());
        g_mqtt.stop();
        return false;
    }

    ESP_LOGI(TAG, "=== init_mqtt DONE ===");
    return true;
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

// =================================FUNCTIONS-STATE-EXECUTE-CYCLE=================================
static volatile bool g_j1_done = false;
static volatile bool g_j3_done = false;





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



static EventGroupHandle_t s_ctrl_events = nullptr;
static portMUX_TYPE       s_angle_mux   = portMUX_INITIALIZER_UNLOCKED;


//=======================================================================
//=======================================================================
//=========================Fin de Pruebas================================
// ============================MAIN======================================
// ======================================================================

static const char* MAIN_TAG = "MAIN_SCARA";

// ── Variable de Estado Global (Totalmente libre de conflictos y MODIFICABLE) ──
static RobotState g_scara_active_state = STATE_INIT;

// ── Variables del Mensaje MQTT / Serial ──
static std::string s_last_topic = "";
static std::string s_last_payload = "";

// ── Variables de Movimiento Incremental (Deltas de movimiento) ──
float delta_j1 = 0.0f;
float delta_j2 = 0.0f; // Eje Z
float delta_j3 = 0.0f;
float delta_j4 = 0.0f;


// =============================================================================
// UTILIDAD: Función para partir el CSV "10.5,20.0" en un vector de floats
// =============================================================================
std::vector<float> parseCSV(const std::string& payload) {
    std::vector<float> values;
    std::stringstream ss(payload);
    std::string item;
    while (std::getline(ss, item, ',')) {
        // strtof convierte el string a float de manera segura sin usar try-catch
        float val = strtof(item.c_str(), nullptr);
        values.push_back(val);
    }
    return values;
}

// =============================================================================
// CONTROL DE MOVIMIENTO: Ejecución en paralelo por incrementos (Deltas)
// =============================================================================
static void moveJ1J3Parallel(float j1_delta, float j3_delta)
{
    g_j1_done = false;
    g_j3_done = false;

    // Variables estáticas para pasar referencias seguras a las tareas de FreeRTOS
    static float j1_arg, j3_arg;
    j1_arg = j1_delta;
    j3_arg = j3_delta;

    ESP_LOGI(MAIN_TAG, "[PARALLEL] Aplicando deltas -> J1: %+.2f° | J3: %+.2f°", j1_delta, j3_delta);

    // Se crean las tareas de movimiento (deben estar declaradas arriba en tu archivo)
    xTaskCreate(taskJ1, "J1_task", 4096, &j1_arg, 5, NULL);
    xTaskCreate(taskJ3, "J3_task", 4096, &j3_arg, 5, NULL);

    // Espera bloqueante: detiene el ciclo de la FSM hasta que ambos motores terminen
    while (!g_j1_done || !g_j3_done) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(MAIN_TAG, "[PARALLEL] Ambos ejes terminaron su movimiento relativo.");
}

void MoverJ1_J3(float j1, float j3) {
    moveJ1J3Parallel(j1, j3);
}

void MoverManualTodos(float j1, float j2, float j3, float j4) {
    ESP_LOGI(TAG, "Manual Incremental -> J1:%+.1f, J2(Z):%+.1f, J3:%+.1f, J4:%+.1f", j1, j2, j3, j4);
    moveJ1J3Parallel(j1, j3);
    moveJ2_Z_Axis(j2); // Asumiendo que tu eje Z también procesa el delta de entrada
}

// =============================================================================
// APP MAIN DEFINITIVO
// =============================================================================
extern "C" void app_main(void)
{
    esp_task_wdt_deinit();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    init_hardware();
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);

    g_gripper.begin(); 

    // Configuración de red y MQTT (Descoméntalo cuando uses WiFi)
    
    if (!wifi_init_sta()) {
        ESP_LOGE(MAIN_TAG, "WiFi initialization failed after %d retries; aborting startup.", WIFI_MAX_RETRIES);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(MAIN_TAG, "Using MQTT broker URI: %s", BROKER_URI.c_str());
    if (!init_mqtt()) {
        ESP_LOGE(MAIN_TAG, "MQTT initialization failed; aborting startup.");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    g_mqtt.subscribe("robot/targets");
    g_mqtt.subscribe("robot/encoders"); 
    g_mqtt.subscribe("scara/manual");
    g_mqtt.subscribe("scara/auto");
    

    ESP_LOGI(MAIN_TAG, "==========================================");
    ESP_LOGI(MAIN_TAG, " SISTEMA INCREMENTAL LISTO PARA PRUEBAS");
    ESP_LOGI(MAIN_TAG, "==========================================");

    static char uart_buf[64];
    static int u_idx = 0;

    while (1)
    {
        // ── PUERTA TRASERA SERIAL DE PRUEBAS ──
        uint8_t c;
        if (uart_read_bytes(UART_NUM_0, &c, 1, 0) > 0) {
            if (c == '\n' || c == '\r') {
                uart_buf[u_idx] = '\0';
                std::string cmd(uart_buf);
                
                if (cmd.find("AUTO:") == 0) {
                    s_last_topic = "scara/auto";
                    s_last_payload = cmd.substr(5);
                    s_new_message = true;
                } else if (cmd.find("MANUAL:") == 0) {
                    s_last_topic = "scara/manual";
                    s_last_payload = cmd.substr(7);
                    s_new_message = true;
                }
                u_idx = 0; 
            } else if (u_idx < sizeof(uart_buf) - 1) {
                uart_buf[u_idx++] = (char)c;
            }
        }

        // ── MÁQUINA DE ESTADOS FINITA (FSM) ──
        switch (g_scara_active_state) 
        {
            case STATE_INIT:
                ESP_LOGI(MAIN_TAG, "[ESTADO] STATE_INIT -> Homing de seguridad...");
                Z_home();             
                g_gripper.open();     
                
                g_mqtt.publish("scara/status", "HOME_READY");
                ESP_LOGI(MAIN_TAG, "[SISTEMA] Home completado de inicio. Pasando a IDLE.");
                g_scara_active_state = STATE_IDLE; // ASIGNACIÓN LIMPIA
                break;

            case STATE_IDLE: {
                std::string payload;

                if (g_mqtt.readMessage("scara/manual", payload, 0)) {
                    cJSON* root = cJSON_Parse(payload.c_str());
                    if (root) {
                        cJSON* motor = cJSON_GetObjectItem(root, "motor");
                        cJSON* angle = cJSON_GetObjectItem(root, "angle");

                        if (motor && angle && cJSON_IsString(motor) && cJSON_IsNumber(angle)) {
                            float val        = (float)angle->valuedouble;
                            std::string name = motor->valuestring;

                            delta_j1 = delta_j2 = delta_j3 = delta_j4 = 0.0f;

                            if      (name == "Base")   delta_j1 = val;
                            else if (name == "Height") delta_j2 = val;
                            else if (name == "J3")     delta_j3 = val;
                            else if (name == "J4")     delta_j4 = val;
                            else ESP_LOGW(MAIN_TAG, "Motor desconocido: %s", name.c_str());

                            ESP_LOGI(MAIN_TAG, "JSON manual: motor=%s angle=%.2f", name.c_str(), val);
                            g_scara_active_state = STATE_MANUAL_MOVE;
                        } else {
                            ESP_LOGW(MAIN_TAG, "Error: JSON manual incompleto!");
                        }
                        cJSON_Delete(root);
                    } else {
                        ESP_LOGW(MAIN_TAG, "Error: JSON manual invalido!");
                    }
                }
                else if (g_mqtt.readMessage("scara/auto", payload, 0)) {
                    float v1 = 0, v3 = 0;
                    int convertidos = sscanf(payload.c_str(), "%f,%f", &v1, &v3);

                    if (convertidos == 2) {
                        delta_j1 = v1;
                        delta_j3 = v3;
                        g_scara_active_state = STATE_AUTO_PICK;
                    } else {
                        ESP_LOGW(MAIN_TAG, "Error: Cadena automática inválida!");
                    }
                }
                break;
            }
            case STATE_MANUAL_MOVE:
                ESP_LOGI(MAIN_TAG, "[ESTADO] STATE_MANUAL_MOVE");
                MoverManualTodos(delta_j1, delta_j2, delta_j3, delta_j4);
                
                g_mqtt.publish("scara/status", "MANUAL_DONE");
                g_scara_active_state = STATE_IDLE; 
                break;

            case STATE_AUTO_PICK:
                ESP_LOGI(MAIN_TAG, "[ESTADO] STATE_AUTO_PICK -> Moviendo delta hacia la pieza");
                MoverJ1_J3(delta_j1, delta_j3);
                
                Z_pick();
                g_gripper.close();
                vTaskDelay(pdMS_TO_TICKS(500)); 
                Z_home();

                g_mqtt.publish("scara/status", "LISTO_PARA_PLACE");
                ESP_LOGI(MAIN_TAG, "[SISTEMA] Pieza sujeta. Esperando delta de destino...");
                g_scara_active_state = STATE_AUTO_WAIT_PLACE;
                break;

            case STATE_AUTO_WAIT_PLACE: {
                std::string payload;
                if (g_mqtt.readMessage("scara/auto", payload, 0)) {
                    float v1 = 0, v3 = 0;
                    int convertidos = sscanf(payload.c_str(), "%f,%f", &v1, &v3);

                    if (convertidos == 2) {
                        delta_j1 = v1;
                        delta_j3 = v3;
                        g_scara_active_state = STATE_AUTO_PLACE;
                    } else {
                        ESP_LOGW(MAIN_TAG, "Error: Cadena de place inválida!");
                    }
                }
                break;
            }

            case STATE_AUTO_PLACE:
                ESP_LOGI(MAIN_TAG, "[ESTADO] STATE_AUTO_PLACE -> Moviendo delta hacia descarga");
                MoverJ1_J3(delta_j1, delta_j3);
                
                Z_pick();
                g_gripper.open();
                vTaskDelay(pdMS_TO_TICKS(500)); 
                Z_home();

                g_mqtt.publish("scara/status", "HOME_READY");
                ESP_LOGI(MAIN_TAG, "[SISTEMA] Ciclo Pick & Place terminado de manera incremental.");
                g_scara_active_state = STATE_IDLE; 
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}