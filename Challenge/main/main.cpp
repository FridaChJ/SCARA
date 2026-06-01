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

// =================================GLOBALS=================================
static const char* TAG = "MAIN";
static void IRAM_ATTR j1_step_handler(void* arg);
typedef enum {
    STATE_INIT,
    STATE_IDLE,
    STATE_EXECUTE_CYCLE
} RobotState;

static RobotState state = STATE_INIT;
static volatile bool s_new_message = false;
static std::string   s_payload     = "";       // ← add this line
static std::string   s_color       = "";
static float         s_j1_target   = 0.0f;
static float         s_j3_target   = 0.0f;
//================Stepper J1
#define INTERRUPT_PIN   GPIO_NUM_1
#define STEP_PER_REV    200

static uint8_t    STEPPERPINS[2]   = {S1_STEP, S1_DIR};
static TimerConfig s_stepper_timer = {
    .timer          = LEDC_TIMER_1,
    .frequency      = 1000,
    .bit_resolution = LEDC_TIMER_10_BIT,
    .mode           = LEDC_LOW_SPEED_MODE
};
static Stepper    Smotor;
static SimpleGPIO s_interrupt_pin;

static volatile float s_j1_position  = 0.0f;
static volatile bool  s_j1_direction = true;

//================DC motors
static HBridge* g_dc1 = nullptr;   // J3
static HBridge* g_dc2 = nullptr;   // J4

//================Encoder pointer (matches old code pattern)
//static Encoders* g_enc = nullptr;


// =================================FUNCTIONS-STATE-INIT=================================
//================Init Hardware
Encoders g_encoders;   // global encoder instance used by init_hardware()
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

    // ── 3. DC motor PWM — LEDC timer 0 @ 20 kHz (J3, J4 H-bridges) ────────
    ledc_timer_config_t dc_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = 20000,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&dc_timer));

    auto init_dc_channel = [](uint8_t gpio, uint8_t ch) {
        ledc_channel_config_t cfg = {
            .gpio_num   = gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = (ledc_channel_t)ch,
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&cfg));
    };

    for (int i = 0; i < 2; i++) {
        init_dc_channel(DC1_PINS[i], DC1_CH[i]);   // J3
        init_dc_channel(DC2_PINS[i], DC2_CH[i]);   // J4
    }
    ESP_LOGI(TAG, "DC motors J3/J4 — PWM timer0 @ 20kHz, channels 0-3, duty=0");

    // ── 4. Gripper servo — LEDC timer 1 @ 50 Hz (separate timer, DC motors ─
    //       run at 20 kHz on timer 0 — servo needs its own timer)
    ledc_timer_config_t servo_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,   // 16383 steps — enough for µs servo control        .timer_num       = LEDC_TIMER_1,
        .freq_hz         = SERVO_FREQ_HZ,        // 50 Hz
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&servo_timer));

    ledc_channel_config_t servo_ch = {
        .gpio_num   = SERVO_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_4,            // channels 0-3 used by DC motors
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_1,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&servo_ch));
    ESP_LOGI(TAG, "Gripper servo — GPIO:%d LEDC timer1 @ 50Hz ch4", SERVO_PIN);

    // ── 5. Encoders (AS5600 J1 on I2C0, quadrature J3+J4) ──────────────────
    g_encoders.setup(
        ENC_I2C_SDA,  ENC_I2C_SCL,      // J1 AS5600  (I2C port 0)
        ENC2_I2C_SDA, ENC2_I2C_SCL,     // J2 unused  (kept for API compat)
        ENC_J3_PINS,  DEG_PER_EDGE_J3,  // J3 quadrature
        ENC_J4_PINS,  DEG_PER_EDGE_J4   // J4 quadrature
    );
    ESP_LOGI(TAG, "Encoders initialised");
    // ── 6. DC motors (HBridge) ────────────────────────────────────────────
    TimerConfig dcTimerCfg = {
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
    s_interrupt_pin.setup(INTERRUPT_PIN, GPIO_MODE_INPUT);
    s_interrupt_pin.addInterrupt(GPIO_INTR_POSEDGE, &j1_step_handler);
    ESP_LOGI(TAG, "Stepper J1 ready");

    ESP_LOGI(TAG, "=== init_hardware DONE ===");
}

//================MQTT
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
//================J1 step ISR 
static void IRAM_ATTR j1_step_handler(void* arg)
{
    float step_deg = 360.0f / STEP_PER_REV;
    if (s_j1_direction) s_j1_position += step_deg;
    else                s_j1_position -= step_deg;
    if (s_j1_position >= 360.0f) s_j1_position -= 360.0f;
    if (s_j1_position <    0.0f) s_j1_position += 360.0f;
}
//================Shortest angular path
static float ShortPath(float current, float target)
{
    float error = target - current;
    if (error >  180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;
    return error;
}
//Proportional duty with dead-band
static float computeDuty(float error, float maxDuty, float minDuty)
{
    float absErr = fabsf(error);
    float sign   = (error >= 0.0f) ? 1.0f : -1.0f;
    if (absErr <= STOP_ZONE) return 0.0f;
    if (absErr >= SLOW_ZONE) return sign * maxDuty;
    float fraction = (absErr - STOP_ZONE) / (SLOW_ZONE - STOP_ZONE);
    return sign * (minDuty + fraction * (maxDuty - minDuty));
}
//================Step 1: Move J1 & J3 to pickup position
bool move_to_pickup(float j1_target, float j3_target)
{
    //Read current positions
    MotorAngles feedback = g_encoders.readAll();
    float       j1_current = s_j1_position;   // stepper tracked by ISR
    float       j3_current = feedback.j3;     // quadrature encoder

    //J1 — Stepper
    bool j1_done = false;
    {
        float error = ShortPath(j1_current, j1_target);

        if (fabsf(error) <= 1.9f)
        {
            Smotor.setSpeed(0.0f);
            j1_done = true;
        }
        else
        {
            // Proportional speed, clamped to ±30 RPM
            float rpm = error;
            if (rpm >  30.0f) rpm =  30.0f;
            if (rpm < -30.0f) rpm = -30.0f;

            s_j1_direction = (rpm > 0.0f);
            Smotor.setSpeed(rpm);

            ESP_LOGD(TAG, "[PICKUP][J1] pos:%.2f°  target:%.2f°  error:%.2f°  rpm:%.1f",
                     j1_current, j1_target, error, rpm);
        }
    }

    //J3 — DC Motor
    bool j3_done = false;
    {
        float error = j3_target - j3_current;
        float duty  = computeDuty(error, MAX_DUTY_J3, MIN_DUTY_J3);

        if (duty == 0.0f)
        {
            g_dc1->setStop();
            j3_done = true;

            ESP_LOGD(TAG, "[PICKUP][J3] Target reached — pos:%.2f°  target:%.2f°",
                     j3_current, j3_target);
        }
        else
        {
            g_dc1->setDuty(duty);

            ESP_LOGD(TAG, "[PICKUP][J3] pos:%.2f°  target:%.2f°  error:%.2f°  duty:%.1f%%",
                     j3_current, j3_target, error, duty);
        }
    }

    //Both done? 
    if (j1_done && j3_done)
    {
        ESP_LOGI(TAG, "[PICKUP] Position reached — J1:%.2f°  J3:%.2f°",
                 j1_current, j3_current);
        return true;
    }

    return false;
}
//================Step 2: Lower arm (J2)
//================Step 3: Close gripper
//================Step 4:  Raise arm (J2)
//================Step 5: Move J1 & J3 to place position
//================Step 6: Lower arm
//================Step 7: Open gripper
//================Step 8: Raise arm

//================Execute cycle (called from main loop when in STATE_EXECUTE_CYCLE)
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


// =================================TEST================================
void test_encoders()
{
    MotorAngles angles = g_encoders.readAll();

    ESP_LOGI(TAG, "──────────────────────────────────────────");
    ESP_LOGI(TAG, "  J1 (AS5600)      : %.2f°", angles.j1);    ESP_LOGI(TAG, "  J3 (quadrature)  : %.2f°", angles.j3);
    ESP_LOGI(TAG, "  J4 (quadrature)  : %.2f°", angles.j4);
    ESP_LOGI(TAG, "──────────────────────────────────────────");
}

// ===========================================================================
// =================================MAIN======================================
// ===========================================================================

/*extern "C" void app_main(void)
{
    state = STATE_INIT;

    while (true)
    {
        if (state == STATE_INIT)
        {
            init_hardware();
            //open_gripper();
            init_mqtt();
            subscribe_topics();
            state = STATE_IDLE;
        }
        else if (state == STATE_IDLE)
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
        vTaskDelay(pdMS_TO_TICKS(10));   // yield to FreeRTOS — never busy-spin
    }
}*/
extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1500));
    init_hardware();

    ESP_LOGI(TAG, "=== ENCODER TEST MODE — move joints by hand ===");
    ESP_LOGI(TAG, "=== Reading every 500 ms ===");

    while (true)
    {
        test_encoders();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}