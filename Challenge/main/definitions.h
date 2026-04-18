#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <string>
#include <cstring>
#include <cstdlib>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"


//=========================== Wifi Config ===============================
constexpr const char* WIFI_SSID     = "Friii";      // WiFi ID into which the ESP is going to connect
constexpr const char* WIFI_PASSWORD = "12345678";   // WiFi password into which the ESP is going to connect

//=========================== MQTT Config ===============================
static const std::string BROKER_URI         = "mqtt://10.48.231.132"; // MQTT broker URI
static const std::string BROKER_USER        = "";                     // MQTT broker username
static const std::string BROKER_PASS        = "";                     // MQTT broker password
static const std::string TOPIC_MOTOR_ANGLES = "robot/motor_angles";   // Topic that receives angles from PC
static const std::string TOPIC_ENCODERS     = "robot/encoders";       // Topic that sends encoder readings to PC

//========================= Pin Definitions ===============================
// Stepper motors

constexpr int S2_STEP = 4,  S2_DIR = 5, S2_EN = 6;          // Joint 2 stepper motor pins
constexpr int S2_LIMIT_PIN = 7;   // GPIO for J2 limit switch (active LOW, internal pull-up)

// DC motors
static uint8_t DC1_PINS[2] = {10, 11};  // Joint 3 H-bridge pins
static uint8_t DC1_CH[2]   = {0,  1};   // Joint 3 PWM channels
static uint8_t DC2_PINS[2] = {12, 13};  // Joint 4 H-bridge pins
static uint8_t DC2_CH[2]   = {2,  3};   // Joint 4 PWM channels

//====== Encoders
// Absolute encoders (AS5600)
constexpr gpio_num_t ENC_I2C_SDA  = GPIO_NUM_8,  ENC_I2C_SCL  = GPIO_NUM_9;   // J1 I2C encoder pins
constexpr gpio_num_t ENC2_I2C_SDA = GPIO_NUM_46, ENC2_I2C_SCL = GPIO_NUM_48;  // (unused for J2 — J2 uses step math)

// Quadrature encoders
static uint8_t ENC_J3_PINS[2] = {14,  15};   // Joint 3 quadrature encoder pins
static uint8_t ENC_J4_PINS[2] = {36, 39};  // Joint 4 quadrature encoder pins
constexpr float DEG_PER_EDGE_J3 = 0.1696f; // Degrees per encoder edge for joint 3
constexpr float DEG_PER_EDGE_J4 = 0.1098f; // Degrees per encoder edge for joint 4

//============================ PID Gains ===============================
constexpr float J1_KP = 1.0f, J1_KI = 0.0f, J1_KD = 0.0f; // Joint 1 PID gains
constexpr float J2_KP = 1.0f, J2_KI = 0.0f, J2_KD = 0.0f; // Joint 2 PID gains
constexpr float J3_KP = 1.0f, J3_KI = 0.0f, J3_KD = 0.0f; // Joint 3 PID gains
constexpr float J4_KP = 1.0f, J4_KI = 0.0f, J4_KD = 0.0f; // Joint 4 PID gains

//============================ Motor Limits ===============================
constexpr float MAX_DUTY = 60.0f; // Maximum duty cycle for DC motors (0–100%)

//============================ Event Bits ===============================
// WiFi
#define TARGET_READY_BIT   BIT0  // Set when a new target is received from MQTT
#define WIFI_CONNECTED_BIT BIT2  // Set when WiFi is connected
#define WIFI_FAIL_BIT      BIT3  // Set when WiFi connection fails
// Hardware init flags
#define HW_FAIL_BIT        BIT5  // Set if hardware initialization fails
#define HW_READY_BIT       BIT4  // Set when hardware initialization is successful

#endif // DEFINITIONS_H