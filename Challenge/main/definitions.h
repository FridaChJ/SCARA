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
static const std::string BROKER_URI         = "mqtt://10.48.4.76"; // MQTT broker URI
static const std::string BROKER_USER        = "";                     // MQTT broker username
static const std::string BROKER_PASS        = "";                     // MQTT broker password
static const std::string TOPIC_MOTOR_ANGLES = "angles/response";   // Topic that receives angles from PC
static const std::string TOPIC_ENCODERS     = "robot/encoders";       // Topic that sends encoder readings to PC
static const std::string TOPIC_SYSTEM_STATUS = "system/status";     // Topic that sends system status updates to PC
//========================= Pin Definitions ===============================

// Stepper motors
constexpr int S1_STEP = 0, S1_DIR = 0;                  // Joint 1 stepper motor pins (EN always on, not connected)
constexpr int S2_STEP = 4,   S2_DIR = 5;                    // Joint 2 stepper motor pins (EN always on, not connected)
constexpr int S2_LIMIT_PIN = 7;                              // GPIO for J2 limit switch (active LOW, internal pull-up)

// DC motors
static uint8_t DC1_PINS[2] = {10, 11};  // Joint 3 H-bridge pins
static uint8_t DC1_CH[2]   = {0,  1};   // Joint 3 PWM channels
static uint8_t DC2_PINS[2] = {12, 13};  // Joint 4 H-bridge pins
static uint8_t DC2_CH[2]   = {2,  3};   // Joint 4 PWM channels

// Gripper servo
constexpr int SERVO_PIN     = 0;                           // Gripper servo PWM pin
constexpr int SERVO_FREQ_HZ = 50;                            // Standard servo: 50 Hz / 20 ms period
constexpr int SERVO_OPEN_US  = 1000;                         // Pulse width for open  position (µs) — tune to your servo
constexpr int SERVO_CLOSE_US = 2000;                         // Pulse width for closed position (µs) — tune to your servo

//====== Encoders
// Absolute encoders (AS5600)
constexpr gpio_num_t ENC_I2C_SDA  = GPIO_NUM_8,  ENC_I2C_SCL  = GPIO_NUM_9;   // J1 I2C encoder pins
constexpr gpio_num_t ENC2_I2C_SDA = GPIO_NUM_46, ENC2_I2C_SCL = GPIO_NUM_48;   //(unused — J2 uses step math)

// Quadrature encoders Q
static uint8_t ENC_J3_PINS[2] = {14,  15};   // Joint 3 quadrature encoder pins
static uint8_t ENC_J4_PINS[2] = {36,  39};   // Joint 4 quadrature encoder pins
constexpr float DEG_PER_EDGE_J3 = 0.1696f;   // Degrees per encoder edge for joint 3
constexpr float DEG_PER_EDGE_J4 = 0.1098f;   // Degrees per encoder edge for joint 4

//============================ PID Gains ===============================
constexpr float J1_KP = 1.0f, J1_KI = 0.0f, J1_KD = 0.0f; // Joint 1 PID gains
constexpr float J2_KP = 1.0f, J2_KI = 0.0f, J2_KD = 0.0f; // Joint 2 PID gains
constexpr float J3_KP = 1.0f, J3_KI = 0.0f, J3_KD = 0.0f; // Joint 3 PID gains
constexpr float J4_KP = 1.0f, J4_KI = 0.0f, J4_KD = 0.0f; // Joint 4 PID gains

//============================ Motor Limits ===============================
static constexpr float MAX_DUTY_J3 = 60.0f;
static constexpr float MIN_DUTY_J3 = 20.0f;
static constexpr float MAX_DUTY_J4 = 60.0f;
static constexpr float MIN_DUTY_J4 = 20.0f;
static constexpr float SLOW_ZONE   = 15.0f;
static constexpr float STOP_ZONE   =  1.5f;
//============================ Event Bits ===============================
// WiFi
#define TARGET_READY_BIT   BIT0  // Set when a new target is received from MQTT
#define WIFI_CONNECTED_BIT BIT2  // Set when WiFi is connected
#define WIFI_FAIL_BIT      BIT3  // Set when WiFi connection fails
// Hardware init flags
#define HW_FAIL_BIT        BIT5  // Set if hardware initialization fails
#define HW_READY_BIT       BIT4  // Set when hardware initialization is successful

#endif // DEFINITIONS_H