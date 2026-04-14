// =============================================================================
// HBridge.cpp
// Authors     : Frida Sophia Chavez Juarez
// Mentor      : Oscar Vargas Perez
// Last updated: 2026-04-13
//
// Change: added ESP_LOGI calls in setDuty() and setStop() so the PWM values
// sent to each channel are visible in the terminal when no hardware is connected.
// =============================================================================

#include "HBridge.h"
#include "esp_log.h"

static const char* TAG = "HBridge";

HBridge::HBridge() { _label[0] = '\0'; _stopped = false; }

void HBridge::setup(uint8_t pin[2], uint8_t ch[2], TimerConfig* config,
                    const char* jointLabel)
{
    for (size_t i = 0; i < 2; i++)
        pwm[i].setup(pin[i], ch[i], config);

    ESP_LOGI(TAG, "setup — pin A:%d (ch %d)  pin B:%d (ch %d)",
             pin[0], ch[0], pin[1], ch[1]);
}

void HBridge::setDuty(float duty)
{
    _stopped = false; 
    if (duty < 0) {
        // Negative duty → forward: channel A drives, channel B = 0
        pwm[0].setDuty(-duty);
        pwm[1].setDuty(0.0f);
        ESP_LOGI(TAG, "setDuty → dir:FWD  chA:%.1f%%  chB:0%%", -duty);
    } else {
        // Positive duty → reverse: channel B drives, channel A = 0
        pwm[0].setDuty(0.0f);
        pwm[1].setDuty(duty);
        ESP_LOGI(TAG, "setDuty → dir:REV  chA:0%%  chB:%.1f%%", duty);
    }
}

void HBridge::setStop()
{
    if (_stopped) return;    // ADD THIS — skip if already stopped
    _stopped = true;
    pwm[0].setDuty(0.0f);
    pwm[1].setDuty(0.0f);
    ESP_LOGI(TAG, "[%s] STOP", _label);
}