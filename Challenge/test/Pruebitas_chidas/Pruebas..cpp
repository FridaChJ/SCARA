
/*
//================================================
//=========== Mover J2 ===========================
//================================================
extern "C" void app_main()
{
    // 1. Desactivar Watchdog y configurar hardware
    esp_task_wdt_deinit();
    init_hardware();

    // (Opcional) Puedes comentar esto si no necesitas WiFi ni MQTT por ahora
    // wifi_init_sta();
    // init_mqtt();
    // subscribe_topics();

    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "=== TEST INTERACTIVO EJE Z (LEAD SCREW) ===");
    ESP_LOGI(TAG, "Comandos disponibles:");
    ESP_LOGI(TAG, "  H      -> Ejecutar Z_home() (Subir al Switch)");
    ESP_LOGI(TAG, "  P      -> Ejecutar Z_pick() (Bajar al piso -91mm)");
    ESP_LOGI(TAG, "  -45.5  -> Bajar a una coordenada en mm");
    ESP_LOGI(TAG, "==========================================");

    // 2. Por seguridad, siempre hacemos Homing al encender la máquina
    Z_home();

    // Instalar el driver del UART0 para poder leer el teclado
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);


    // 3. Bucle infinito para recibir comandos por el Monitor Serial
    while (true)
    {
        char buf[32] = {};
        int  idx     = 0;

        printf("\nIngresa comando para J2 (Z): ");
        fflush(stdout);

        // Esperar el primer caracter
        while (idx == 0) {
            uint8_t c = 0;
            if (uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(10000)) > 0)
                buf[idx++] = (char)c;
        }

        // Leer el resto hasta presionar Enter (\r o \n)
        while (idx < (int)sizeof(buf) - 1) {
            uint8_t c = 0;
            if (uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(500)) <= 0) break;
            if (c == '\r' || c == '\n') break;
            buf[idx++] = (char)c;
        }
        buf[idx] = '\0';

        if (idx == 0) continue;

        // 4. Evaluar qué comando mandaste
        if (buf[0] == 'H' || buf[0] == 'h') 
        {
            ESP_LOGI("TEST", "Comando recibido: Homing");
            Z_home();
        } 
        else if (buf[0] == 'P' || buf[0] == 'p') 
        {
            ESP_LOGI("TEST", "Comando recibido: Pick (Ir al fondo)");
            Z_pick();
        } 
        else 
        {
            // Si no es una letra especial, asumimos que es un número flotante
            float target_mm = atof(buf);
            
            // Validar que no hayamos convertido pura basura a 0.0 sin querer
            if (target_mm == 0.0f && buf[0] != '0') {
                ESP_LOGW("TEST", "Comando no reconocido: %s", buf);
            } else {
                ESP_LOGI("TEST", "Comando recibido: Mover a %.2f mm", target_mm);
                moveJ2_Z_Axis(target_mm);
            }
        }
    }
}
*/



//=====================================================

/*
//================================================
//=========== Mover J1 y J3 ========================
//================================================
extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(10));
    init_hardware();
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);

    g_j3_current_angle = g_enc->readAll().j3;
    g_dc1->setStop();
    
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

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
} 
*/

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

        float rpm_sent = rpm;   // inverted as per your fix

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
/*
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
}*/


/*
float readEncoderRaw()
{
    uint16_t raw = 0;
    //xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
    g_enc->_enc_j1.read_RAWANGLE(raw);
    //  xSemaphoreGive(g_i2c_mutex);
    return (raw * 360.0f) / 4096.0f;
}
static void moveJ1ClosedLoop(float delta_angle)
{
    const float MAX_RPM        = 18.0f;
    const float MIN_RPM        = 2.0f;
    const float SLOWDOWN_ANGLE = 15.0f;
    const float STOP_DEADBAND  = 0.5f; // Lo bajé un poquito para más precisión

    float previous_raw_angle = readEncoderRaw();
    float current_position   = 0.0f;
    float target_position    = delta_angle;

    int64_t prev_time = esp_timer_get_time();
    const int64_t dt_us = 10000;

    ESP_LOGI("J1", "start=%.2f° delta=%.2f°", previous_raw_angle, delta_angle);

    while (true)
    {
        int64_t now = esp_timer_get_time();
        
        // CORRECCIÓN PARA EL WATCHDOG: En vez de un 'continue' salvaje, usamos delay.
        if (now - prev_time < dt_us) {
            vTaskDelay(pdMS_TO_TICKS(1)); // Deja respirar al ESP32
            continue;
        }
        
        prev_time = now;

        // ── (Aquí va todo tu cálculo de pasos y cruce por cero igualito) ──
        float raw  = readEncoderRaw();
        float step = raw - previous_raw_angle;
        if      (step >  180.0f) step -= 360.0f;
        else if (step < -180.0f) step += 360.0f;
        current_position   += step;
        previous_raw_angle  = raw;

        float error = target_position - current_position;

        // ── Stop ──
        if (fabsf(error) <= STOP_DEADBAND)
        {
            Smotor.setSpeed(0.0f);
            ESP_LOGI("J1", "Done — pos=%.2f° target=%.2f°", current_position, target_position);
            return; // ¡Sale de la función y permite que el código siga a otra cosa!
        }

        // ── Speed: full or ramp ──
        float rpm;
        if (fabsf(error) >= SLOWDOWN_ANGLE) {
            rpm = MAX_RPM;                          
        } else {
            float t = (fabsf(error) - STOP_DEADBAND) / (SLOWDOWN_ANGLE - STOP_DEADBAND);
            rpm = MIN_RPM + t * (MAX_RPM - MIN_RPM);
        }

        rpm = (error > 0.0f) ? rpm : -rpm;

        // IMPORTANTE: Checa si tu Smotor requiere conversión a Hz o si recibe RPM directo.
        Smotor.setSpeed(rpm);   

    }
}*/

// =================================TEST================================
/*void test_encoders()
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
*/