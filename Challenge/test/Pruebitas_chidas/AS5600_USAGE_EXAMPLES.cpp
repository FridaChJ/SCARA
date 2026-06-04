// =============================================================================
// EJEMPLO: Cómo integrar tu lógica de AS5600 en el main actual
// =============================================================================

// Copypaste este código en la función execute_cycle() donde leas los ángulos:

void execute_cycle(float color, float j1_target, float j3_target)
{
    // ========= LECTURA DEL ENCODER J1 (TU AS5600) ==========
    MotorAngles angles = g_enc->readAll();  // Lee todos los encoders
    
    // Acceso a tus datos:
    float j1_current = angles.j1;       // Posición actual de J1 (AS5600)
    float j3_current = angles.j3;       // Posición actual de J3 (cuadratura)
    float j4_current = angles.j4;       // Posición actual de J4 (cuadratura)
    
    // ========= SI NECESITAS ÁNGULO RAW DE 12 BITS ==========
    // (Esto es lo que hacías con encoder.read_RAWANGLE())
    uint16_t raw = 0;
    if (g_enc->_enc_j1.read_RAWANGLE(raw) == ESP_OK) {
        float angle_deg = (raw * 360.0f) / 4096.0f;
        ESP_LOGI(TAG, "J1 RAW=%u (12-bits) → ANGLE=%.2f°", raw, angle_deg);
    }
    
    // ========= CONTROL DEL STEPPER J1 BASADO EN EL ENCODER ==========
    float error = j1_target - j1_current;
    
    // Tu lógica PID o de control proporcional:
    if (fabs(error) > 2.0f) {  // Si estás lejos del target
        float rpm = error * 0.5f;  // Control proporcional simple
        Smotor.setSpeed(rpm);
        ESP_LOGI(TAG, "J1 moving: error=%.2f°, rpm=%.1f", error, rpm);
    } else if (fabs(error) > 0.5f) {  // Zona de lentitud
        float rpm = error * 0.1f;  // Control más lento
        Smotor.setSpeed(rpm);
    } else {  // En el target
        Smotor.setSpeed(0);
        ESP_LOGI(TAG, "J1 reached target: %.2f°", j1_target);
    }
    
    // ========= LECTURA DE STATUS DEL IMÁN ==========
    // (Opcional: verificar que el imán está presente)
    AS5600_STATUS status = AS5600_STATUS(0);
    g_enc->_enc_j1.read_STATUS(status);
    if (!status.MagnetDetected()) {
        ESP_LOGW(TAG, "⚠️ Imán no detectado en J1 (AS5600)");
    }
    
    // ... resto de tu código para J3 y J4 ...
}


// =============================================================================
// ALTERNATIVA: Si quieres una función separada para el encoder
// (Tu amiga sugirió esto, aunque no es necesario)
// =============================================================================

struct AS5600Reading {
    uint16_t raw;
    float angle_deg;
    bool magnet_detected;
    esp_err_t err;
};

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

// Uso en el main:
// AS5600Reading enc_data = read_as5600();
// printf("RAW=%u, ANGLE=%.2f°, MAGNET=%s\n", 
//        enc_data.raw, enc_data.angle_deg,
//        enc_data.magnet_detected ? "OK" : "NO");

