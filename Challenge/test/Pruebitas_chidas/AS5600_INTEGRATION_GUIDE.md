# AS5600 Integration Guide

## ¿Cómo está tu encoder integrado?

Tu código usa `AS5600_i2c` directamente. El código actual de tu amiga usa una clase `Encoders` que **ya incluye el AS5600 para J1**.

```
Tu código:
    AS5600_i2c encoder(ENC_SDA, ENC_SCL);
    encoder.read_RAWANGLE(raw);

Código actual (equivalente):
    Encoders g_enc;
    g_enc->_enc_j1.read_RAWANGLE(raw);  ← mismo sensor, pero dentro de Encoders
```

---

## Formas de leer el encoder J1:

### **Forma 1: Lectura simple (RECOMENDADA)**
```cpp
// En main.cpp o cualquier función que tenga acceso a g_enc
MotorAngles angles = g_enc->readAll();
float j1_angle = angles.j1;  // Ángulo absoluto de J1 (AS5600)
printf("J1 angle: %.2f°\n", j1_angle);
```

### **Forma 2: Acceso directo al AS5600 (si necesitas métodos específicos)**
```cpp
// Leer ángulo RAW de 12 bits
uint16_t raw = 0;
esp_err_t err = g_enc->_enc_j1.read_RAWANGLE(raw);
if (err == ESP_OK) {
    float angle_deg = (raw * 360.0f) / 4096.0f;
    printf("RAW=%u, ANGLE=%.2f°\n", raw, angle_deg);
}

// O leer el status del imán
AS5600_STATUS status = AS5600_STATUS(0);
g_enc->_enc_j1.read_STATUS(status);
if (status.MagnetDetected()) {
    printf("Imán detectado ✓\n");
}
```

---

## Diferencia: 12 bits del encoder vs. 12 bits del PWM

- **AS5600 = 12 bits**: El sensor de ángulo tiene 4096 cuentas por revolución completa
- **LEDC_TIMER_12_BIT**: La resolución del PWM tiene 4096 niveles de duty cycle

Estos son **independientes**. Ya actualicé el timer del stepper a 12 bits en `main.cpp` para que tengas mejor control.

---

## Integración en el loop principal:

```cpp
// En STATE_EXECUTE_CYCLE o donde leas el encoder:
MotorAngles angles = g_enc->readAll();
float current_position = angles.j1;  // Tu posición absoluta actual
float error = s_j1_target - current_position;

// Control del motor paso a paso...
if (error > THRESHOLD) {
    Smotor.setSpeed(rpm);  // Acelera hacia el target
} else if (error < -THRESHOLD) {
    Smotor.setSpeed(-rpm);  // Reversa
} else {
    Smotor.setSpeed(0);  // Detén
}
```

---

## Resumen: no necesitas crear una función separada

Tu código ya funciona a través de:
- `g_enc->_enc_j1` = acceso directo al `AS5600_i2c`
- `g_enc->readAll()` = lectura integrada con otros encoders
- Timer de 12 bits ya está configurado ✓
