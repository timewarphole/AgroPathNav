#include "hcsr04.h"
#include <stdio.h>

#define MAX_SENSORS 4

// Lista de sensores registrados para el callback de GPIO
static HCSR04_Sensor *registered_sensors[MAX_SENSORS] = {0};
static uint8_t sensor_count = 0;

// --- Función auxiliar para calcular distancia ---
static float calculate_distance(uint32_t pulse_duration_us) {
    // Distancia = (Tiempo * Velocidad del Sonido) / 2
    float distance_cm = (pulse_duration_us * SOUND_SPEED_CM_US) / 2.0f;

    if (distance_cm < HCSR04_MIN_DISTANCE_CM || distance_cm > HCSR04_MAX_DISTANCE_CM) {
        return -1.0f; // Fuera de rango
    }
    return distance_cm;
}

// --- Implementación de hcsr04_init ---
void hcsr04_init(HCSR04_Sensor *sensor, uint trig_pin, uint echo_pin) {
    sensor->trig_pin        = trig_pin;
    sensor->echo_pin        = echo_pin;
    sensor->echo_start_time = 0;
    sensor->echo_end_time   = 0;
    sensor->echo_received   = false;
    sensor->timeout         = false;
    sensor->last_distance_cm = -1.0f;

    // TRIG como salida
    gpio_init(trig_pin);
    gpio_set_dir(trig_pin, GPIO_OUT);
    gpio_put(trig_pin, 0);

    // ECHO como entrada con pull-down
    gpio_init(echo_pin);
    gpio_set_dir(echo_pin, GPIO_IN);
    gpio_pull_down(echo_pin);

    // Habilitar IRQ en ambos flancos (rising/falling) para este pin
    gpio_set_irq_enabled_with_callback(
        echo_pin,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &hcsr04_gpio_callback
    );

    printf("[HCSR04] Init TRIG=GP%d, ECHO=GP%d\n", trig_pin, echo_pin);
}

// --- Implementación de hcsr04_trigger ---
void hcsr04_trigger(HCSR04_Sensor *sensor) {
    // Resetear flags y tiempos
    sensor->echo_received   = false;
    sensor->timeout         = false;
    sensor->echo_start_time = 0;
    sensor->echo_end_time   = 0;

    // Pulso de 10 µs en TRIG
    gpio_put(sensor->trig_pin, 1);
    sleep_us(10);
    gpio_put(sensor->trig_pin, 0);
}

// --- Implementación de hcsr04_get_distance_cm (blocking) ---
float hcsr04_get_distance_cm(HCSR04_Sensor *sensor) {
    hcsr04_trigger(sensor);

    uint32_t start_wait = time_us_32();

    // Esperar a que se reciba el pulso o timeout
    while (!sensor->echo_received && !sensor->timeout) {
        if ((time_us_32() - start_wait) > HCSR04_TIMEOUT_US) {
            sensor->timeout = true;
            break;
        }
        tight_loop_contents();
    }

    if (sensor->timeout || !sensor->echo_received) {
        sensor->last_distance_cm = -1.0f;
        return -1.0f;
    }

    uint32_t pulse_duration = sensor->echo_end_time - sensor->echo_start_time;
    float distance = calculate_distance(pulse_duration);
    sensor->last_distance_cm = distance;

    return distance;
}

// --- Implementación de hcsr04_is_ready (non-blocking) ---
bool hcsr04_is_ready(HCSR04_Sensor *sensor) {
    // Revisar timeout manual
    if (!sensor->echo_received && sensor->echo_start_time != 0) {
        uint32_t elapsed = time_us_32() - sensor->echo_start_time;
        if (elapsed > HCSR04_TIMEOUT_US) {
            sensor->timeout = true;
            sensor->echo_received = false;
        }
    }
    return sensor->echo_received || sensor->timeout;
}

// --- Implementación de hcsr04_read_distance_cm (non-blocking) ---
float hcsr04_read_distance_cm(HCSR04_Sensor *sensor) {
    if (!sensor->echo_received || sensor->timeout) {
        return -1.0f;
    }

    uint32_t pulse_duration = sensor->echo_end_time - sensor->echo_start_time;
    float distance = calculate_distance(pulse_duration);
    sensor->last_distance_cm = distance;

    return distance;
}

// --- Implementación de hcsr04_register_sensor ---
void hcsr04_register_sensor(HCSR04_Sensor *sensor) {
    if (sensor_count < MAX_SENSORS) {
        registered_sensors[sensor_count++] = sensor;
        printf("[HCSR04] Sensor registrado. Total: %d\n", sensor_count);
    } else {
        printf("[HCSR04] ERROR: Máximo de sensores alcanzado\n");
    }
}

// --- Callback de interrupción GPIO (global) ---
void hcsr04_gpio_callback(uint gpio, uint32_t events) {
    // Buscar qué sensor corresponde a este pin ECHO
    HCSR04_Sensor *sensor = NULL;

    for (uint8_t i = 0; i < sensor_count; i++) {
        if (registered_sensors[i] && registered_sensors[i]->echo_pin == gpio) {
            sensor = registered_sensors[i];
            break;
        }
    }

    if (sensor == NULL) {
        return; // Pin no pertenece a ningún sensor registrado
    }

    if (events & GPIO_IRQ_EDGE_RISE) {
        // Flanco ascendente: inicio del pulso ECHO
        sensor->echo_start_time = time_us_32();
        sensor->echo_received   = false;
        sensor->timeout         = false;
    } else if (events & GPIO_IRQ_EDGE_FALL) {
        // Flanco descendente: fin del pulso ECHO
        sensor->echo_end_time = time_us_32();
        sensor->echo_received = true;
    }
}
