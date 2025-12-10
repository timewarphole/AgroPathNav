#ifndef HCSR04_H
#define HCSR04_H

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

// Estructura para un sensor HC-SR04
typedef struct {
    uint trig_pin;                  // Pin trigger
    uint echo_pin;                  // Pin echo
    volatile uint32_t echo_start_time; // Tiempo de inicio del pulso echo
    volatile uint32_t echo_end_time;   // Tiempo de fin del pulso echo
    volatile bool echo_received;       // Flag: se recibió echo completo
    volatile bool timeout;             // Flag: timeout
    float last_distance_cm;            // Última distancia medida en cm
} HCSR04_Sensor;

// --- Constantes ---
#define HCSR04_TIMEOUT_US      30000   // Timeout de 30 ms (~5 m máx)
#define HCSR04_MAX_DISTANCE_CM 20     // Distancia máx teórica
#define HCSR04_MIN_DISTANCE_CM 2       // Distancia mín teórica
#define SOUND_SPEED_CM_US      0.0343f // 343 m/s = 0.0343 cm/µs

// --- Pines sugeridos para Raspberry Pi Pico W ---
#define HCSR04_LEFT_TRIG_PIN   21   // GP2
#define HCSR04_LEFT_ECHO_PIN   20   // GP3
#define HCSR04_RIGHT_TRIG_PIN  2   // GP4
#define HCSR04_RIGHT_ECHO_PIN  3   // GP5

// --- Funciones Públicas ---


/**
 * @brief Inicializa un sensor HC-SR04.
 * @param sensor   Puntero a la estructura HCSR04_Sensor.
 * @param trig_pin Pin GPIO para TRIGGER.
 * @param echo_pin Pin GPIO para ECHO.
 */
void hcsr04_init(HCSR04_Sensor *sensor, uint trig_pin, uint echo_pin);

/**
 * @brief Inicia una medición enviando el pulso trigger (modo polling).
 * @param sensor Puntero a la estructura HCSR04_Sensor.
 */
void hcsr04_trigger(HCSR04_Sensor *sensor);

/**
 * @brief Espera y calcula la distancia medida (blocking, con timeout).
 * @param sensor Puntero a la estructura HCSR04_Sensor.
 * @return Distancia en cm, o -1.0 si hay error/timeout.
 */
float hcsr04_get_distance_cm(HCSR04_Sensor *sensor);

/**
 * @brief Verifica si la medición está completa (modo non-blocking).
 * @param sensor Puntero a la estructura HCSR04_Sensor.
 * @return true si hay datos disponibles o timeout, false si aún no.
 */
bool hcsr04_is_ready(HCSR04_Sensor *sensor);

/**
 * @brief Obtiene la última distancia calculada sin bloquear.
 * @param sensor Puntero a la estructura HCSR04_Sensor.
 * @return Distancia en cm, o -1.0 si no hay datos válidos.
 */
float hcsr04_read_distance_cm(HCSR04_Sensor *sensor);

/**
 * @brief Callback de interrupción GPIO para el pin ECHO (uso interno).
 * Debe ser usado como callback global de GPIO.
 * @param gpio   Pin que generó la interrupción.
 * @param events Eventos de la interrupción.
 */
void hcsr04_gpio_callback(uint gpio, uint32_t events);

/**
 * @brief Registra un sensor para el manejo de interrupciones.
 * @param sensor Puntero a la estructura HCSR04_Sensor.
 */
void hcsr04_register_sensor(HCSR04_Sensor *sensor);

#endif // HCSR04_H