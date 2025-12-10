#include <stdio.h>
#include "pico/stdlib.h"
#include "hcsr04.h"

int main() {
    stdio_init_all();
    sleep_ms(1500);   // Permitir que el puerto serial se estabilice

    // Declaración de sensores
    HCSR04_Sensor sensor_left;
    HCSR04_Sensor sensor_right;

    // Inicialización
    hcsr04_init(&sensor_left,  HCSR04_LEFT_TRIG_PIN,  HCSR04_LEFT_ECHO_PIN);
    hcsr04_init(&sensor_right, HCSR04_RIGHT_TRIG_PIN, HCSR04_RIGHT_ECHO_PIN);

    // Registro para manejo por interrupciones
    hcsr04_register_sensor(&sensor_left);
    hcsr04_register_sensor(&sensor_right);

    printf("\n---- Prueba HC-SR04 iniciada ----\n");

    while (true) {
        // --- Sensor Izquierdo ---
        hcsr04_trigger(&sensor_left);
        float dist_left = hcsr04_get_distance_cm(&sensor_left);

        sleep_ms(60);  // Evitar interferencia acústica

        // --- Sensor Derecho ---
        hcsr04_trigger(&sensor_right);
        float dist_right = hcsr04_get_distance_cm(&sensor_right);

        // Impresión de resultados
        printf("Izq: %.2f cm  |  Der: %.2f cm\n", dist_left, dist_right);

        sleep_ms(100);  // Tiempo entre ciclos
    }

    return 0;
}
