#include <stdio.h>
#include "pico/stdlib.h"
#include "hcsr04.h"
#include "pid.h"
#include "l298n.h"

// Pines de los motores
#define MA_IN1_PIN  9   // Motor A IN1
#define MA_IN2_PIN  10  // Motor A IN2
#define MA_ENA_PIN  8   // Motor A ENA (PWM)

#define MB_IN3_PIN  11  // Motor B IN3
#define MB_IN4_PIN  14  // Motor B IN4
#define MB_ENB_PIN  15  // Motor B ENB (PWM)

int main() {

    stdio_init_all();
    sleep_ms(1500);

    printf("\n-----------------------------------------------------\n");
    printf("   Control PID con dos HC-SR04 para centrado lateral\n");
    printf("-----------------------------------------------------\n\n");

    // ----------------------------------------------------------
    // Sensores ultrasónicos
    // ----------------------------------------------------------
    HCSR04_Sensor sensor_left;
    HCSR04_Sensor sensor_right;

    hcsr04_init(&sensor_left,  HCSR04_LEFT_TRIG_PIN,  HCSR04_LEFT_ECHO_PIN);
    hcsr04_init(&sensor_right, HCSR04_RIGHT_TRIG_PIN, HCSR04_RIGHT_ECHO_PIN);

    hcsr04_register_sensor(&sensor_left);
    hcsr04_register_sensor(&sensor_right);

    // ----------------------------------------------------------
    // Driver de motores L298N
    // ----------------------------------------------------------
    L298N_Driver motors;
    l298n_init(&motors, 
               MA_IN1_PIN, MA_IN2_PIN, MA_ENA_PIN,  // Motor A (Izquierdo)
               MB_IN3_PIN, MB_IN4_PIN, MB_ENB_PIN); // Motor B (Derecho)

    printf("[L298N] Motores inicializados\n");

    // ----------------------------------------------------------
    // PID: error = distancia_izq - distancia_der
    // ----------------------------------------------------------
    PID_Controller pid;

    float kp = 4.4f;
    float ki = 0.12f;
    float kd = 1.0f;

    float setpoint = 0.0f;       // Error deseado: centrado
    float integral_limit = 60.0f;
    float output_min = -100.0f;
    float output_max = +100.0f;

    pid_init(&pid, kp, ki, kd, setpoint,
             integral_limit, output_min, output_max);

    // ----------------------------------------------------------
    // Parámetros de control de motores
    // ----------------------------------------------------------
    const int base_speed = 30;  // Velocidad base en % (ajusta según tu robot)
    const int min_speed  = 15;  // Velocidad mínima para evitar que se detenga
    const int max_speed  = 50;  // Velocidad máxima

    // ----------------------------------------------------------
    // Bucle principal
    // ----------------------------------------------------------
    while (true) {

        // ------ Lectura secuencial para evitar interferencia ------
        hcsr04_trigger(&sensor_left);
        float d_left = hcsr04_get_distance_cm(&sensor_left);

        sleep_ms(60);

        hcsr04_trigger(&sensor_right);
        float d_right = hcsr04_get_distance_cm(&sensor_right);

        // ------ Error para el PID ------
        float error_lateral = d_left - d_right;

        // ------ PID ------
        float correction = pid_compute(&pid, error_lateral);

        // ------ Aplicar corrección a los motores ------
        // Motor izquierdo (A) reduce velocidad si correction > 0 (más cerca de la izquierda)
        // Motor derecho (B) aumenta velocidad si correction > 0
        int speed_left  = base_speed - (int)correction;
        int speed_right = base_speed + (int)correction;

        // Saturar velocidades
        if (speed_left < min_speed)   speed_left = min_speed;
        if (speed_left > max_speed)   speed_left = max_speed;
        if (speed_right < min_speed)  speed_right = min_speed;
        if (speed_right > max_speed)  speed_right = max_speed;

        // Enviar comandos a los motores (dirección 1 = adelante)
        l298n_set_motor(&motors, 0, speed_left,  1); // Motor A (Izquierdo)
        l298n_set_motor(&motors, 1, speed_right, 1); // Motor B (Derecho)

        // ------ Información útil ------
        printf("DL: %.2f cm | DR: %.2f cm | Err: %.2f | PID: %.2f | ML: %d%% | MR: %d%%\n",
               d_left, d_right, error_lateral, correction, speed_left, speed_right);

        sleep_ms(100);
    }

    return 0;
}
