#include <stdio.h>
#include "pico/stdlib.h"

#include "hcsr04.h"
#include "pid.h"

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
    // PID: error = distancia_izq - distancia_der
    // ----------------------------------------------------------
    PID_Controller pid;

    float kp = 1.4f;
    float ki = 0.15f;
    float kd = 0.35f;

    float setpoint = 0.0f;       // Error deseado: centrado
    float integral_limit = 60.0f;
    float output_min = -100.0f;
    float output_max = +100.0f;

    pid_init(&pid, kp, ki, kd, setpoint,
             integral_limit, output_min, output_max);

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

        // ------ Información útil ------
        printf("DL: %.2f cm | DR: %.2f cm | Error: %.2f | PID: %.2f\n",
               d_left, d_right, error_lateral, correction);

        /* 
            En una aplicación real:
            motor_left  = base_speed - correction;
            motor_right = base_speed + correction;
        */

        sleep_ms(100);
    }

    return 0;
}