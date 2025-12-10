#ifndef L298N_DRIVER_H
#define L298N_DRIVER_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"

// --- Estructura del Driver para Dos Motores ---
typedef struct {
    // Motor A
    uint motorA_in1;    // Control A (Dirección)
    uint motorA_in2;   
    uint motorA_enA;    // Enable A (PWM para Velocidad)
    uint motorA_pwm_slice;
    uint motorA_pwm_channel;
    
    // Motor B
    uint motorB_in3;    // Control B (Dirección)
    uint motorB_in4;   
    uint motorB_enB;    // Enable B (PWM para Velocidad)
    uint motorB_pwm_slice;
    uint motorB_pwm_channel;
    
    // Frecuencia y Rango del PWM
    uint pwm_wrap;
} L298N_Driver;

// --- Funciones Públicas ---

/**
 * @brief El driver L298N se debe inicializar con los pines GPIO especificados.
 * @param driver Puntero a la estructura L298N_Driver.
 * @param pin_in1 Pin IN1 (Motor A).
 * @param pin_in2 Pin IN2 (Motor A).
 * @param pin_enA Pin ENA (Motor A, Velocidad PWM).
 * @param pin_in3 Pin IN3 (Motor B).
 * @param pin_in4 Pin IN4 (Motor B).
 * @param pin_enB Pin ENB (Motor B, Velocidad PWM).
 */
void l298n_init(L298N_Driver *driver, uint pin_in1, uint pin_in2, uint pin_enA, uint pin_in3, uint pin_in4, uint pin_enB);

/**
 * @brief Establece la velocidad y dirección de un motor.
 * @param driver Puntero a la estructura L298N_Driver.
 * @param motor_id ID del motor (0 para Motor A, 1 para Motor B).
 * @param speed_percent Velocidad en porcentaje (0 a 100).
 * @param direction 0: Detener, 1: Adelante, -1: Atrás.
 */
void l298n_set_motor(L298N_Driver *driver, int motor_id, int speed_percent, int direction);

/**
 * @brief Detiene ambos motores.
 * @param driver Puntero a la estructura L298N_Driver.
 */
void l298n_stop_all(L298N_Driver *driver);

#endif // L298N_DRIVER_H