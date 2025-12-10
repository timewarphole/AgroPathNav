#include "l298n.h"
#include <stdio.h>

#define DEFAULT_PWM_FREQ 1000 // Frecuencia de 1kHz
#define PWM_WRAP_VALUE (65535) // 16-bit resolution

/**
 * @brief Función auxiliar para configurar el PWM.
 */
static void setup_pwm(uint pin, uint wrap_value, uint *slice, uint *channel) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    *slice = pwm_gpio_to_slice_num(pin);
    *channel = pwm_gpio_to_channel(pin);

    // Configuración básica del PWM
    pwm_set_wrap(*slice, wrap_value);
    pwm_set_chan_level(*slice, *channel, 0); // Empieza en 0%
    pwm_set_enabled(*slice, true);
}

// --- Implementación de l298n_init ---
void l298n_init(L298N_Driver *driver, uint pin_in1, uint pin_in2, uint pin_enA, uint pin_in3, uint pin_in4, uint pin_enB) {
    
    // Asignar Pines a la estructura
    driver->motorA_in1 = pin_in1;
    driver->motorA_in2 = pin_in2;
    driver->motorA_enA = pin_enA;
    driver->motorB_in3 = pin_in3;
    driver->motorB_in4 = pin_in4;
    driver->motorB_enB = pin_enB;
    driver->pwm_wrap = PWM_WRAP_VALUE;

    // Configurar pines de dirección como salidas digitales
    gpio_init(driver->motorA_in1);
    gpio_set_dir(driver->motorA_in1, GPIO_OUT);
    gpio_init(driver->motorA_in2);
    gpio_set_dir(driver->motorA_in2, GPIO_OUT);
    
    gpio_init(driver->motorB_in3);
    gpio_set_dir(driver->motorB_in3, GPIO_OUT);
    gpio_init(driver->motorB_in4);
    gpio_set_dir(driver->motorB_in4, GPIO_OUT);
    
    // Inicializar pines de dirección en estado detenido (LOW)
    gpio_put(driver->motorA_in1, 0);
    gpio_put(driver->motorA_in2, 0);
    gpio_put(driver->motorB_in3, 0);
    gpio_put(driver->motorB_in4, 0);

    // Configurar pines de Enable (Velocidad) para PWM
    setup_pwm(driver->motorA_enA, driver->pwm_wrap, &driver->motorA_pwm_slice, &driver->motorA_pwm_channel);
    setup_pwm(driver->motorB_enB, driver->pwm_wrap, &driver->motorB_pwm_slice, &driver->motorB_pwm_channel);
}

// --- Implementación de l298n_set_motor ---
void l298n_set_motor(L298N_Driver *driver, int motor_id, int speed_percent, int direction) {
    if (speed_percent < 0) speed_percent = 0;
    if (speed_percent > 100) speed_percent = 100;

    // Calcular el nivel del PWM (0 a driver->pwm_wrap)
    uint16_t pwm_level = (uint16_t)(((float)speed_percent / 100.0f) * driver->pwm_wrap);

    uint in1_pin, in2_pin, pwm_slice, pwm_channel;
    
    // Determinar qué motor controlar
    if (motor_id == 0) { // Motor A
        in1_pin = driver->motorA_in1;
        in2_pin = driver->motorA_in2;
        pwm_slice = driver->motorA_pwm_slice;
        pwm_channel = driver->motorA_pwm_channel;
    } else if (motor_id == 1) { // Motor B
        in1_pin = driver->motorB_in3;
        in2_pin = driver->motorB_in4;
        pwm_slice = driver->motorB_pwm_slice;
        pwm_channel = driver->motorB_pwm_channel;
    } else {
        return; // ID de motor inválida
    }

    // Configurar dirección y velocidad
    if (direction == 1) { // Adelante
        gpio_put(in1_pin, 1);
        gpio_put(in2_pin, 0);
    } else if (direction == -1) { // Atrás
        gpio_put(in1_pin, 0);
        gpio_put(in2_pin, 1);
    } else { // Detener (direction == 0 o cualquier otro valor)
        pwm_level = 0; // Detener completamente el motor
        gpio_put(in1_pin, 0);
        gpio_put(in2_pin, 0);
    }

    // Aplicar nivel de PWM (Velocidad)
    pwm_set_chan_level(pwm_slice, pwm_channel, pwm_level);
}

// --- Implementación de l298n_stop_all ---
void l298n_stop_all(L298N_Driver *driver) {
    // Detener Motor A
    l298n_set_motor(driver, 0, 0, 0);
    // Detener Motor B
    l298n_set_motor(driver, 1, 0, 0);
}