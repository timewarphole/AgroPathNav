#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Estructura para un controlador PID discreto.
 *
 * kp, ki, kd      : ganancias del controlador
 * setpoint        : valor objetivo
 * integral        : acumulador integral (anti-windup con límites)
 * prev_error      : error en la iteración anterior (para derivada)
 * integral_limit  : límite absoluto del término integral
 * output_min/max  : saturación de la salida (por ejemplo -100..100)
 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float setpoint;

    float integral;
    float prev_error;

    float integral_limit;
    float output_min;
    float output_max;
} PID_Controller;

/**
 * @brief Inicializa un controlador PID.
 *
 * @param pid            Puntero a la estructura PID_Controller.
 * @param kp             Ganancia proporcional.
 * @param ki             Ganancia integral.
 * @param kd             Ganancia derivativa.
 * @param setpoint       Valor objetivo.
 * @param integral_limit Límite absoluto del término integral (anti-windup).
 * @param output_min     Salida mínima permitida.
 * @param output_max     Salida máxima permitida.
 */
void pid_init(PID_Controller *pid,
              float kp, float ki, float kd,
              float setpoint,
              float integral_limit,
              float output_min, float output_max);

/**
 * @brief Calcula la salida del PID para una nueva medición.
 *
 * @param pid           Puntero al controlador.
 * @param measured_val  Valor medido actual (variable de proceso).
 * @return              Salida del controlador (saturada en output_min/max).
 */
float pid_compute(PID_Controller *pid, float measured_val);

/**
 * @brief Resetea el estado interno del PID (integral y error previo).
 *
 * @param pid Puntero al controlador.
 */
void pid_reset(PID_Controller *pid);

/**
 * @brief Cambia el setpoint del controlador.
 *
 * @param pid          Puntero al controlador.
 * @param new_setpoint Nuevo valor objetivo.
 */
void pid_set_setpoint(PID_Controller *pid, float new_setpoint);

#endif // PID_H
