#include "pid.h"
#include <stdio.h>

void pid_init(PID_Controller *pid,
              float kp, float ki, float kd,
              float setpoint,
              float integral_limit,
              float output_min, float output_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = setpoint;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;

    pid->integral_limit = (integral_limit >= 0.0f) ? integral_limit : 0.0f;
    pid->output_min = output_min;
    pid->output_max = output_max;

    // Mensaje de debug opcional
    printf("[PID] Init: Kp=%.2f Ki=%.2f Kd=%.2f SP=%.2f\n",
           kp, ki, kd, setpoint);
}

float pid_compute(PID_Controller *pid, float measured_val) {
    // Error = setpoint - valor_medido
    float error = pid->setpoint - measured_val;

    // Término proporcional
    float p_term = pid->kp * error;

    // Término integral con anti-windup
    pid->integral += error;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    float i_term = pid->ki * pid->integral;

    // Término derivativo (diferencia de error)
    float derivative = error - pid->prev_error;
    float d_term = pid->kd * derivative;

    // Salida total
    float output = p_term + i_term + d_term;

    // Saturación de la salida
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    // Guardar error previo
    pid->prev_error = error;

    return output;
}

void pid_reset(PID_Controller *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    printf("[PID] Reset\n");
}

void pid_set_setpoint(PID_Controller *pid, float new_setpoint) {
    pid->setpoint = new_setpoint;
    printf("[PID] New setpoint: %.2f\n", new_setpoint);
}
