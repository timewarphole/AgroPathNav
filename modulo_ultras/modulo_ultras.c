#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
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

// Pin del botón de inicio
#define START_BUTTON_PIN  22

// ----------------------------------------------------------
// Variables globales para el botón (polling)
// ----------------------------------------------------------
bool system_running = false;
bool last_button_state = true;  // Pull-up: 1=no presionado, 0=presionado
uint32_t last_button_time = 0;
const uint32_t DEBOUNCE_TIME_MS = 200;  // 200ms de antirrebote

// ----------------------------------------------------------
// Función para leer botón con antirrebote (polling)
// ----------------------------------------------------------
void check_button() {
    bool current_state = gpio_get(START_BUTTON_PIN);  // Lee el estado del pin
    
    // Detectar flanco descendente (presión del botón)
    if (last_button_state == true && current_state == false) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        // Antirrebote: ignorar si no han pasado 200ms
        if ((now - last_button_time) >= DEBOUNCE_TIME_MS) {
            last_button_time = now;
            
            // Toggle del estado
            system_running = !system_running;
            
            printf("\n[BOTON] Sistema %s\n\n", system_running ? "INICIADO" : "DETENIDO");
        }
    }
    
    last_button_state = current_state;  // Actualizar estado
}

// ----------------------------------------------------------
// Inicialización del botón (sin IRQ)
// ----------------------------------------------------------
void init_start_button() {
    gpio_init(START_BUTTON_PIN);
    gpio_set_dir(START_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(START_BUTTON_PIN);  // Pull-up interno (activo en bajo)
    
    printf("[BOTON] Configurado en GP%d (activo bajo con pull-up, polling)\n", START_BUTTON_PIN);
    printf("[BOTON] Presiona el boton para INICIAR el sistema\n\n");
}

int main() {

    stdio_init_all();
    sleep_ms(1500);

    printf("\n-----------------------------------------------------\n");
    printf("   Control PID con dos HC-SR04 para centrado lateral\n");
    printf("   (Modo: Polling + IRQ Non-Blocking)\n");
    printf("-----------------------------------------------------\n\n");

    // ----------------------------------------------------------
    // Botón de inicio/parada
    // ----------------------------------------------------------
    init_start_button();

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

    float kp = 4.5f;
    float ki = 0.12f;
    float kd = 1.0f;

    float setpoint = 0.0f;       
    float integral_limit = 60.0f;
    float output_min = -100.0f;
    float output_max = +100.0f;

    pid_init(&pid, kp, ki, kd, setpoint,
             integral_limit, output_min, output_max);

    // ----------------------------------------------------------
    // Parámetros de control de motores
    // ----------------------------------------------------------
    const int base_speed = 50;
    const int min_speed  = 20;
    const int max_speed  = 70;

    // ----------------------------------------------------------
    // Bucle principal (Non-Blocking con IRQ)
    // ----------------------------------------------------------
    while (true) {

        // ------ Leer botón en cada iteración ------
        check_button();

        // ------ Verificar si el sistema está corriendo ------
        if (!system_running) {
            // Detener motores y resetear PID
            l298n_stop_all(&motors);
            pid_reset(&pid);
            
            printf("Sistema detenido. Esperando boton...\r");
            sleep_ms(100);
            continue;
        }

        // ------ Disparo casi simultáneo con pequeño delay ------
        hcsr04_trigger(&sensor_left);
        sleep_us(10);
        hcsr04_trigger(&sensor_right);

        uint32_t start_time = time_us_32();
        bool left_ready  = false;
        bool right_ready = false;

        // ------ Polling no bloqueante (máx 30ms de espera) ------
        while ((time_us_32() - start_time) < 30000) {
            
            if (!left_ready && hcsr04_is_ready(&sensor_left)) {
                left_ready = true;
            }
            
            if (!right_ready && hcsr04_is_ready(&sensor_right)) {
                right_ready = true;
            }

            if (left_ready && right_ready) {
                break;
            }

            tight_loop_contents();
        }

        // ------ Lectura de resultados ------
        float d_left  = hcsr04_read_distance_cm(&sensor_left);
        float d_right = hcsr04_read_distance_cm(&sensor_right);

        if (d_left <= 0.0f)  d_left  = 0.0f;
        if (d_right <= 0.0f) d_right = 0.0f;

        // ------ Error para el PID ------
        float error_lateral = d_left - d_right;

        // ------ PID ------
        float correction = pid_compute(&pid, error_lateral);

        // ------ Aplicar corrección a los motores ------
        int speed_left  = (base_speed - (int)correction);
        int speed_right = (base_speed + (int)correction) + 15;

        // Saturar velocidades
        if (speed_left < min_speed)   speed_left = min_speed;
        if (speed_left > max_speed)   speed_left = max_speed;
        if (speed_right < min_speed)  speed_right = min_speed;
        if (speed_right > max_speed)  speed_right = max_speed;

        // Enviar comandos a los motores
        l298n_set_motor(&motors, 0, speed_left,  1); // Motor A (Izquierdo)
        l298n_set_motor(&motors, 1, speed_right, 1); // Motor B (Derecho)

        // ------ Información útil ------
        printf("DL: %.2f cm | DR: %.2f cm | Err: %.2f | PID: %.2f | ML: %d%% | MR: %d%%\n",
               d_left, d_right, error_lateral, correction, speed_left, speed_right);

        sleep_ms(50);
    }

    return 0;
}
