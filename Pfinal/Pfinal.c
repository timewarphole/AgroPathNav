/**
 * @file Pfinal.c
 * @brief Sistema de carro autónomo con muestreo ambiental
 * 
 * @details Integra:
 *          - Control PID para corrección de trayectoria con HC-SR04
 *          - Muestreo de temperatura (LM35) y luz (LDR) durante operación
 *          - Almacenamiento en EEPROM con buffer circular
 *          - Consola USB/UART con comandos DATA y CLEAR
 *          - Botón físico para control de estado (IRQ)
 *          - Arquitectura polling + interrupciones
 * 
 * @copyright Copyright (c) 2025
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>         
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/timer.h"

// Drivers del proyecto
#include "lib/control/pid.h"
#include "lib/motores/l298n.h" 
#include "lib/sensores/hcsr04.h"
#include "lib/sensores/lm35.h"  
#include "lib/sensores/fotoresistencia.h"
#include "lib/eeprom/eeprom.h"



/* ========== CONFIGURACIÓN DEL SISTEMA ========== */

// Pin del botón de control
#define BUTTON_PIN 22

// Configuración de operación
#define OPERATION_TIME_MS 10000  // 10 segundos de operación máxima
#define SAMPLE_INTERVAL_MS 1000  // Muestreo cada 1 segundo

// Parámetros del PID
#define PID_KP 3.0f
#define PID_KI 0.5f
#define PID_KD 1.5f
#define PID_SETPOINT 0.0f        // Equilibrio: ambos sensores a misma distancia
#define PID_INTEGRAL_LIMIT 50.0f
#define PID_OUTPUT_MIN -100.0f
#define PID_OUTPUT_MAX 100.0f

// Velocidad base de los motores
#define BASE_SPEED 50  // 50% de velocidad base

// Setpoint de distancia lateral (cm)
#define TARGET_DISTANCE_CM 10.0f

/* ========== MÁQUINA DE ESTADOS ========== */

typedef enum {
    STATE_IDLE = 0,      // Reposo: sin movimiento ni logging
    STATE_RUNNING        // Activo: PID + motores + muestreo
} SystemState;

/* ========== VARIABLES GLOBALES ========== */

// Estado del sistema
static volatile SystemState g_system_state = STATE_IDLE;

// Flag de botón presionado (IRQ)
static volatile bool g_button_pressed = false;

// Timestamp de inicio de operación
static uint32_t g_operation_start_time = 0;

// Timestamp de último muestreo
static uint32_t g_last_sample_time = 0;

// Drivers de hardware
static L298N_Driver motor_driver;
static HCSR04_Sensor sensor_left, sensor_right;
static PID_Controller pid_controller;
static LM35_Sensor lm35;
static LDR_Sensor ldr;

/* ========== PROTOTIPOS DE FUNCIONES ========== */

// Inicialización
void system_init(void);
void hardware_init(void);
void sensors_init(void);

// Control de estado
void button_isr(uint gpio, uint32_t events);
void state_machine_update(void);

// Navegación y control
void navigation_task(void);
void sampling_task(void);

// Consola de comandos
void console_task(void);
void command_data(void);
void command_clear(void);

/* ========== FUNCIÓN PRINCIPAL ========== */

int main(void) {
    // Inicialización completa del sistema
    system_init();
    
    printf("\n");
    printf("========================================\n");
    printf("  CARRO AUTÓNOMO - MUESTREO AMBIENTAL  \n");
    printf("========================================\n");
    printf("Comandos disponibles:\n");
    printf("  DATA  - Mostrar todas las muestras\n");
    printf("  CLEAR - Limpiar buffer de EEPROM\n");
    printf("\n");
    printf("Presione el botón para iniciar/detener\n");
    printf("Estado: IDLE\n\n");
    
    // Loop principal
    while (true) {
        // Actualizar máquina de estados
        state_machine_update();
        
        // Ejecutar tareas según estado
        if (g_system_state == STATE_RUNNING) {
            navigation_task();
            sampling_task();
        }
        
        // Consola de comandos (siempre activa)
        console_task();
        
        // Pequeño delay para no saturar CPU
        sleep_ms(10);
    }
    
    return 0;
}

/* ========== INICIALIZACIÓN ========== */

void system_init(void) {
    // Inicializar stdio para USB
    stdio_init_all();
    
    // Esperar conexión USB (opcional, comentar si no necesario)
    sleep_ms(2000);
    
    // Inicializar hardware
    hardware_init();
    
    // Inicializar sensores
    sensors_init();
    
    printf("[INIT] Sistema inicializado correctamente\n");
}

void hardware_init(void) {
    // Inicializar botón con interrupción
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);
    
    // Configurar IRQ del botón (flanco de bajada, botón con pull-up)
    gpio_set_irq_enabled_with_callback(
        BUTTON_PIN,
        GPIO_IRQ_EDGE_FALL,
        true,
        &button_isr
    );
    
    printf("[INIT] Botón configurado en GP%d\n", BUTTON_PIN);
    
    // Inicializar ADC (compartido por LM35 y LDR)
    adc_init();
    printf("[INIT] ADC inicializado\n");
    
    // Inicializar EEPROM
    eeprom_error_t eeprom_err = eeprom_init();
    if (eeprom_err == EEPROM_SUCCESS) {
        printf("[INIT] EEPROM inicializada (registros: %d/%d)\n",
               eeprom_get_record_count(),
               eeprom_get_capacity());
    } else {
        printf("[ERROR] Fallo al inicializar EEPROM: %d\n", eeprom_err);
    }
}

void sensors_init(void) {
    // Inicializar driver de motores L298N
    l298n_init(&motor_driver,
               10,  // Motor A IN1 (GP10)
               11,  // Motor A IN2 (GP11)
               14,  // Motor A ENA (GP14, PWM)
               12,  // Motor B IN3 (GP12)
               13,  // Motor B IN4 (GP13)
               15   // Motor B ENB (GP15, PWM)
    );
    printf("[INIT] Driver L298N inicializado\n");
    
    // Inicializar sensores HC-SR04
    hcsr04_init(&sensor_left, 2, 3);   // TRIG=GP2, ECHO=GP3
    hcsr04_register_sensor(&sensor_left);
    
    hcsr04_init(&sensor_right, 4, 5);  // TRIG=GP4, ECHO=GP5
    hcsr04_register_sensor(&sensor_right);
    
    printf("[INIT] Sensores HC-SR04 inicializados\n");
    
    // Inicializar controlador PID
    pid_init(&pid_controller,
             PID_KP, PID_KI, PID_KD,
             PID_SETPOINT,
             PID_INTEGRAL_LIMIT,
             PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    
    printf("[INIT] Controlador PID configurado\n");
    
    // Inicializar sensores analógicos
    lm35_init(&lm35, 0, 26);  // ADC0, GP26
    ldr_init(&ldr, 1, 27);    // ADC1, GP27
    
    printf("[INIT] Sensores ambientales (LM35, LDR) inicializados\n");
}

/* ========== INTERRUPCIÓN DEL BOTÓN ========== */

void button_isr(uint gpio, uint32_t events) {
    // Debounce simple: marcar flag para procesar en main loop
    if (gpio == BUTTON_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
        g_button_pressed = true;
    }
}

/* ========== MÁQUINA DE ESTADOS ========== */

void state_machine_update(void) {
    // Verificar si se presionó el botón
    if (g_button_pressed) {
        g_button_pressed = false;  // Limpiar flag
        
        if (g_system_state == STATE_IDLE) {
            // Transición: IDLE → RUNNING
            g_system_state = STATE_RUNNING;
            g_operation_start_time = to_ms_since_boot(get_absolute_time());
            g_last_sample_time = g_operation_start_time;
            
            // Resetear PID
            pid_reset(&pid_controller);
            
            printf("\n[ESTADO] RUNNING - Operación iniciada\n");
            
        } else if (g_system_state == STATE_RUNNING) {
            // Transición: RUNNING → IDLE (detención manual)
            g_system_state = STATE_IDLE;
            
            // Detener motores
            l298n_stop_all(&motor_driver);
            
            // Persistir metadatos en EEPROM
            eeprom_save_metadata();
            
            printf("[ESTADO] IDLE - Operación detenida por botón\n");
            printf("Registros almacenados: %d\n\n", eeprom_get_record_count());
        }
    }
    
    // Verificar timeout de 10 segundos en modo RUNNING
    if (g_system_state == STATE_RUNNING) {
        uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - g_operation_start_time;
        
        if (elapsed >= OPERATION_TIME_MS) {
            // Transición: RUNNING → IDLE (timeout automático)
            g_system_state = STATE_IDLE;
            
            // Detener motores
            l298n_stop_all(&motor_driver);
            
            // Persistir metadatos en EEPROM
            eeprom_save_metadata();
            
            printf("[ESTADO] IDLE - Timeout de 10 segundos alcanzado\n");
            printf("Registros almacenados: %d\n\n", eeprom_get_record_count());
        }
    }
}

/* ========== TAREA DE NAVEGACIÓN (PID + MOTORES) ========== */

void navigation_task(void) {
    // Disparar mediciones de ambos sensores (polling)
    hcsr04_trigger(&sensor_left);
    sleep_us(100);  // Pequeña pausa entre disparos
    hcsr04_trigger(&sensor_right);
    
    // Esperar a que ambas mediciones estén listas (polling sin bloquear indefinidamente)
    uint32_t timeout_start = time_us_32();
    while ((!hcsr04_is_ready(&sensor_left) || !hcsr04_is_ready(&sensor_right))) {
        if ((time_us_32() - timeout_start) > 50000) {  // Timeout 50ms
            printf("[WARN] Timeout en sensores HC-SR04\n");
            l298n_stop_all(&motor_driver);
            return;
        }
        tight_loop_contents();
    }
    
    // Leer distancias (las interrupciones GPIO ya capturaron los tiempos)
    float dist_left = hcsr04_read_distance_cm(&sensor_left);
    float dist_right = hcsr04_read_distance_cm(&sensor_right);
    
    // Validar lecturas
    if (dist_left < 0.0f || dist_right < 0.0f) {
        // Lecturas inválidas: detener motores por seguridad
        l298n_stop_all(&motor_driver);
        return;
    }
    
    // Calcular error: diferencia entre distancias laterales
    // Si dist_left > dist_right → carro se desvía a la izquierda
    // Si dist_left < dist_right → carro se desvía a la derecha
    float error = dist_left - dist_right;
    
    // Calcular corrección PID
    float correction = pid_compute(&pid_controller, error);
    
    // Aplicar corrección a las velocidades de los motores
    int speed_left = BASE_SPEED + (int)correction;
    int speed_right = BASE_SPEED - (int)correction;
    
    // Saturar velocidades en rango válido [0, 100]
    if (speed_left < 0) speed_left = 0;
    if (speed_left > 100) speed_left = 100;
    if (speed_right < 0) speed_right = 0;
    if (speed_right > 100) speed_right = 100;
    
    // Enviar comandos a los motores (dirección 1 = adelante)
    l298n_set_motor(&motor_driver, 0, speed_left, 1);   // Motor A (izquierdo)
    l298n_set_motor(&motor_driver, 1, speed_right, 1);  // Motor B (derecho)
    
    // Debug opcional (comentar para reducir tráfico serial)
    // printf("[NAV] L:%.1fcm R:%.1fcm Err:%.2f Corr:%.2f SpdL:%d SpdR:%d\n",
    //        dist_left, dist_right, error, correction, speed_left, speed_right);
}

/* ========== TAREA DE MUESTREO AMBIENTAL ========== */

void sampling_task(void) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    // Verificar si ha pasado el intervalo de muestreo (1 segundo)
    if ((current_time - g_last_sample_time) >= SAMPLE_INTERVAL_MS) {
        g_last_sample_time = current_time;
        
        // Leer sensores ambientales (ADC, polling)
        float temperature = lm35_read_celsius(&lm35);
        float light = ldr_read_percent(&ldr);
        
        // Crear registro ambiental
        EnvironmentalRecord record;
        record.temperature = temperature;
        record.light = light;
        record.sample_id = 0;  // Se asigna automáticamente en eeprom_store_sample()
        
        // Almacenar en EEPROM
        eeprom_error_t err = eeprom_store_sample(&record);
        
        if (err == EEPROM_SUCCESS) {
            printf("[SAMPLE] Temp: %.2f°C | Luz: %.1f%% | Total: %d registros\n",
                   temperature, light, eeprom_get_record_count());
        } else {
            printf("[ERROR] Fallo al guardar muestra: %d\n", err);
        }
    }
}

/* ========== CONSOLA DE COMANDOS ========== */

void console_task(void) {
    // Verificar si hay datos disponibles en stdin (USB/UART)
    int c = getchar_timeout_us(0);  // Non-blocking
    
    if (c == PICO_ERROR_TIMEOUT) {
        return;  // No hay datos disponibles
    }
    
    // Buffer estático para acumular comando
    static char cmd_buffer[32];
    static uint8_t cmd_index = 0;
    
    // Acumular caracteres hasta newline o CR
    if (c == '\n' || c == '\r') {
        if (cmd_index > 0) {
            cmd_buffer[cmd_index] = '\0';  // Null-terminate
            
            // Procesar comando
            if (strcmp(cmd_buffer, "DATA") == 0) {
                command_data();
            } else if (strcmp(cmd_buffer, "CLEAR") == 0) {
                command_clear();
            } else {
                printf("Comando desconocido: '%s'\n", cmd_buffer);
                printf("Comandos válidos: DATA, CLEAR\n");
            }
            
            // Resetear buffer
            cmd_index = 0;
        }
    } else if (cmd_index < (sizeof(cmd_buffer) - 1)) {
        // Acumular carácter (convertir a mayúscula)
        if (c >= 'a' && c <= 'z') {
            c = c - 'a' + 'A';
        }
        cmd_buffer[cmd_index++] = (char)c;
    }
}

void command_data(void) {
    printf("\n========== COMANDO: DATA ==========\n");
    
    uint16_t count = eeprom_get_record_count();
    
    if (count == 0) {
        printf("No hay registros almacenados.\n\n");
        return;
    }
    
    // Reservar buffer para leer todas las muestras
    EnvironmentalRecord *buffer = (EnvironmentalRecord *)malloc(count * sizeof(EnvironmentalRecord));
    
    if (buffer == NULL) {
        printf("ERROR: No hay memoria suficiente para leer %d registros\n\n", count);
        return;
    }
    
    // Leer todas las muestras desde EEPROM
    uint16_t read_count = 0;
    eeprom_error_t err = eeprom_read_all_samples(buffer, &read_count);
    
    if (err != EEPROM_SUCCESS) {
        printf("ERROR al leer EEPROM: %d\n\n", err);
        free(buffer);
        return;
    }
    
    // Imprimir todas las muestras
    printf("Total de registros: %d\n", read_count);
    printf("-----------------------------------\n");
    printf("ID  | Temp (°C) | Luz (%%)\n");
    printf("-----------------------------------\n");
    
    for (uint16_t i = 0; i < read_count; i++) {
        printf("%-3d | %8.2f | %6.1f\n",
               buffer[i].sample_id,
               buffer[i].temperature,
               buffer[i].light);
    }
    
    printf("===================================\n\n");
    
    // Liberar memoria
    free(buffer);
}

void command_clear(void) {
    printf("\n========== COMANDO: CLEAR ==========\n");
    
    eeprom_error_t err = eeprom_clear();
    
    if (err == EEPROM_SUCCESS) {
        printf("Buffer de EEPROM limpiado correctamente.\n");
        printf("Registros actuales: %d\n\n", eeprom_get_record_count());
    } else {
        printf("ERROR al limpiar EEPROM: %d\n\n", err);
    }
}
