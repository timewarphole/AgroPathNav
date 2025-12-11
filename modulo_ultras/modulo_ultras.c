/**
 * @file modulo_ultras.c
 * @brief Sistema completo de control agrícola autónomo con almacenamiento persistente
 * @details Integra:
 *          - Control PID diferencial con ultrasonidos HC-SR04 para centrado lateral
 *          - Sensores ambientales LM35 (temperatura) + LDR (luz)
 *          - Almacenamiento automático en EEPROM 24LC128
 *          - Botón de inicio/parada con antirrebote 
 *          - Comandos seriales DATA/CLEAN/INFO/HELP para extracción de datos
 * 
 * © 2025 - AgroPath Navigation System
 */

/** @cond Includes necesarios */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hcsr04.h"
#include "pid.h"
#include "l298n.h"
#include "lm35.h"
#include "fotoresistencia.h"
#include "eeprom.h"
/** @endcond */

/**
 * @defgroup PIN_DEFINITIONS Definiciones de Pines
 * @{
 */

/** @{ Pines Motor A (Izquierdo) L298N */
#define MA_IN1_PIN  9   /**< Control dirección IN1 Motor A */
#define MA_IN2_PIN  10  /**< Control dirección IN2 Motor A */
#define MA_ENA_PIN  8   /**< PWM velocidad ENA Motor A */
/** @} */

/** @{ Pines Motor B (Derecho) L298N */
#define MB_IN3_PIN  11  /**< Control dirección IN3 Motor B */
#define MB_IN4_PIN  14  /**< Control dirección IN4 Motor B */
#define MB_ENB_PIN  15  /**< PWM velocidad ENB Motor B */
/** @} */

/** @brief Pin botón inicio/parada (activo bajo con pull-up interno) */
#define START_BUTTON_PIN  22

/** @} */

/**
 * @defgroup TIMING_CONSTANTS Constantes de Temporización
 * @{
 */

/** @brief Duración total de operación autónoma: 10 segundos */
#define RUN_TIME_MS              10000

/** @brief Frecuencia de muestreo ambiental: 10 Hz (100ms intervalo) */
#define EEPROM_SAVE_INTERVAL_MS  100

/** @brief Buffer para comandos seriales */
#define CMD_BUFFER_SIZE          16

/** @brief Tiempo de antirrebote del botón: 200ms */
#define DEBOUNCE_TIME_MS         200

/** @} */

/**
 * @defgroup GLOBAL_VARIABLES Variables Globales
 * @{
 */

/** @{ Variables de control del sistema */
static bool        system_running    = false;  /**< Estado del sistema (true=operando) */
static bool        last_button_state = true;   /**< Estado anterior del botón */
static uint32_t    last_button_time  = 0;      /**< Timestamp última pulsación botón */
static uint32_t    run_start_time    = 0;      /**< Inicio del ciclo de 10s */

/** @{ Variables de muestreo EEPROM */
static uint32_t    last_eeprom_save_time = 0;  /**< Último guardado en EEPROM */
static uint16_t    sample_counter        = 0;  /**< Contador total de muestras */

/** @{ Buffer comandos seriales */
static char        cmd_buffer[CMD_BUFFER_SIZE]; /**< Buffer de comandos */
static uint8_t     cmd_index         = 0;       /**< Índice actual del buffer */

/** @} */

/**
 * @brief Lee el botón con detección de flanco y antirrebote por software
 * @details Detecta transición HIGH→LOW (presión) con debounce de 200ms
 * @note Pull-up interno: HIGH=reposo, LOW=presionado
 */
void check_button(void) {
    bool current_state = gpio_get(START_BUTTON_PIN);
    
    /** Detectar flanco descendente (presión del botón) */
    if (last_button_state == true && current_state == false) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        /** Antirrebote: validar intervalo mínimo de 200ms */
        if ((now - last_button_time) >= DEBOUNCE_TIME_MS) {
            last_button_time = now;
            
            if (!system_running) {
                /** Iniciar ciclo de 10 segundos */
                system_running        = true;
                run_start_time        = now;
                last_eeprom_save_time = now;
                sample_counter        = 0;
                printf("\n[BOTON] Sistema INICIADO - 10 segundos de operacion\n\n");
            } else {
                /** Detener inmediatamente */
                system_running = false;
                printf("\n[BOTON] Sistema DETENIDO por el usuario\n");
                printf("[EEPROM] Muestras guardadas: %d\n\n", sample_counter);
            }
        }
    }
    
    last_button_state = current_state;
}

/**
 * @brief Verifica timeout automático de 10 segundos de operación
 * @details Detiene el sistema cuando se cumple RUN_TIME_MS desde run_start_time
 */
void check_timeout(void) {
    if (system_running) {
        uint32_t now    = to_ms_since_boot(get_absolute_time());
        uint32_t elapsed = now - run_start_time;
        
        if (elapsed >= RUN_TIME_MS) {
            system_running = false;
            printf("\n[TIMEOUT] 10 segundos cumplidos - Sistema DETENIDO\n");
            printf("[EEPROM] Muestras guardadas: %d\n\n", sample_counter);
        }
    }
}

/**
 * @brief Guarda muestra ambiental (Temp + Luz) en EEPROM cada 100ms
 * @param temp_sensor Puntero al sensor LM35
 * @param light_sensor Puntero al sensor LDR
 * @details 
 *  - Frecuencia: 10 Hz (10 muestras/segundo)
 *  - Retry: 3 intentos si I2C falla
 *  - Valores inválidos (<0): Guarda 0.0
 *  - sample_id: Secuencial desde 0
 */
void save_environmental_data(LM35_Sensor *temp_sensor, LDR_Sensor *light_sensor) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    /** Verificar intervalo de muestreo (100ms) */
    if ((now - last_eeprom_save_time) >= EEPROM_SAVE_INTERVAL_MS) {
        last_eeprom_save_time = now;
        
        /** Leer sensores ambientales */
        float temperature = lm35_read_celsius(temp_sensor);
        float light       = ldr_read_percent(light_sensor);
        
        /** Crear registro con validación */
        EnvironmentalRecord record;
        record.temperature = (temperature >= 0.0f) ? temperature : 0.0f;
        record.light       = (light       >= 0.0f) ? light       : 0.0f;
        record.sample_id   = sample_counter;
        
        /** Guardar con retry (3 intentos) */
        eeprom_error_t result = eeprom_store_sample(&record);
        uint8_t retries = 3;
        while (result != EEPROM_SUCCESS && retries > 0) {
            sleep_ms(10);
            result = eeprom_store_sample(&record);
            retries--;
        }
        
        if (result == EEPROM_SUCCESS) {
            sample_counter++;
        } else {
            printf("[EEPROM] Error persistente (codigo: %d)\n", result);
        }
    }
}

/**
 * @brief Muestra estado completo de la memoria EEPROM
 * @details Capacidad, ocupación, espacio libre y tamaño de registro
 */
void show_eeprom_info(void) {
    uint16_t capacity    = eeprom_get_capacity();
    uint16_t used        = eeprom_get_record_count();
    uint16_t available   = capacity - used;
    float percent_used   = (used * 100.0f) / capacity;
    
    printf("\n========== ESTADO DE MEMORIA EEPROM ==========\n");
    printf("Capacidad total:   %d registros\n", capacity);
    printf("Registros usados:  %d (%.1f%%)\n", used, percent_used);
    printf("Espacio libre:     %d registros\n", available);
    printf("Tamano registro:   %d bytes\n", (int)sizeof(EnvironmentalRecord));
    printf("===============================================\n\n");
}

/**
 * @brief Extrae y muestra TODOS los datos almacenados (comando DATA)
 * @details Formato tabular: ID | Temperatura | Luz
 */
void show_all_data(void) {
    uint16_t count = eeprom_get_record_count();
    
    if (count == 0) {
        printf("\n[EEPROM] No hay datos almacenados\n\n");
        return;
    }
    
    printf("\n========== DATOS ALMACENADOS (DATA) ==========\n");
    printf("ID  | Temp(°C) | Luz(%%)\n");
    printf("----+----------+--------\n");
    
    EnvironmentalRecord buffer[EEPROM_MAX_RECORDS];
    uint16_t read_count = 0;
    
    eeprom_error_t result = eeprom_read_all_samples(buffer, &read_count);
    
    if (result == EEPROM_SUCCESS && read_count > 0) {
        for (uint16_t i = 0; i < read_count; i++) {
            printf("%3d | %8.2f | %6.1f\n",
                   buffer[i].sample_id,
                   buffer[i].temperature,
                   buffer[i].light);
        }
        printf("==============================================\n");
        printf("Total de muestras: %d\n", read_count);
    } else {
        printf("[ERROR] Fallo al leer datos (codigo: %d)\n", result);
    }
    
    show_eeprom_info();
}

/**
 * @brief Procesador de comandos seriales no bloqueante
 * @details Comandos soportados:
 *          - DATA: Mostrar todos los datos almacenados
 *          - CLEAN: Borrar EEPROM completamente
 *          - INFO: Estado de memoria
 *          - HELP: Lista de comandos
 */
void process_serial_commands(void) {
    int c = getchar_timeout_us(10000);  /* 10ms timeout no bloqueante */
    
    if (c == PICO_ERROR_TIMEOUT) {
        return;
    }
    
    /** Convertir a mayúsculas */
    if (c >= 'a' && c <= 'z') {
        c = c - 32;
    }
    
    /** Procesar Enter */
    if (c == '\r' || c == '\n') {
        if (cmd_index > 0) {
            cmd_buffer[cmd_index] = '\0';
            
            /** Routing de comandos */
            if (strcmp(cmd_buffer, "DATA") == 0) {
                show_all_data();
            }
            else if (strcmp(cmd_buffer, "CLEAN") == 0) {
                eeprom_clear();
                printf("\n[EEPROM] Memoria borrada completamente\n");
                show_eeprom_info();
            }
            else if (strcmp(cmd_buffer, "INFO") == 0) {
                show_eeprom_info();
            }
            else if (strcmp(cmd_buffer, "HELP") == 0) {
                printf("\n========== COMANDOS ==========\n");
                printf("DATA  - Mostrar datos\n");
                printf("CLEAN - Borrar EEPROM\n");
                printf("INFO  - Estado memoria\n");
                printf("HELP  - Esta ayuda\n");
                printf("============================\n\n");
            }
            else {
                printf("\nComando desconocido: '%s'\nEscribe HELP\n\n", cmd_buffer);
            }
            
            cmd_index = 0;
        }
    }
    /** Procesar caracteres imprimibles */
    else if (c >= 32 && c <= 126) {
        if (cmd_index < CMD_BUFFER_SIZE - 1) {
            cmd_buffer[cmd_index++] = (char)c;
            putchar(c);  /* Echo inmediato */
        }
    }
    /** Backspace */
    else if (c == 127 || c == 8) {
        if (cmd_index > 0) {
            cmd_index--;
            printf("\b \b");
        }
    }
}

/**
 * @brief Función principal del sistema
 * @return 0 en éxito (nunca retorna en operación normal)
 * @details Secuencia completa:
 *          1. Inicialización hardware (ADC, GPIO, sensores, EEPROM)
 *          2. Bucle principal no bloqueante:
 *             - Comandos seriales (DATA/CLEAN)
 *             - Polling botón inicio/parada
 *             - Timeout automático 10s
 *             - Muestreo ambiental 10Hz → EEPROM
 
 */
int main(void) {
    /** Inicialización básica */
    stdio_init_all();
    sleep_ms(2000);  /* Estabilización USB */
    
    /** Banner de bienvenida */
    printf("\n=====================================================\n");
    printf("   Sistema Agricola - Modo Baterias + PC DATA\n");
    printf("=====================================================\n\n");

    /** @subsection init_adc Inicialización ADC */
    adc_init();
    adc_set_temp_sensor_enabled(true);
    printf("[ADC] Inicializado\n");

    /** @subsection init_button Inicialización botón */
    gpio_init(START_BUTTON_PIN);
    gpio_set_dir(START_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(START_BUTTON_PIN);
    printf("[BOTON] GP%d listo\n", START_BUTTON_PIN);

    /** @subsection init_sensors Inicialización sensores ambientales */
    LM35_Sensor temp_sensor;
    LDR_Sensor light_sensor;
    
    lm35_init(&temp_sensor, LM35_ADC_CHANNEL, LM35_GPIO_PIN);
    ldr_init(&light_sensor, LDR_ADC_CHANNEL, LDR_GPIO_PIN);
    printf("[SENSORS] LM35+ LDR listos\n");

    /** @subsection init_eeprom Inicialización EEPROM crítica */
    printf("[EEPROM] Inicializando...");
    eeprom_error_t status = eeprom_init();
    if (status == EEPROM_SUCCESS) {
        printf(" OK\n");
        show_eeprom_info();
    } else {
        printf(" FALLO (codigo %d)\n", status);
    }

    /** @subsection init_control Inicialización subsistema de control */
    HCSR04_Sensor sensor_left, sensor_right;
    hcsr04_init(&sensor_left,  HCSR04_LEFT_TRIG_PIN,  HCSR04_LEFT_ECHO_PIN);
    hcsr04_init(&sensor_right, HCSR04_RIGHT_TRIG_PIN, HCSR04_RIGHT_ECHO_PIN);
    hcsr04_register_sensor(&sensor_left);
    hcsr04_register_sensor(&sensor_right);

    L298N_Driver motors;
    l298n_init(&motors, MA_IN1_PIN, MA_IN2_PIN, MA_ENA_PIN,
                      MB_IN3_PIN, MB_IN4_PIN, MB_ENB_PIN);

    PID_Controller pid;
    /** Ganancias PID optimizadas para centrado lateral */
    pid_init(&pid, 4.5f, 0.12f, 1.0f, 0.0f, 60.0f, -100.0f, 100.0f);

    /** Parámetros motores */
    const int base_speed = 50, min_speed = 20, max_speed = 70;

    /** Mensaje de instrucciones */
    printf("\n[SISTEMA] Boton para 10s | DATA/CLEAN/INFO/HELP\n\n");

    /** @subsection main_loop Bucle principal no bloqueante */
    while (true) {
        /** Procesamiento comandos seriales (no bloqueante) */
        process_serial_commands();
        
        /** Polling botón inicio/parada */
        check_button();
        
        /** Verificación timeout automático */
        check_timeout();

        /** Modo detenido: motores OFF */
        if (!system_running) {
            l298n_stop_all(&motors);
            sleep_ms(50);
            continue;
        }

        /** Muestreo ambiental → EEPROM (10 Hz) */
        save_environmental_data(&temp_sensor, &light_sensor);

        /** Control PID diferencial */
        hcsr04_trigger(&sensor_left);
        sleep_us(10);  /* Evitar crosstalk ultrasónicos */
        hcsr04_trigger(&sensor_right);

        /** Polling no bloqueante de ecos (timeout 30ms) */
        uint32_t start_time = time_us_32();
        bool left_ready = false, right_ready = false;
        while ((time_us_32() - start_time) < 30000) {
            if (!left_ready  && hcsr04_is_ready(&sensor_left))  left_ready  = true;
            if (!right_ready && hcsr04_is_ready(&sensor_right)) right_ready = true;
            if (left_ready && right_ready) break;
            tight_loop_contents();
        }

        /** Lectura distancias con validación */
        float d_left  = hcsr04_read_distance_cm(&sensor_left);
        float d_right = hcsr04_read_distance_cm(&sensor_right);
        if (d_left  <= 0.0f) d_left  = 0.0f;
        if (d_right <= 0.0f) d_right = 0.0f;

        /** Cálculo error lateral y corrección PID */
        float error     = d_left - d_right;
        float correction = pid_compute(&pid, error);

        /** Velocidades diferenciales */
        int speed_left  = (base_speed - (int)correction);
        int speed_right = (base_speed + (int)correction) + 15;
        
        /** Saturación velocidades */
        speed_left  = speed_left  < min_speed ? min_speed : (speed_left  > max_speed ? max_speed : speed_left);
        speed_right = speed_right < min_speed ? min_speed : (speed_right > max_speed ? max_speed : speed_right);

        /** Aplicar comandos motores */
        l298n_set_motor(&motors, 0, speed_left,  1); /* Motor izquierdo */
        l298n_set_motor(&motors, 1, speed_right, 1); /* Motor derecho */

        /** Frecuencia de control: 20 Hz */
        sleep_ms(50);
    }

    return 0; 
}


