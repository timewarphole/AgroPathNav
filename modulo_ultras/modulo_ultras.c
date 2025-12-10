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

// Pines de los motores
#define MA_IN1_PIN  9   
#define MA_IN2_PIN  10  
#define MA_ENA_PIN  8   

#define MB_IN3_PIN  11  
#define MB_IN4_PIN  14  
#define MB_ENB_PIN  15  

// Pin del botón
#define START_BUTTON_PIN  22

// Tiempo de operación (10 segundos)
#define RUN_TIME_MS  10000

// Intervalo de guardado en EEPROM (100ms = 10 muestras/seg)
#define EEPROM_SAVE_INTERVAL_MS  100

// Buffer para comandos seriales
#define CMD_BUFFER_SIZE 16

// ----------------------------------------------------------
// Variables globales para el botón (polling)
// ----------------------------------------------------------
bool system_running = false;
bool last_button_state = true;
uint32_t last_button_time = 0;
uint32_t run_start_time = 0;
const uint32_t DEBOUNCE_TIME_MS = 200;

// ----------------------------------------------------------
// Variables para guardado periódico en EEPROM
// ----------------------------------------------------------
uint32_t last_eeprom_save_time = 0;
uint16_t sample_counter = 0;

// ----------------------------------------------------------
// Buffer para comandos
// ----------------------------------------------------------
char cmd_buffer[CMD_BUFFER_SIZE];
uint8_t cmd_index = 0;

// ----------------------------------------------------------
// Función para leer botón con antirrebote (polling)
// ----------------------------------------------------------
void check_button() {
    bool current_state = gpio_get(START_BUTTON_PIN);
    
    if (last_button_state == true && current_state == false) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        if ((now - last_button_time) >= DEBOUNCE_TIME_MS) {
            last_button_time = now;
            
            if (!system_running) {
                system_running = true;
                run_start_time = now;
                last_eeprom_save_time = now;
                sample_counter = 0;
                printf("\n[BOTON] Sistema INICIADO - 10 segundos de operacion\n\n");
            } else {
                system_running = false;
                printf("\n[BOTON] Sistema DETENIDO por el usuario\n");
                printf("[EEPROM] Muestras guardadas: %d\n\n", sample_counter);
            }
        }
    }
    
    last_button_state = current_state;
}

// ----------------------------------------------------------
// Verificar timeout de 10 segundos
// ----------------------------------------------------------
void check_timeout() {
    if (system_running) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        uint32_t elapsed = now - run_start_time;
        
        if (elapsed >= RUN_TIME_MS) {
            system_running = false;
            printf("\n[TIMEOUT] 10 segundos cumplidos - Sistema DETENIDO\n");
            printf("[EEPROM] Muestras guardadas: %d\n\n", sample_counter);
        }
    }
}

// ----------------------------------------------------------
// Guardar SOLO temperatura y luz en EEPROM (forzada)
// ----------------------------------------------------------
// En main.c

void save_environmental_data(LM35_Sensor *temp_sensor, LDR_Sensor *light_sensor) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    if ((now - last_eeprom_save_time) >= EEPROM_SAVE_INTERVAL_MS) {
        last_eeprom_save_time = now;
        
        // Leer sensores
        float temperature = lm35_read_celsius(temp_sensor);
        float light = ldr_read_percent(light_sensor);
        
        // Crear registro
        EnvironmentalRecord record;
        record.temperature = (temperature >= 0.0f) ? temperature : 0.0f;
        record.light = (light >= 0.0f) ? light : 0.0f;
        

        record.sample_id = 0; 
        // ---------------------------------------------------------
        
        // Intentar guardar (con reintentos)
        eeprom_error_t result = eeprom_store_sample(&record);
        uint8_t retries = 3;
        while (result != EEPROM_SUCCESS && retries > 0) {
            sleep_ms(10);
            result = eeprom_store_sample(&record);
            retries--;
        }
        
        if (result == EEPROM_SUCCESS) {
            // Este contador es SOLO para que sepas cuántas llevas en ESTA ejecución.
            // No es el ID que se guarda en la memoria.
            sample_counter++; 
        } else {
            printf("[EEPROM] Error persistente (codigo: %d)\n", result);
        }
    }
}

// ----------------------------------------------------------
// Mostrar información de memoria EEPROM
// ----------------------------------------------------------
void show_eeprom_info() {
    uint16_t capacity = eeprom_get_capacity();
    uint16_t used = eeprom_get_record_count();
    uint16_t available = capacity - used;
    float percent_used = (used * 100.0f) / capacity;
    
    printf("\n========== ESTADO DE MEMORIA EEPROM ==========\n");
    printf("Capacidad total:   %d registros\n", capacity);
    printf("Registros usados:  %d (%.1f%%)\n", used, percent_used);
    printf("Espacio libre:     %d registros\n", available);
    printf("Tamano registro:   %d bytes\n", (int)sizeof(EnvironmentalRecord));
    printf("===============================================\n\n");
}

// ----------------------------------------------------------
// Mostrar todos los datos almacenados
// ----------------------------------------------------------
void show_all_data() {
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

// ----------------------------------------------------------
// Procesar comandos por serial
// ----------------------------------------------------------
void process_serial_commands() {
    int c = getchar_timeout_us(10000);  // 10ms timeout
    
    if (c == PICO_ERROR_TIMEOUT) {
        return;
    }
    
    if (c >= 'a' && c <= 'z') {
        c = c - 32;
    }
    
    if (c == '\r' || c == '\n') {
        if (cmd_index > 0) {
            cmd_buffer[cmd_index] = '\0';
            
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
    else if (c >= 32 && c <= 126) {
        if (cmd_index < CMD_BUFFER_SIZE - 1) {
            cmd_buffer[cmd_index++] = (char)c;
            putchar(c);
        }
    }
    else if (c == 127 || c == 8) {
        if (cmd_index > 0) {
            cmd_index--;
            printf("\b \b");
        }
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000);  // Más tiempo para USB

    printf("\n=====================================================\n");
    printf("   Sistema Agricola - Modo Baterias + PC DATA\n");
    printf("=====================================================\n\n");

    // ----------------------------------------------------------
    // Inicialización ADC
    // ----------------------------------------------------------
    adc_init();
    adc_set_temp_sensor_enabled(true);
    printf("[ADC] Inicializado\n");

    // ----------------------------------------------------------
    // Botón
    // ----------------------------------------------------------
    gpio_init(START_BUTTON_PIN);
    gpio_set_dir(START_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(START_BUTTON_PIN);
    printf("[BOTON] GP%d listo\n", START_BUTTON_PIN);

    // ----------------------------------------------------------
    // Sensores ambientales
    // ----------------------------------------------------------
    LM35_Sensor temp_sensor;
    LDR_Sensor light_sensor;
    
    lm35_init(&temp_sensor, LM35_ADC_CHANNEL, LM35_GPIO_PIN);
    ldr_init(&light_sensor, LDR_ADC_CHANNEL, LDR_GPIO_PIN);
    printf("[SENSORS] LM35+ LDR listos\n");

    // ----------------------------------------------------------
    // EEPROM (crítico para baterías)
    // ----------------------------------------------------------
    printf("[EEPROM] Inicializando...");
    eeprom_error_t status = eeprom_init();
    if (status == EEPROM_SUCCESS) {
        printf(" OK\n");
        show_eeprom_info();
    } else {
        printf(" FALLO (codigo %d)\n", status);
    }

    // ----------------------------------------------------------
    // Control (HC-SR04 + Motores + PID)
    // ----------------------------------------------------------
    HCSR04_Sensor sensor_left, sensor_right;
    hcsr04_init(&sensor_left, HCSR04_LEFT_TRIG_PIN, HCSR04_LEFT_ECHO_PIN);
    hcsr04_init(&sensor_right, HCSR04_RIGHT_TRIG_PIN, HCSR04_RIGHT_ECHO_PIN);
    hcsr04_register_sensor(&sensor_left);
    hcsr04_register_sensor(&sensor_right);

    L298N_Driver motors;
    l298n_init(&motors, MA_IN1_PIN, MA_IN2_PIN, MA_ENA_PIN,
                      MB_IN3_PIN, MB_IN4_PIN, MB_ENB_PIN);

    PID_Controller pid;
    pid_init(&pid, 4.5f, 0.12f, 1.0f, 0.0f, 60.0f, -100.0f, 100.0f);

    const int base_speed = 50, min_speed = 20, max_speed = 70;

    printf("\n[SISTEMA] Boton para 10s | DATA/CLEAN/INFO/HELP\n\n");

    // ----------------------------------------------------------
    // Bucle principal optimizado
    // ----------------------------------------------------------
    while (true) {
        process_serial_commands();
        check_button();
        check_timeout();

        if (!system_running) {
            l298n_stop_all(&motors);
            sleep_ms(50);  // ← SIN pid_reset() para no interferir
            continue;
        }

        // Guardar datos ambientales
        save_environmental_data(&temp_sensor, &light_sensor);

        // Control silencioso PID
        hcsr04_trigger(&sensor_left);
        sleep_us(10);
        hcsr04_trigger(&sensor_right);

        uint32_t start_time = time_us_32();
        bool left_ready = false, right_ready = false;
        while ((time_us_32() - start_time) < 30000) {
            if (!left_ready && hcsr04_is_ready(&sensor_left)) left_ready = true;
            if (!right_ready && hcsr04_is_ready(&sensor_right)) right_ready = true;
            if (left_ready && right_ready) break;
            tight_loop_contents();
        }

        float d_left = hcsr04_read_distance_cm(&sensor_left);
        float d_right = hcsr04_read_distance_cm(&sensor_right);
        if (d_left <= 0.0f) d_left = 0.0f;
        if (d_right <= 0.0f) d_right = 0.0f;

        float error = d_left - d_right;
        float correction = pid_compute(&pid, error);

        int speed_left = (base_speed - (int)correction);
        int speed_right = (base_speed + (int)correction) + 15;
        
        speed_left = speed_left < min_speed ? min_speed : (speed_left > max_speed ? max_speed : speed_left);
        speed_right = speed_right < min_speed ? min_speed : (speed_right > max_speed ? max_speed : speed_right);

        l298n_set_motor(&motors, 0, speed_left, 1);
        l298n_set_motor(&motors, 1, speed_right, 1);

        sleep_ms(50);
    }

    return 0;
}
