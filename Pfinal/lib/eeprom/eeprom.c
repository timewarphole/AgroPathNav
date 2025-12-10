/**
 * @file eeprom.c
 * @brief Implementación del driver EEPROM para muestras ambientales
 */

#include "eeprom.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <string.h>

/* ========== VARIABLES GLOBALES DEL BUFFER CIRCULAR ========== */

/** @brief Índice donde se escribirá el próximo registro (0 a MAX_RECORDS-1) */
static uint16_t g_write_index = 0;

/** @brief Número actual de registros almacenados en el buffer */
static uint16_t g_record_count = 0;

/** @brief Indicador de buffer circular completamente lleno */
static bool g_is_full = false;

/** @brief Contador global de muestras (para sample_id) */
static uint16_t g_sample_counter = 0;

/* ========== ESTRUCTURA DE METADATOS ========== */

/**
 * @brief Estructura para persistencia de metadatos en EEPROM (8 bytes)
 */
typedef struct __attribute__((packed)) {
    uint16_t write_index;    /**< Índice de escritura circular */
    uint16_t record_count;   /**< Número de registros almacenados */
    uint32_t magic_number;   /**< Número mágico para validación de integridad */
} eeprom_metadata_t;

/* ========== FUNCIONES PRIVADAS (HELPERS) ========== */

/**
 * @brief Verifica si la EEPROM está lista mediante ACK polling
 */
static bool eeprom_check_ready(void) {
    uint8_t dummy = 0;
    int result = i2c_write_blocking(EEPROM_I2C_ID, EEPROM_DEV_ADDR, 
                                    &dummy, 0, false);
    return (result >= 0);
}

/**
 * @brief Espera a que la EEPROM complete el ciclo de escritura interno
 */
static eeprom_error_t eeprom_wait_write_complete(uint32_t timeout_ms) {
    /* Delay inicial obligatorio: la EEPROM necesita tiempo para write cycle */
    sleep_ms(6);
    
    /* Ahora comenzar a contar el timeout */
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    
    while (!eeprom_check_ready()) {
        uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - start_time;
        if (elapsed >= timeout_ms) {
            return EEPROM_ERR_TIMEOUT;
        }
        sleep_ms(2);  // Polling cada 2ms 
    }
    
    return EEPROM_SUCCESS;
}

/* ========== FUNCIONES PÚBLICAS - INICIALIZACIÓN ========== */

eeprom_error_t eeprom_init(void) {
    /* Inicializar puerto I2C */
    i2c_init(EEPROM_I2C_ID, EEPROM_BAUD_RATE);
    
    /* Configurar pines GPIO para función I2C (GP16/17) */
    gpio_set_function(EEPROM_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(EEPROM_SCL_PIN, GPIO_FUNC_I2C);
    
    /* Habilitar resistencias pull-up internas */
    gpio_pull_up(EEPROM_SDA_PIN);
    gpio_pull_up(EEPROM_SCL_PIN);
    
    /* Pequeño delay para estabilización del bus */
    sleep_ms(10);
    
    /* Verificar presencia de EEPROM en el bus */
    if (!eeprom_check_ready()) {
        return EEPROM_ERR_I2C_FAIL;
    }
    
    /* Cargar metadatos persistidos desde EEPROM */
    return eeprom_load_metadata();
}

/* ========== FUNCIONES PÚBLICAS - BAJO NIVEL ========== */

eeprom_error_t eeprom_write_bytes(uint16_t address, const uint8_t *data, size_t length) {
    /* Validar puntero de datos */
    if (data == NULL) {
        return EEPROM_ERR_NULL_POINTER;
    }
    
    /* Validar longitud */
    if (length == 0) {
        return EEPROM_ERR_INVALID_SIZE;
    }
    
    /* Validar rango de dirección */
    if (address + length > EEPROM_TOTAL_BYTES) {
        return EEPROM_ERR_INVALID_ADDRESS;
    }
    
    size_t bytes_written = 0;
    
    /* Escribir respetando límites de página (64 bytes) */
    while (bytes_written < length) {
        uint16_t current_addr = address + bytes_written;
        
        /* Calcular cuántos bytes quedan hasta el final de la página actual */
        uint8_t page_offset = current_addr % EEPROM_PAGE_SIZE;
        uint8_t bytes_to_page_end = EEPROM_PAGE_SIZE - page_offset;
        
        /* Determinar cuántos bytes escribir en esta iteración */
        size_t bytes_to_write = length - bytes_written;
        if (bytes_to_write > bytes_to_page_end) {
            bytes_to_write = bytes_to_page_end;
        }
        
        /* Preparar buffer de transmisión: [addr_high][addr_low][data...] */
        uint8_t tx_buffer[EEPROM_PAGE_SIZE + 2];
        tx_buffer[0] = (uint8_t)(current_addr >> 8);    // MSB de dirección
        tx_buffer[1] = (uint8_t)(current_addr & 0xFF);  // LSB de dirección
        memcpy(&tx_buffer[2], &data[bytes_written], bytes_to_write);
        
        /* Transmitir por I2C */
        int result = i2c_write_blocking(EEPROM_I2C_ID, EEPROM_DEV_ADDR,
                                        tx_buffer, bytes_to_write + 2, false);
        
        if (result < 0) {
            return EEPROM_ERR_I2C_FAIL;
        }
        
        /* Esperar ciclo de escritura (20ms para estabilidad) */
        eeprom_error_t err = eeprom_wait_write_complete(20);
        if (err != EEPROM_SUCCESS) {
            return err;
        }
        
        bytes_written += bytes_to_write;
    }
    
    return EEPROM_SUCCESS;
}

eeprom_error_t eeprom_read_bytes(uint16_t address, uint8_t *data, size_t length) {
    /* Validar puntero de datos */
    if (data == NULL) {
        return EEPROM_ERR_NULL_POINTER;
    }
    
    /* Validar longitud */
    if (length == 0) {
        return EEPROM_ERR_INVALID_SIZE;
    }
    
    /* Validar rango de dirección */
    if (address + length > EEPROM_TOTAL_BYTES) {
        return EEPROM_ERR_INVALID_ADDRESS;
    }
    
    /* Preparar dirección de memoria (big-endian) */
    uint8_t addr_buffer[2];
    addr_buffer[0] = (uint8_t)(address >> 8);    // MSB
    addr_buffer[1] = (uint8_t)(address & 0xFF);  // LSB
    
    /* Fase 1: Enviar dirección con repeated start */
    int result = i2c_write_blocking(EEPROM_I2C_ID, EEPROM_DEV_ADDR,
                                    addr_buffer, 2, true);  // nostop=true
    
    if (result < 0) {
        return EEPROM_ERR_I2C_FAIL;
    }
    
    /* Fase 2: Leer datos en lectura secuencial */
    result = i2c_read_blocking(EEPROM_I2C_ID, EEPROM_DEV_ADDR,
                               data, length, false);
    
    if (result < 0 || (size_t)result != length) {
        return EEPROM_ERR_I2C_FAIL;
    }
    
    return EEPROM_SUCCESS;
}

eeprom_error_t eeprom_read_record(uint16_t record_index, EnvironmentalRecord *record) {
    /* Validar puntero */
    if (record == NULL) {
        return EEPROM_ERR_NULL_POINTER;
    }
    
    /* Validar índice de registro */
    if (record_index >= EEPROM_MAX_RECORDS) {
        return EEPROM_ERR_INVALID_ADDRESS;
    }
    
    /* Calcular dirección física en EEPROM */
    uint16_t address = record_index * EEPROM_RECORD_SIZE;
    
    /* Leer registro completo */
    return eeprom_read_bytes(address, (uint8_t *)record, EEPROM_RECORD_SIZE);
}

/* ========== FUNCIONES PÚBLICAS - ALMACENAMIENTO AMBIENTAL ========== */

eeprom_error_t eeprom_store_sample(const EnvironmentalRecord *record) {
    /* Validar puntero */
    if (record == NULL) {
        return EEPROM_ERR_NULL_POINTER;
    }
    
    /* Crear copia con sample_id actualizado */
    EnvironmentalRecord record_copy = *record;
    record_copy.sample_id = g_sample_counter++;
    
    /* Calcular dirección física del registro en buffer circular */
    uint16_t address = g_write_index * EEPROM_RECORD_SIZE;
    
    /* Escribir registro en EEPROM */
    eeprom_error_t err = eeprom_write_bytes(address, 
                                            (const uint8_t *)&record_copy,
                                            EEPROM_RECORD_SIZE);
    
    if (err != EEPROM_SUCCESS) {
        return err;
    }
    
    /* Actualizar índice circular con wrap-around automático */
    g_write_index = (g_write_index + 1) % EEPROM_MAX_RECORDS;
    
    /* Actualizar contador de registros */
    if (g_record_count < EEPROM_MAX_RECORDS) {
        g_record_count++;
    } else {
        g_is_full = true;
    }
    
    return EEPROM_SUCCESS;
}

eeprom_error_t eeprom_read_all_samples(EnvironmentalRecord *buffer, uint16_t *count) {
    /* Validar punteros */
    if (buffer == NULL || count == NULL) {
        return EEPROM_ERR_NULL_POINTER;
    }
    
    /* Verificar que existan registros */
    if (g_record_count == 0) {
        *count = 0;
        return EEPROM_ERR_BUFFER_EMPTY;
    }
    
    /* Calcular índice de inicio (el más antiguo) */
    uint16_t start_index;
    uint16_t n = g_record_count;
    
    if (g_is_full) {
        /* Buffer lleno: el más antiguo está justo después de write_index */
        start_index = g_write_index;
    } else {
        /* Buffer parcial: empezar desde el índice 0 */
        start_index = 0;
    }
    
    /* Leer registros manejando wrap-around del buffer circular */
    for (uint16_t i = 0; i < n; i++) {
        uint16_t index = (start_index + i) % EEPROM_MAX_RECORDS;
        
        eeprom_error_t err = eeprom_read_record(index, &buffer[i]);
        if (err != EEPROM_SUCCESS) {
            *count = i;  // Retornar cuántos se leyeron antes del error
            return err;
        }
    }
    
    *count = n;
    return EEPROM_SUCCESS;
}

eeprom_error_t eeprom_clear(void) {
    /* Resetear variables del buffer circular */
    g_write_index = 0;
    g_record_count = 0;
    g_is_full = false;
    g_sample_counter = 0;
    
    /* Persistir metadatos limpios en EEPROM */
    return eeprom_save_metadata();
}

/* ========== FUNCIONES PÚBLICAS - METADATOS ========== */

eeprom_error_t eeprom_save_metadata(void) {
    eeprom_metadata_t meta;
    
    /* Preparar estructura de metadatos */
    meta.write_index = g_write_index;
    meta.record_count = g_record_count;
    meta.magic_number = EEPROM_MAGIC_NUMBER;
    
    /* Escribir en los últimos 8 bytes de la EEPROM */
    return eeprom_write_bytes(EEPROM_METADATA_ADDR,
                              (const uint8_t *)&meta,
                              sizeof(eeprom_metadata_t));
}

eeprom_error_t eeprom_load_metadata(void) {
    eeprom_metadata_t meta;
    
    /* Leer metadatos desde EEPROM */
    eeprom_error_t err = eeprom_read_bytes(EEPROM_METADATA_ADDR,
                                           (uint8_t *)&meta,
                                           sizeof(eeprom_metadata_t));
    
    if (err != EEPROM_SUCCESS) {
        /* Error de lectura: inicializar desde cero */
        g_write_index = 0;
        g_record_count = 0;
        g_is_full = false;
        g_sample_counter = 0;
        return eeprom_save_metadata();
    }
    
    /* Validar magic number para detectar corrupción */
    if (meta.magic_number != EEPROM_MAGIC_NUMBER) {
        /* Metadatos corruptos o primera vez: inicializar */
        g_write_index = 0;
        g_record_count = 0;
        g_is_full = false;
        g_sample_counter = 0;
        return eeprom_save_metadata();
    }
    
    /* Validar rangos de valores */
    if (meta.write_index >= EEPROM_MAX_RECORDS || 
        meta.record_count > EEPROM_MAX_RECORDS) {
        /* Datos inválidos: reinicializar */
        g_write_index = 0;
        g_record_count = 0;
        g_is_full = false;
        g_sample_counter = 0;
        return eeprom_save_metadata();
    }
    
    /* Metadatos válidos: cargar valores en variables globales */
    g_write_index = meta.write_index;
    g_record_count = meta.record_count;
    g_is_full = (meta.record_count >= EEPROM_MAX_RECORDS);
    g_sample_counter = meta.record_count;  // Continuar desde el último count
    
    return EEPROM_SUCCESS;
}

/* ========== FUNCIONES PÚBLICAS - GETTERS ========== */

uint16_t eeprom_get_record_count(void) {
    return g_record_count;
}

uint16_t eeprom_get_capacity(void) {
    return EEPROM_MAX_RECORDS;
}

uint16_t eeprom_get_write_index(void) {
    return g_write_index;
}

bool eeprom_is_full(void) {
    return g_is_full;
}