/**
 * @file eeprom.h
 * @brief Driver para EEPROM 24LC128 con buffer circular para muestras ambientales
 * @details Sistema de almacenamiento persistente con:
 *          - Buffer circular automático (wrap-around)
 *          - Metadatos persistentes (write_index, count)
 *          - Comandos DATA (leer N muestras) y CLEAR (limpiar buffer)
 * 
 * @copyright Copyright (c) 2025
 */

#ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "hardware/i2c.h"

/* ========== CONFIGURACIÓN DE HARDWARE ========== */

/** @brief Puerto I2C utilizado */
#define EEPROM_I2C_ID i2c0

/** @brief Pin SDA (GPIO) - Raspberry Pi Pico W */
#define EEPROM_SDA_PIN 16

/** @brief Pin SCL (GPIO) - Raspberry Pi Pico W */
#define EEPROM_SCL_PIN 17

/** @brief Velocidad del bus I2C en Hz (Standard Mode) */
#define EEPROM_BAUD_RATE 100000

/** @brief Dirección I2C del chip 24LC128 (A2=A1=A0=GND) */
#define EEPROM_DEV_ADDR 0x50

/* ========== CARACTERÍSTICAS DE LA MEMORIA ========== */

/** @brief Capacidad total de la EEPROM en kilobytes */
#define EEPROM_CAPACITY_KB 16

/** @brief Tamaño total de la EEPROM en bytes (16384 bytes) */
#define EEPROM_TOTAL_BYTES (EEPROM_CAPACITY_KB * 1024)

/** @brief Tamaño de página para escritura en bytes */
#define EEPROM_PAGE_SIZE 64

/** @brief Número total de páginas disponibles (256) */
#define EEPROM_NUM_PAGES (EEPROM_TOTAL_BYTES / EEPROM_PAGE_SIZE)

/* ========== ESTRUCTURA DE DATOS AMBIENTALES ========== */

/**
 * @brief Registro de muestra ambiental
 * 
 * @details Estructura para almacenar lecturas de sensores LM35 y LDR
 *          durante la operación del carro autónomo.
 */
typedef struct __attribute__((packed)) {
    float temperature;    /**< Temperatura en °C desde LM35 */
    float light;          /**< Intensidad de luz en % desde LDR */
    uint16_t sample_id;   /**< Identificador secuencial de la muestra */
} EnvironmentalRecord;

/* ========== CONFIGURACIÓN DEL BUFFER CIRCULAR ========== */

/** @brief Tamaño de la estructura de metadatos en bytes (write_index + count + magic) */
#define EEPROM_METADATA_SIZE 8

/** @brief Dirección de inicio donde se almacenan los metadatos (últimos 8 bytes) */
#define EEPROM_METADATA_ADDR (EEPROM_TOTAL_BYTES - EEPROM_METADATA_SIZE)

/** @brief Tamaño de cada registro ambiental en bytes */
#define EEPROM_RECORD_SIZE sizeof(EnvironmentalRecord)

/** @brief Capacidad máxima de registros que caben en el área útil */
#define EEPROM_MAX_RECORDS (EEPROM_METADATA_ADDR / EEPROM_RECORD_SIZE)

/** @brief Número mágico para validar integridad de metadatos (0xCAFEBABE) */
#define EEPROM_MAGIC_NUMBER 0xCAFEBABE

/* ========== CÓDIGOS DE ERROR ========== */

/**
 * @brief Códigos de retorno para operaciones de EEPROM
 */
typedef enum {
    EEPROM_SUCCESS = 0,          /**< Operación completada exitosamente */
    EEPROM_ERR_I2C_FAIL,         /**< Error de comunicación I2C */
    EEPROM_ERR_TIMEOUT,          /**< Timeout esperando respuesta del dispositivo */
    EEPROM_ERR_INVALID_ADDRESS,  /**< Dirección fuera de rango válido */
    EEPROM_ERR_NULL_POINTER,     /**< Puntero nulo recibido como argumento */
    EEPROM_ERR_INVALID_SIZE,     /**< Tamaño de datos inválido o fuera de límites */
    EEPROM_ERR_BUFFER_EMPTY,     /**< No hay registros almacenados en el buffer */
    EEPROM_ERR_METADATA_CORRUPT  /**< Metadatos corruptos o inválidos */
} eeprom_error_t;

/* ========== FUNCIONES PÚBLICAS - INICIALIZACIÓN ========== */

/**
 * @brief Inicializa el driver de EEPROM y carga metadatos persistentes
 * 
 * Configura el bus I2C en GP16/17, verifica la presencia de la EEPROM y carga
 * los metadatos (write_index, count) desde la memoria. Si los metadatos son
 * inválidos o no existen, inicializa el sistema desde cero.
 * 
 * @retval EEPROM_SUCCESS Inicialización exitosa
 * @retval EEPROM_ERR_I2C_FAIL La EEPROM no responde en el bus I2C
 * 
 * @note Debe llamarse una sola vez al inicio del programa antes de cualquier
 *       otra operación con la EEPROM
 */
eeprom_error_t eeprom_init(void);

/* ========== FUNCIONES PÚBLICAS - ALMACENAMIENTO AMBIENTAL ========== */

/**
 * @brief Almacena una muestra ambiental en el buffer circular
 * 
 * Escribe el registro en la posición actual indicada por write_index,
 * incrementa el índice de forma circular, y actualiza el contador.
 * Cuando la memoria se llena, sobrescribe automáticamente el registro más antiguo.
 * 
 * @param[in] record Puntero a la estructura EnvironmentalRecord a almacenar
 * 
 * @retval EEPROM_SUCCESS Registro guardado correctamente
 * @retval EEPROM_ERR_NULL_POINTER El parámetro record es NULL
 * @retval EEPROM_ERR_I2C_FAIL Fallo en la comunicación I2C durante escritura
 * 
 * @warning Esta función NO persiste los metadatos automáticamente.
 *          Debe llamarse eeprom_save_metadata() periódicamente
 */
eeprom_error_t eeprom_store_sample(const EnvironmentalRecord *record);

/**
 * @brief Lee todas las muestras almacenadas en orden cronológico (comando DATA)
 * 
 * Recupera todos los registros del buffer circular manejando correctamente
 * el wrap-around. Los registros se retornan ordenados del más antiguo al más reciente.
 * 
 * @param[out] buffer Array donde almacenar los registros (debe tener espacio para count elementos)
 * @param[out] count Puntero donde se almacenará el número de registros leídos
 * 
 * @retval EEPROM_SUCCESS Lectura completada exitosamente
 * @retval EEPROM_ERR_NULL_POINTER Algún parámetro es NULL
 * @retval EEPROM_ERR_BUFFER_EMPTY No hay registros almacenados
 * @retval EEPROM_ERR_I2C_FAIL Fallo en la comunicación I2C durante lectura
 * 
 * @note Usado para implementar el comando DATA de la consola
 */
eeprom_error_t eeprom_read_all_samples(EnvironmentalRecord *buffer, uint16_t *count);

/**
 * @brief Limpia lógicamente la EEPROM (comando CLEAR)
 * 
 * Resetea write_index y count a 0, marca el buffer como vacío, y persiste
 * los metadatos limpios en la EEPROM. NO borra físicamente los datos de la
 * memoria, solo los marca como inválidos.
 * 
 * @retval EEPROM_SUCCESS Limpieza completada correctamente
 * @retval EEPROM_ERR_I2C_FAIL Fallo al persistir los metadatos
 * 
 * @note Usado para implementar el comando CLEAR de la consola
 */
eeprom_error_t eeprom_clear(void);

/* ========== FUNCIONES PÚBLICAS - METADATOS ========== */

/**
 * @brief Persiste los metadatos actuales en la EEPROM
 * 
 * Escribe write_index, count y magic_number en los últimos 8 bytes de la
 * memoria para poder recuperarlos tras un reinicio del sistema.
 * 
 * @retval EEPROM_SUCCESS Metadatos guardados correctamente
 * @retval EEPROM_ERR_I2C_FAIL Fallo en la comunicación I2C durante escritura
 * 
 * @note Debe llamarse antes de detener el carro para preservar datos
 */
eeprom_error_t eeprom_save_metadata(void);

/**
 * @brief Carga los metadatos persistidos desde la EEPROM
 * 
 * Lee write_index, count y valida el magic_number desde la memoria.
 * Si los metadatos son inválidos o el magic_number no coincide,
 * inicializa el sistema desde cero.
 * 
 * @retval EEPROM_SUCCESS Metadatos cargados o inicializados correctamente
 * @retval EEPROM_ERR_I2C_FAIL Fallo en la comunicación I2C durante lectura
 * 
 * @note Esta función es llamada automáticamente por eeprom_init()
 */
eeprom_error_t eeprom_load_metadata(void);

/* ========== FUNCIONES PÚBLICAS - CONSULTAS (GETTERS) ========== */

/**
 * @brief Obtiene el número actual de registros almacenados
 * 
 * @return Número de registros válidos en el buffer (0 a EEPROM_MAX_RECORDS)
 */
uint16_t eeprom_get_record_count(void);

/**
 * @brief Obtiene la capacidad máxima de registros del buffer
 * 
 * @return Capacidad total del buffer circular en número de registros
 */
uint16_t eeprom_get_capacity(void);

/**
 * @brief Obtiene el índice de escritura actual
 * 
 * @return Índice donde se escribirá el próximo registro (0 a EEPROM_MAX_RECORDS-1)
 */
uint16_t eeprom_get_write_index(void);

/**
 * @brief Indica si el buffer circular está completamente lleno
 * 
 * @retval true El buffer está lleno (count == EEPROM_MAX_RECORDS)
 * @retval false El buffer tiene espacio disponible
 */
bool eeprom_is_full(void);

/* ========== FUNCIONES DE BAJO NIVEL ========== */

/**
 * @brief Escribe un bloque arbitrario de bytes en la EEPROM
 * 
 * Gestiona automáticamente los límites de página y los tiempos de espera
 * requeridos por el dispositivo (write cycle time).
 * 
 * @param[in] address Dirección inicial de escritura (0 a EEPROM_METADATA_ADDR-1)
 * @param[in] data Puntero a los datos a escribir
 * @param[in] length Número de bytes a escribir
 * 
 * @retval EEPROM_SUCCESS Escritura completada correctamente
 * @retval EEPROM_ERR_INVALID_ADDRESS Dirección fuera de rango
 * @retval EEPROM_ERR_NULL_POINTER El parámetro data es NULL
 * @retval EEPROM_ERR_I2C_FAIL Fallo en la comunicación I2C
 * 
 * @warning Puede sobrescribir metadatos si se usa incorrectamente
 */
eeprom_error_t eeprom_write_bytes(uint16_t address, const uint8_t *data, size_t length);

/**
 * @brief Lee un bloque arbitrario de bytes desde la EEPROM
 * 
 * @param[in] address Dirección inicial de lectura
 * @param[out] data Puntero al buffer destino
 * @param[in] length Número de bytes a leer
 * 
 * @retval EEPROM_SUCCESS Lectura completada correctamente
 * @retval EEPROM_ERR_INVALID_ADDRESS Dirección fuera de rango
 * @retval EEPROM_ERR_NULL_POINTER El parámetro data es NULL
 * @retval EEPROM_ERR_I2C_FAIL Fallo en la comunicación I2C
 */
eeprom_error_t eeprom_read_bytes(uint16_t address, uint8_t *data, size_t length);

/**
 * @brief Lee un registro ambiental específico por índice
 * 
 * @param[in] record_index Índice del registro a leer (0 a EEPROM_MAX_RECORDS-1)
 * @param[out] record Puntero donde almacenar el registro leído
 * 
 * @retval EEPROM_SUCCESS Registro leído correctamente
 * @retval EEPROM_ERR_INVALID_ADDRESS Índice fuera de rango
 * @retval EEPROM_ERR_NULL_POINTER El parámetro record es NULL
 * @retval EEPROM_ERR_I2C_FAIL Fallo en la comunicación I2C
 */
eeprom_error_t eeprom_read_record(uint16_t record_index, EnvironmentalRecord *record);

#endif /* EEPROM_H */
