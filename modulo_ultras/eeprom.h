/**
 * @file eeprom.h
 * @brief Driver para EEPROM 24LC128 con buffer circular para muestras ambientales
 * @details Sistema de almacenamiento persistente con:
 *          - Buffer circular automático (wrap-around)
 *          - Metadatos persistentes (write_index, count)
 *          - Comandos DATA (leer N muestras) y CLEAR (limpiar buffer)
 * 
 * © 2025
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

/** @brief Pin SDA (GPIO) */
#define EEPROM_SDA_PIN 12

/** @brief Pin SCL (GPIO) */
#define EEPROM_SCL_PIN 13

/** @brief Velocidad del bus I2C */
#define EEPROM_BAUD_RATE 100000

/** @brief Dirección I2C del chip 24LC128 */
#define EEPROM_DEV_ADDR 0x50

/* ========== CARACTERÍSTICAS DE LA MEMORIA ========== */

#define EEPROM_CAPACITY_KB 16
#define EEPROM_TOTAL_BYTES (EEPROM_CAPACITY_KB * 1024)
#define EEPROM_PAGE_SIZE 64
#define EEPROM_NUM_PAGES (EEPROM_TOTAL_BYTES / EEPROM_PAGE_SIZE)

/* ========== ESTRUCTURA DE DATOS AMBIENTALES ========== */

typedef struct __attribute__((packed)) {
    float temperature;     /**< Temperatura °C */
    float light;           /**< Luz % */
    uint16_t sample_id;    /**< ID secuencial del registro */
} EnvironmentalRecord;

/* ========== CONFIGURACIÓN DEL BUFFER CIRCULAR ========== */

#define EEPROM_METADATA_SIZE 8
#define EEPROM_METADATA_ADDR (EEPROM_TOTAL_BYTES - EEPROM_METADATA_SIZE)
#define EEPROM_RECORD_SIZE sizeof(EnvironmentalRecord)
#define EEPROM_MAX_RECORDS (EEPROM_METADATA_ADDR / EEPROM_RECORD_SIZE)
#define EEPROM_MAGIC_NUMBER 0xCAFEBABE

/* ========== CÓDIGOS DE ERROR ========== */

typedef enum {
    EEPROM_SUCCESS = 0,
    EEPROM_ERR_I2C_FAIL,
    EEPROM_ERR_TIMEOUT,
    EEPROM_ERR_INVALID_ADDRESS,
    EEPROM_ERR_NULL_POINTER,
    EEPROM_ERR_INVALID_SIZE,
    EEPROM_ERR_BUFFER_EMPTY,
    EEPROM_ERR_METADATA_CORRUPT
} eeprom_error_t;

/* ========== FUNCIONES PÚBLICAS - INICIALIZACIÓN ========== */

eeprom_error_t eeprom_init(void);

/* ========== ALMACENAMIENTO AMBIENTAL ========== */

eeprom_error_t eeprom_store_sample(const EnvironmentalRecord *record);

eeprom_error_t eeprom_read_all_samples(EnvironmentalRecord *buffer, uint16_t *count);

eeprom_error_t eeprom_clear(void);

/* ========== METADATOS ========== */

eeprom_error_t eeprom_save_metadata(void);
eeprom_error_t eeprom_load_metadata(void);

/* ========== GETTERS ========== */

uint16_t eeprom_get_record_count(void);
uint16_t eeprom_get_capacity(void);
uint16_t eeprom_get_write_index(void);
bool eeprom_is_full(void);

/* ========== BAJO NIVEL ========== */

eeprom_error_t eeprom_write_bytes(uint16_t address, const uint8_t *data, size_t length);
eeprom_error_t eeprom_read_bytes(uint16_t address, uint8_t *data, size_t length);
eeprom_error_t eeprom_read_record(uint16_t record_index, EnvironmentalRecord *record);

/* ========== FUNCIÓN AGREGADA (FALTANTE) ========== */
/**
 * @brief Imprime en consola todos los registros ordenados cronológicamente (comando DATA)
 */
void eeprom_print_records(void);

#endif /* EEPROM_H */
