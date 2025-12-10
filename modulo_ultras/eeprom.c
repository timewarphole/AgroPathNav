/**
 * @file eeprom.c
 * @brief Implementación del driver EEPROM para muestras ambientales
 */

#include "eeprom.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <string.h>
#include <stdio.h>

/* ============================================================
 * VARIABLES GLOBALES DEL BUFFER CIRCULAR
 * ============================================================ */

static uint16_t g_write_index = 0;     /**< Índice donde se escribirá el próximo registro */
static uint16_t g_record_count = 0;    /**< Número actual de registros almacenados */
static bool     g_is_full = false;     /**< Indicador de buffer circular lleno */
static uint16_t g_sample_counter = 0;  /**< Contador global de sample_id */

/* ============================================================
 * ESTRUCTURA DE METADATOS
 * ============================================================ */

typedef struct __attribute__((packed)) {
    uint16_t write_index;
    uint16_t record_count;
    uint32_t magic_number;
} eeprom_metadata_t;

/* ============================================================
 * FUNCIONES PRIVADAS / HELPERS
 * ============================================================ */

/**
 * @brief Verifica si la EEPROM está lista para recibir comandos
 */
static bool eeprom_check_ready(void) {
    uint8_t dummy = 0;
    int result = i2c_write_blocking(EEPROM_I2C_ID, EEPROM_DEV_ADDR,
                                    &dummy, 0, false);
    return (result >= 0);
}

/**
 * @brief Espera hasta que la EEPROM termine su escritura interna
 */
static eeprom_error_t eeprom_wait_write_complete(uint32_t timeout_ms) {
    sleep_ms(6);

    uint32_t start = to_ms_since_boot(get_absolute_time());

    while (!eeprom_check_ready()) {
        if (to_ms_since_boot(get_absolute_time()) - start >= timeout_ms)
            return EEPROM_ERR_TIMEOUT;

        sleep_ms(2);
    }
    return EEPROM_SUCCESS;
}

/* ============================================================
 * INICIALIZACIÓN DEL DRIVER
 * ============================================================ */

eeprom_error_t eeprom_init(void) {
    i2c_init(EEPROM_I2C_ID, EEPROM_BAUD_RATE);

    gpio_set_function(EEPROM_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(EEPROM_SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(EEPROM_SDA_PIN);
    gpio_pull_up(EEPROM_SCL_PIN);
    sleep_ms(10);

    if (!eeprom_check_ready())
        return EEPROM_ERR_I2C_FAIL;

    return eeprom_load_metadata();
}

/* ============================================================
 * FUNCIONES DE BAJO NIVEL
 * ============================================================ */

eeprom_error_t eeprom_write_bytes(uint16_t address, const uint8_t *data, size_t length) {
    if (!data) return EEPROM_ERR_NULL_POINTER;
    if (!length) return EEPROM_ERR_INVALID_SIZE;
    if (address + length > EEPROM_TOTAL_BYTES) return EEPROM_ERR_INVALID_ADDRESS;

    size_t written = 0;

    while (written < length) {
        uint16_t curr_addr = address + written;
        uint8_t page_offset = curr_addr % EEPROM_PAGE_SIZE;
        uint8_t bytes_page_end = EEPROM_PAGE_SIZE - page_offset;

        size_t to_write = length - written;
        if (to_write > bytes_page_end)
            to_write = bytes_page_end;

        uint8_t tx_buffer[EEPROM_PAGE_SIZE + 2];
        tx_buffer[0] = curr_addr >> 8;
        tx_buffer[1] = curr_addr & 0xFF;
        memcpy(&tx_buffer[2], &data[written], to_write);

        int result = i2c_write_blocking(EEPROM_I2C_ID, EEPROM_DEV_ADDR,
                                        tx_buffer, to_write + 2, false);

        if (result < 0) return EEPROM_ERR_I2C_FAIL;

        eeprom_error_t wait = eeprom_wait_write_complete(20);
        if (wait != EEPROM_SUCCESS) return wait;

        written += to_write;
    }
    return EEPROM_SUCCESS;
}

eeprom_error_t eeprom_read_bytes(uint16_t address, uint8_t *data, size_t length) {
    if (!data) return EEPROM_ERR_NULL_POINTER;
    if (!length) return EEPROM_ERR_INVALID_SIZE;
    if (address + length > EEPROM_TOTAL_BYTES) return EEPROM_ERR_INVALID_ADDRESS;

    uint8_t addr_buf[2] = { (uint8_t)(address >> 8), (uint8_t)(address & 0xFF) };

    int result = i2c_write_blocking(EEPROM_I2C_ID, EEPROM_DEV_ADDR,
                                    addr_buf, 2, true);
    if (result < 0) return EEPROM_ERR_I2C_FAIL;

    result = i2c_read_blocking(EEPROM_I2C_ID, EEPROM_DEV_ADDR,
                               data, length, false);

    if (result < 0 || (size_t)result != length)
        return EEPROM_ERR_I2C_FAIL;

    return EEPROM_SUCCESS;
}

eeprom_error_t eeprom_read_record(uint16_t index, EnvironmentalRecord *record) {
    if (!record) return EEPROM_ERR_NULL_POINTER;
    if (index >= EEPROM_MAX_RECORDS) return EEPROM_ERR_INVALID_ADDRESS;

    uint16_t address = index * EEPROM_RECORD_SIZE;
    return eeprom_read_bytes(address, (uint8_t *)record, EEPROM_RECORD_SIZE);
}

/* ============================================================
 * ALMACENAMIENTO AMBIENTAL
 * ============================================================ */

eeprom_error_t eeprom_store_sample(const EnvironmentalRecord *record) {
    if (!record) return EEPROM_ERR_NULL_POINTER;

    EnvironmentalRecord copy = *record;
    copy.sample_id = g_sample_counter++;

    uint16_t address = g_write_index * EEPROM_RECORD_SIZE;

    eeprom_error_t err = eeprom_write_bytes(address, (uint8_t *)&copy,EEPROM_RECORD_SIZE);

    if (err != EEPROM_SUCCESS) return err;

    g_write_index = (g_write_index + 1) % EEPROM_MAX_RECORDS;

    if (g_record_count < EEPROM_MAX_RECORDS)
        g_record_count++;
    else
        g_is_full = true;
    err = eeprom_save_metadata();

    if (err != EEPROM_SUCCESS) return err;

    return EEPROM_SUCCESS;
}

eeprom_error_t eeprom_read_all_samples(EnvironmentalRecord *buffer, uint16_t *count) {
    if (!buffer || !count) return EEPROM_ERR_NULL_POINTER;

    if (g_record_count == 0) {
        *count = 0;
        return EEPROM_ERR_BUFFER_EMPTY;
    }

    uint16_t start = g_is_full ? g_write_index : 0;
    uint16_t n = g_record_count;

    for (uint16_t i = 0; i < n; i++) {
        uint16_t idx = (start + i) % EEPROM_MAX_RECORDS;

        eeprom_error_t err = eeprom_read_record(idx, &buffer[i]);
        if (err != EEPROM_SUCCESS) {
            *count = i;
            return err;
        }
    }

    *count = n;
    return EEPROM_SUCCESS;
}

eeprom_error_t eeprom_clear(void) {
    g_write_index = 0;
    g_record_count = 0;
    g_is_full = false;
    g_sample_counter = 0;

    return eeprom_save_metadata();
}

/* ============================================================
 * METADATOS
 * ============================================================ */

eeprom_error_t eeprom_save_metadata(void) {
    eeprom_metadata_t meta = {
        .write_index = g_write_index,
        .record_count = g_record_count,
        .magic_number = EEPROM_MAGIC_NUMBER
    };

    return eeprom_write_bytes(EEPROM_METADATA_ADDR,
                              (uint8_t *)&meta,
                              sizeof(eeprom_metadata_t));
}

eeprom_error_t eeprom_load_metadata(void) {
    eeprom_metadata_t meta;

    eeprom_error_t err = eeprom_read_bytes(EEPROM_METADATA_ADDR,
                                           (uint8_t *)&meta,
                                           sizeof(eeprom_metadata_t));

    if (err != EEPROM_SUCCESS ||
        meta.magic_number != EEPROM_MAGIC_NUMBER ||
        meta.write_index >= EEPROM_MAX_RECORDS ||
        meta.record_count > EEPROM_MAX_RECORDS) {

        g_write_index = 0;
        g_record_count = 0;
        g_is_full = false;
        g_sample_counter = 0;

        return eeprom_save_metadata();
    }

    g_write_index = meta.write_index;
    g_record_count = meta.record_count;
    g_is_full = (meta.record_count >= EEPROM_MAX_RECORDS);
    g_sample_counter = meta.record_count;

    return EEPROM_SUCCESS;
}

/* ============================================================
 * GETTERS
 * ============================================================ */

uint16_t eeprom_get_record_count(void) { return g_record_count; }
uint16_t eeprom_get_capacity(void)     { return EEPROM_MAX_RECORDS; }
uint16_t eeprom_get_write_index(void)  { return g_write_index; }
bool     eeprom_is_full(void)          { return g_is_full; }

/* ============================================================
 * IMPRESIÓN DE REGISTROS
 * ============================================================ */

void eeprom_print_records(void) {
    uint16_t count = 0;

    if (g_record_count == 0) {
        printf("\n[EEPROM] No hay registros almacenados.\n");
        return;
    }

    EnvironmentalRecord buffer[EEPROM_MAX_RECORDS];

    eeprom_error_t err = eeprom_read_all_samples(buffer, &count);
    if (err != EEPROM_SUCCESS) {
        printf("\n[EEPROM] Error leyendo registros (codigo: %d)\n", err);
        return;
    }

    printf("\n===== REGISTROS AMBIENTALES GUARDADOS =====\n");

    for (uint16_t i = 0; i < count; i++) {
        printf("[%u] ID=%u | Temp=%.2f°C | Luz=%u | %02u:%02u:%02u\n",
               i,
               buffer[i].sample_id,
               buffer[i].temperature,
               buffer[i].light);
    }

    printf("===== FIN DE REGISTROS (%u muestras) =====\n\n", count);
}
