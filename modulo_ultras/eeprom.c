/**
 * @file eeprom.c
 * @brief Implementación del driver EEPROM para muestras ambientales.
 * @details Este driver gestiona una memoria EEPROM I2C (tipo 24LC128) utilizando un 
 * buffer circular para maximizar la vida útil y permitir el almacenamiento
 * continuo de datos. Incluye mecanismos de persistencia de metadatos 
 * para recuperar el estado tras un reinicio.
 * @date 2025
 */

#include "eeprom.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <string.h>
#include <stdio.h>

/* ============================================================
 * VARIABLES GLOBALES Y ESTADO INTERNO
 * ============================================================ */

/** @brief Índice donde se escribirá el próximo registro en el buffer circular. */
static uint16_t g_write_index = 0;    

/** @brief Número actual de registros válidos almacenados. */
static uint16_t g_record_count = 0;   

/** @brief Bandera que indica si el buffer circular ha dado la vuelta (está lleno). */
static bool     g_is_full = false;    

/** @brief Contador global monotónico para asignar IDs únicos a las muestras. */
static uint16_t g_sample_counter = 0; 

/**
 * @struct eeprom_metadata_t
 * @brief Estructura para almacenar el estado del sistema en la EEPROM.
 * @details Esta estructura se guarda en una dirección reservada de la memoria
 * para permitir que el sistema recuerde dónde estaba escribiendo
 * después de un corte de energía.
 */
typedef struct __attribute__((packed)) {
    uint16_t write_index;   /**< Posición actual de escritura */
    uint16_t record_count;  /**< Cantidad de registros guardados */
    uint32_t magic_number;  /**< Número mágico para validar la integridad de los datos */
} eeprom_metadata_t;


/* ============================================================
 * FUNCIONES PRIVADAS / HELPERS
 * ============================================================ */

/**
 * @brief Verifica si la EEPROM está lista para recibir comandos (ACK polling).
 * @return true Si el dispositivo responde (ACK).
 * @return false Si el dispositivo no responde (NACK).
 */
static bool eeprom_check_ready(void) {
    uint8_t dummy = 0;
    // Intenta escribir 0 bytes. Si devuelve >= 0, el dispositivo respondió ACK.
    int result = i2c_write_blocking(EEPROM_I2C_ID, EEPROM_DEV_ADDR,
                                    &dummy, 0, false);
    return (result >= 0);
}

/**
 * @brief Espera hasta que la EEPROM termine su ciclo de escritura interna.
 * @param timeout_ms Tiempo máximo de espera en milisegundos.
 * @return EEPROM_SUCCESS si la escritura terminó, EEPROM_ERR_TIMEOUT si excedió el tiempo.
 */
static eeprom_error_t eeprom_wait_write_complete(uint32_t timeout_ms) {
    sleep_ms(6); // Espera inicial mínima recomendada por datasheet (tWR ~5ms)

    uint32_t start = to_ms_since_boot(get_absolute_time());

    while (!eeprom_check_ready()) {
        if (to_ms_since_boot(get_absolute_time()) - start >= timeout_ms)
            return EEPROM_ERR_TIMEOUT;

        sleep_ms(2);
    }
    return EEPROM_SUCCESS;
}


/* ============================================================
 * FUNCIONES DE INICIALIZACIÓN
 * ============================================================ */

/**
 * @brief Inicializa el hardware I2C y carga el estado previo de la EEPROM.
 * @return EEPROM_SUCCESS si todo es correcto.
 * @return EEPROM_ERR_I2C_FAIL si no se detecta la memoria.
 */
eeprom_error_t eeprom_init(void) {
    i2c_init(EEPROM_I2C_ID, EEPROM_BAUD_RATE);

    gpio_set_function(EEPROM_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(EEPROM_SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(EEPROM_SDA_PIN);
    gpio_pull_up(EEPROM_SCL_PIN);
    sleep_ms(10);

    if (!eeprom_check_ready())
        return EEPROM_ERR_I2C_FAIL;

    // Recuperar el estado anterior (índices) si existe
    return eeprom_load_metadata();
}


/* ============================================================
 * FUNCIONES DE BAJO NIVEL (LECTURA/ESCRITURA RAW)
 * ============================================================ */

/**
 * @brief Escribe una secuencia de bytes en la EEPROM gestionando los límites de página.
 * @details Las EEPROM escriben en páginas (ej. 64 bytes). Si una escritura cruza
 * un límite de página, debe dividirse en múltiples transacciones.
 * * @param address Dirección de memoria inicial (0x0000 - 0xFFFF).
 * @param data Puntero al buffer de datos a escribir.
 * @param length Cantidad de bytes a escribir.
 * @return Código de error o éxito.
 */
eeprom_error_t eeprom_write_bytes(uint16_t address, const uint8_t *data, size_t length) {
    if (!data) return EEPROM_ERR_NULL_POINTER;
    if (!length) return EEPROM_ERR_INVALID_SIZE;
    if (address + length > EEPROM_TOTAL_BYTES) return EEPROM_ERR_INVALID_ADDRESS;

    size_t written = 0;

    while (written < length) {
        uint16_t curr_addr = address + written;
        uint8_t page_offset = curr_addr % EEPROM_PAGE_SIZE;
        uint8_t bytes_page_end = EEPROM_PAGE_SIZE - page_offset;

        // Calcular cuánto podemos escribir en la página actual
        size_t to_write = length - written;
        if (to_write > bytes_page_end)
            to_write = bytes_page_end;

        // Preparar buffer: [Addr High] [Addr Low] [Data...]
        uint8_t tx_buffer[EEPROM_PAGE_SIZE + 2];
        tx_buffer[0] = curr_addr >> 8;
        tx_buffer[1] = curr_addr & 0xFF;
        memcpy(&tx_buffer[2], &data[written], to_write);

        int result = i2c_write_blocking(EEPROM_I2C_ID, EEPROM_DEV_ADDR,
                                        tx_buffer, to_write + 2, false);

        if (result < 0) return EEPROM_ERR_I2C_FAIL;

        // Esperar a que el ciclo de escritura físico termine
        eeprom_error_t wait = eeprom_wait_write_complete(20);
        if (wait != EEPROM_SUCCESS) return wait;

        written += to_write;
    }
    return EEPROM_SUCCESS;
}

/**
 * @brief Lee una secuencia de bytes de la EEPROM.
 * @param address Dirección de memoria inicial.
 * @param data Puntero al buffer donde se guardarán los datos.
 * @param length Cantidad de bytes a leer.
 * @return Código de error o éxito.
 */
eeprom_error_t eeprom_read_bytes(uint16_t address, uint8_t *data, size_t length) {
    if (!data) return EEPROM_ERR_NULL_POINTER;
    if (!length) return EEPROM_ERR_INVALID_SIZE;
    if (address + length > EEPROM_TOTAL_BYTES) return EEPROM_ERR_INVALID_ADDRESS;

    uint8_t addr_buf[2] = { (uint8_t)(address >> 8), (uint8_t)(address & 0xFF) };

    // 1. Escritura dummy para establecer el puntero de dirección
    int result = i2c_write_blocking(EEPROM_I2C_ID, EEPROM_DEV_ADDR,
                                    addr_buf, 2, true); // true = restart cond
    if (result < 0) return EEPROM_ERR_I2C_FAIL;

    // 2. Lectura de los datos
    result = i2c_read_blocking(EEPROM_I2C_ID, EEPROM_DEV_ADDR,
                               data, length, false);

    if (result < 0 || (size_t)result != length)
        return EEPROM_ERR_I2C_FAIL;

    return EEPROM_SUCCESS;
}

/**
 * @brief Lee un registro individual basado en su índice lógico.
 * @param index Índice del registro (0 a MAX_RECORDS-1).
 * @param record Puntero a la estructura donde se cargarán los datos.
 * @return Código de error o éxito.
 */
eeprom_error_t eeprom_read_record(uint16_t index, EnvironmentalRecord *record) {
    if (!record) return EEPROM_ERR_NULL_POINTER;
    if (index >= EEPROM_MAX_RECORDS) return EEPROM_ERR_INVALID_ADDRESS;

    uint16_t address = index * EEPROM_RECORD_SIZE;
    return eeprom_read_bytes(address, (uint8_t *)record, EEPROM_RECORD_SIZE);
}


/* ============================================================
 * LOGICA DE ALMACENAMIENTO (BUFFER CIRCULAR)
 * ============================================================ */

/**
 * @brief Almacena una nueva muestra en el buffer circular y persiste el estado.
 * @details Esta función gestiona el índice de escritura, sobrescribe datos antiguos
 * si el buffer está lleno y guarda los metadatos para persistencia.
 * * @param record Puntero a los datos de la muestra (temp, luz, etc).
 * @return EEPROM_SUCCESS si se guardó la muestra y los metadatos correctamente.
 */
eeprom_error_t eeprom_store_sample(const EnvironmentalRecord *record) {
    if (!record) return EEPROM_ERR_NULL_POINTER;

    // Copiar y asignar ID único internamente
    EnvironmentalRecord copy = *record;
    copy.sample_id = g_sample_counter++;

    // Calcular dirección física
    uint16_t address = g_write_index * EEPROM_RECORD_SIZE;

    // Guardar datos
    eeprom_error_t err = eeprom_write_bytes(address, (uint8_t *)&copy, EEPROM_RECORD_SIZE);
    if (err != EEPROM_SUCCESS) return err;

    // Actualizar índices del buffer circular
    g_write_index = (g_write_index + 1) % EEPROM_MAX_RECORDS;

    if (g_record_count < EEPROM_MAX_RECORDS)
        g_record_count++;
    else
        g_is_full = true;

    // CRÍTICO: Guardar metadatos para no perder la posición al reiniciar
    err = eeprom_save_metadata();
    if (err != EEPROM_SUCCESS) return err;

    return EEPROM_SUCCESS;
}

/**
 * @brief Lee todas las muestras almacenadas en orden cronológico.
 * @details "Desenrolla" el buffer circular, empezando por el dato más antiguo.
 * * @param buffer Array donde se guardarán los registros leídos.
 * @param count Puntero donde se almacenará la cantidad de registros leídos.
 * @return Código de error o éxito.
 */
eeprom_error_t eeprom_read_all_samples(EnvironmentalRecord *buffer, uint16_t *count) {
    if (!buffer || !count) return EEPROM_ERR_NULL_POINTER;

    if (g_record_count == 0) {
        *count = 0;
        return EEPROM_ERR_BUFFER_EMPTY;
    }

    // Determinar dónde empieza el dato más antiguo
    uint16_t start = g_is_full ? g_write_index : 0;
    uint16_t n = g_record_count;

    for (uint16_t i = 0; i < n; i++) {
        uint16_t idx = (start + i) % EEPROM_MAX_RECORDS;

        eeprom_error_t err = eeprom_read_record(idx, &buffer[i]);
        if (err != EEPROM_SUCCESS) {
            *count = i; // Devolver los leídos hasta el error
            return err;
        }
    }

    *count = n;
    return EEPROM_SUCCESS;
}

/**
 * @brief Borra lógicamente la memoria reiniciando los contadores.
 * @return Resultado de guardar los metadatos vacíos.
 */
eeprom_error_t eeprom_clear(void) {
    g_write_index = 0;
    g_record_count = 0;
    g_is_full = false;
    g_sample_counter = 0;

    return eeprom_save_metadata();
}


/* ============================================================
 * GESTIÓN DE METADATOS (PERSISTENCIA)
 * ============================================================ */

/**
 * @brief Guarda el estado actual de los índices en la zona reservada de la EEPROM.
 */
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

/**
 * @brief Carga el estado de los índices desde la EEPROM.
 * @details Verifica el "Magic Number". Si es inválido, resetea la memoria.
 */
eeprom_error_t eeprom_load_metadata(void) {
    eeprom_metadata_t meta;

    eeprom_error_t err = eeprom_read_bytes(EEPROM_METADATA_ADDR,
                                           (uint8_t *)&meta,
                                           sizeof(eeprom_metadata_t));

    // Validación de integridad
    if (err != EEPROM_SUCCESS ||
        meta.magic_number != EEPROM_MAGIC_NUMBER ||
        meta.write_index >= EEPROM_MAX_RECORDS ||
        meta.record_count > EEPROM_MAX_RECORDS) {

        // Datos corruptos o primera ejecución -> Resetear
        g_write_index = 0;
        g_record_count = 0;
        g_is_full = false;
        g_sample_counter = 0;

        return eeprom_save_metadata();
    }

    // Restaurar estado
    g_write_index = meta.write_index;
    g_record_count = meta.record_count;
    g_is_full = (meta.record_count >= EEPROM_MAX_RECORDS);
    g_sample_counter = meta.record_count; // Ojo: esto asume conteo lineal sin wrap del ID

    return EEPROM_SUCCESS;
}


/* ============================================================
 * GETTERS
 * ============================================================ */

/** @brief Obtiene la cantidad de registros almacenados. */
uint16_t eeprom_get_record_count(void) { return g_record_count; }

/** @brief Obtiene la capacidad total en número de registros. */
uint16_t eeprom_get_capacity(void)     { return EEPROM_MAX_RECORDS; }

/** @brief Obtiene el índice actual de escritura. */
uint16_t eeprom_get_write_index(void)  { return g_write_index; }

/** @brief Verifica si la memoria está llena (overwrite mode). */
bool     eeprom_is_full(void)          { return g_is_full; }


/* ============================================================
 * FUNCIONES DE DEBUG / UI
 * ============================================================ */

/**
 * @brief Imprime todos los registros almacenados en el puerto serial (stdout).
 */
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
        // Corrección visual: printf no soporta float en algunas configs de embedded
        // Asegúrate de linkear con printf_float o castear si es necesario
        printf("[%u] ID=%u | Temp=%.2f°C | Luz=%u | %02u:%02u:%02u\n",
               i,
               buffer[i].sample_id,
               buffer[i].temperature,
               (unsigned int)buffer[i].light); // Casteo simple para luz si es %
    }

    printf("===== FIN DE REGISTROS (%u muestras) =====\n\n", count);
}