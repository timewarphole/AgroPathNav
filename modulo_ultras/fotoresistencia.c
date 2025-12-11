/**
 * @file fotoresistencia.c
 * @brief Driver para sensor de luz tipo LDR (Light Dependent Resistor).
 * @details Este módulo permite leer la intensidad lumínica utilizando el ADC
 * del microcontrolador. Convierte la lectura analógica en un valor crudo (raw)
 * o en un porcentaje relativo de iluminación (0-100%).
 */

#include "fotoresistencia.h"
#include "hardware/adc.h"

/* ============================================================
 * CONSTANTES DE CONVERSIÓN
 * ============================================================ */

/** @brief Voltaje de referencia lógico de la Raspberry Pi Pico (3.3V). */
#define ADC_REF_VOLTAGE 3.3f

/** @brief Valor máximo representable por el ADC de 12 bits (2^12 - 1). */
#define ADC_MAX_VALUE   4095.0f

/* ============================================================
 * FUNCIONES DE CONTROL
 * ============================================================ */

/**
 * @brief Inicializa el sensor LDR configurando el pin GPIO para uso analógico.
 * @param sensor Puntero a la estructura de configuración del sensor LDR.
 * @param adc_channel Canal del ADC a utilizar (0, 1 o 2).
 * @param gpio_pin Pin físico GPIO conectado al divisor de voltaje del LDR.
 */
void ldr_init(LDR_Sensor *sensor, uint adc_channel, uint gpio_pin) {
    sensor->adc_channel = adc_channel;
    sensor->gpio_pin = gpio_pin;

    // Configurar el pin como entrada ADC (High-Impedance)
    adc_gpio_init(gpio_pin);
}

/**
 * @brief Lee el valor digital directo del conversor analógico-digital (ADC).
 * @details Realiza una lectura simple de 12 bits. Útil si se necesita procesar
 * el dato con una fórmula personalizada fuera de este driver.
 * @param sensor Puntero a la estructura del sensor.
 * @return uint16_t Valor entero entre 0 (0V) y 4095 (3.3V).
 */
uint16_t ldr_read_raw(const LDR_Sensor *sensor) {
    // Seleccionar el canal ADC correspondiente (mux)
    adc_select_input(sensor->adc_channel);
    
    // Leer valor bruto (0..4095)
    return adc_read();
}

/**
 * @brief Obtiene la intensidad de luz como un porcentaje (0% a 100%).
 * @details Convierte linealmente el valor del ADC a porcentaje.
 * * @note Esta función asume una configuración de hardware donde:
 * - Más Luz = Menor Resistencia LDR = Mayor Voltaje en el pin.
 * - 0% = Oscuridad total (0V).
 * - 100% = Luz máxima saturada (3.3V).
 * Si tu circuito es inverso (pull-up al LDR), el porcentaje será inverso.
 * * @param sensor Puntero a la estructura del sensor.
 * @return float Porcentaje de luz entre 0.0f y 100.0f.
 */
float ldr_read_percent(const LDR_Sensor *sensor) {
    uint16_t raw = ldr_read_raw(sensor);

    // Convertir a porcentaje 0..100 %
    // Fórmula: (Lectura / 4095) * 100
    float percent = (raw * 100.0f) / ADC_MAX_VALUE;

    // Clamping (asegurar límites por seguridad numérica)
    if (percent < 0.0f)   percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;

    return percent;
}