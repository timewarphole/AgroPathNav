#ifndef FOTORESISTENCIA_H
#define FOTORESISTENCIA_H

#include "pico/stdlib.h"
#include <stdint.h>

/**
 * @brief Estructura para representar una LDR conectada al ADC del RP2040.
 *
 * adc_channel: canal ADC (0, 1 o 2 para GPIO 26, 27, 28).
 * gpio_pin:    pin físico GPIO correspondiente al canal ADC.
 */
typedef struct {
    uint adc_channel;   // Canal ADC (0..2)
    uint gpio_pin;      // GPIO 26/27/28
} LDR_Sensor;


// --- Pin sugerido para Raspberry Pi Pico W ---
#define LDR_ADC_CHANNEL  1    // ADC1
#define LDR_GPIO_PIN     27   // GP27


/**
 * @brief Inicializa la LDR en el canal ADC indicado.
 *
 * NO llama a adc_init(); eso debe hacerse una sola vez en main.
 *
 * @param sensor      Puntero a la estructura LDR_Sensor.
 * @param adc_channel Canal ADC (0..2).
 * @param gpio_pin    GPIO asociado (26, 27 o 28).
 */
void ldr_init(LDR_Sensor *sensor, uint adc_channel, uint gpio_pin);

/**
 * @brief Lee el valor RAW del ADC (0..4095).
 *
 * @param sensor Puntero al sensor LDR.
 * @return Valor bruto del ADC.
 */
uint16_t ldr_read_raw(const LDR_Sensor *sensor);

/**
 * @brief Lee la intensidad de luz como porcentaje 0..100 %.
 *
 * @param sensor Puntero al sensor LDR.
 * @return Porcentaje de luz (0.0 a 100.0).
 */
float ldr_read_percent(const LDR_Sensor *sensor);

#endif // FOTORESISTENCIA_H
