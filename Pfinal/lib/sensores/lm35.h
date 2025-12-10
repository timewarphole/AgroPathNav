#ifndef LM35_H
#define LM35_H

#include "pico/stdlib.h"
#include <stdint.h>

/**
 * @brief Estructura para representar un sensor LM35 conectado al ADC.
 *
 * adc_channel: canal ADC (0, 1 o 2 para GPIO 26, 27, 28).
 * gpio_pin:    pin físico GPIO correspondiente al canal ADC.
 */
typedef struct {
    uint adc_channel;   // Canal ADC (0..2)
    uint gpio_pin;      // GPIO 26/27/28
} LM35_Sensor;


#define LM35_ADC_CHANNEL  0    // ADC0
#define LM35_GPIO_PIN     26   // GP26


/**
 * @brief Inicializa el LM35 en el canal ADC indicado.
 *
 * NO llama a adc_init(); eso debe hacerse una sola vez en main.
 *
 * @param sensor      Puntero a la estructura LM35_Sensor.
 * @param adc_channel Canal ADC (0..2).
 * @param gpio_pin    GPIO asociado (26, 27 o 28).
 */
void lm35_init(LM35_Sensor *sensor, uint adc_channel, uint gpio_pin);

/**
 * @brief Lee la temperatura en grados Celsius desde el LM35.
 *
 * Usa referencia de 3.3 V y resolución de 12 bits (0..4095) del ADC del Pico,
 * y la relación del LM35 de 10 mV/°C (T = V * 100). [web:35][web:41][web:44]
 *
 * @param sensor Puntero al sensor LM35.
 * @return Temperatura en °C.
 */
float lm35_read_celsius(const LM35_Sensor *sensor);

#endif // LM35_H
