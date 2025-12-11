/**
 * @file lm35.c
 * @brief Driver para el sensor de temperatura analógico LM35 para Raspberry Pi Pico.
 * @details Este driver maneja la lectura del ADC y la conversión matemática 
 * necesaria para transformar el voltaje leído en grados Celsius, basándose 
 * en el factor de escala lineal del LM35 (10 mV/°C).
 */

#include "lm35.h"
#include "hardware/adc.h"

/** * @brief Voltaje de referencia del ADC de la Raspberry Pi Pico.
 * @note Si se usa una referencia externa o la fuente es ruidosa, ajustar este valor
 * para mejorar la precisión. Por defecto es 3.3V.
 */
#define ADC_REF_VOLTAGE 3.3f      

/** * @brief Resolución máxima del ADC (12 bits).
 * @details El rango de valores va de 0 a 4095.
 */
#define ADC_MAX_VALUE   4095.0f   

/**
 * @brief Inicializa el hardware necesario para leer el sensor LM35.
 * @details Configura el pin GPIO especificado como entrada analógica (High-Z)
 * y asocia los parámetros a la estructura del sensor.
 * * @param sensor Puntero a la estructura de control del LM35.
 * @param adc_channel Canal del ADC a utilizar (0, 1 o 2 para pines GP26-GP28).
 * @param gpio_pin Pin físico GPIO correspondiente al canal ADC (ej. 26, 27, 28).
 */
void lm35_init(LM35_Sensor *sensor, uint adc_channel, uint gpio_pin) {
    sensor->adc_channel = adc_channel;
    sensor->gpio_pin = gpio_pin;

    // Configurar el pin como entrada ADC (deshabilita funciones digitales)
    adc_gpio_init(gpio_pin);
}

/**
 * @brief Realiza una lectura del ADC y calcula la temperatura en Celsius.
 * * @details El proceso de conversión es:
 * 1. Lectura del valor RAW (0-4095).
 * 2. Conversión a voltaje: V = (RAW * 3.3) / 4095.
 * 3. Conversión a temperatura: T = V / 0.010V (10mV por grado).
 * * @param sensor Puntero a la estructura del sensor previamente inicializado.
 * @return float Temperatura actual en grados Celsius.
 */
float lm35_read_celsius(const LM35_Sensor *sensor) {
    // Seleccionar el canal ADC correspondiente al sensor
    adc_select_input(sensor->adc_channel);

    // Leer valor bruto de 12 bits (0..4095)
    uint16_t raw = adc_read();

    // Convertir el valor digital a voltaje (Volts)
    float voltage = (raw * ADC_REF_VOLTAGE) / ADC_MAX_VALUE;

    // LM35 Factor de escala: 10 mV por °C
    // Fórmula: T(°C) = Voltaje / 0.01
    // Equivalente a: T(°C) = Voltaje * 100
    float temperature_c = voltage * 100.0f;

    return temperature_c;
}