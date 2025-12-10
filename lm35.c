#include "lm35.h"
#include "hardware/adc.h"

#define ADC_REF_VOLTAGE 3.3f      // Referencia típica del ADC del Pico [web:35]
#define ADC_MAX_VALUE   4095.0f   // 12 bits (0..4095) [web:35]

void lm35_init(LM35_Sensor *sensor, uint adc_channel, uint gpio_pin) {
    sensor->adc_channel = adc_channel;
    sensor->gpio_pin = gpio_pin;

    // Configurar el pin como entrada ADC
    adc_gpio_init(gpio_pin);
}

float lm35_read_celsius(const LM35_Sensor *sensor) {
    // Seleccionar canal ADC
    adc_select_input(sensor->adc_channel);

    // Leer valor bruto (0..4095)
    uint16_t raw = adc_read();

    // Convertir a voltaje
    float voltage = (raw * ADC_REF_VOLTAGE) / ADC_MAX_VALUE;

    // LM35: 10 mV por °C -> T(°C) = V / 0.01 = V * 100 [web:41][web:44]
    float temperature_c = voltage * 100.0f;

    return temperature_c;
}
