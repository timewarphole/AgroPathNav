#include "fotoresistencia.h"
#include "hardware/adc.h"

#define ADC_REF_VOLTAGE 3.3f
#define ADC_MAX_VALUE   4095.0f

void ldr_init(LDR_Sensor *sensor, uint adc_channel, uint gpio_pin) {
    sensor->adc_channel = adc_channel;
    sensor->gpio_pin = gpio_pin;

    // Configurar el pin como entrada ADC
    adc_gpio_init(gpio_pin);
}

uint16_t ldr_read_raw(const LDR_Sensor *sensor) {
    // Seleccionar el canal ADC correspondiente
    adc_select_input(sensor->adc_channel);
    // Leer valor bruto (0..4095)
    return adc_read();
}

float ldr_read_percent(const LDR_Sensor *sensor) {
    uint16_t raw = ldr_read_raw(sensor);

    // Convertir a porcentaje 0..100 %
    float percent = (raw * 100.0f) / ADC_MAX_VALUE;

    if (percent < 0.0f)  percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;

    return percent;
}
