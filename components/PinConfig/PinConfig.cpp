#include <stdio.h>
#include <iostream>
#include "PinConfig.h"
#include "esp_task_wdt.h"

void configure_enca(unsigned long long bit_mask) 
{
    gpio_config_t config_output = {
    .pin_bit_mask = bit_mask,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_POSEDGE
    };    
    gpio_config(&config_output);
}

void configure_encb(unsigned long long bit_mask) 
{
    gpio_config_t config_output = {
    .pin_bit_mask = bit_mask,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
    };    
    gpio_config(&config_output);
}

void configure_pins_output(unsigned long long bit_mask) 
{
    gpio_config_t config_output = {
    .pin_bit_mask = bit_mask,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
    };    
    gpio_config(&config_output);
}

void pin_configuration() {
    configure_pins_output(OUTPUT_ESQUERDO_FRENTE);
    configure_pins_output(OUTPUT_ESQUERDO_TRAS);
    configure_pins_output(OUTPUT_DIREITO_FRENTE);
    configure_pins_output(OUTPUT_DIREITO_TRAS);
    configure_enca(ENCA_GERAL);
    // configure_encb(ENCB_GERAL);
    // gpio_config(&config_enca);
    // gpio_config(&config_encb);
    std::cout << "Pinos configurados" << std::endl;
}