#include <stdio.h>
#include <iostream>
#include "PinConfig.h"
#include "esp_task_wdt.h"


gpio_config_t config_output = {
    .pin_bit_mask = OUTPUT_ESQUERDO_FRENTE | OUTPUT_ESQUERDO_TRAS | OUTPUT_DIREITO_FRENTE | OUTPUT_DIREITO_TRAS,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

gpio_config_t config_enca = {
    .pin_bit_mask = ENCA_GERAL,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_POSEDGE
};

gpio_config_t config_encb = {
    .pin_bit_mask = ENCB_GERAL,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

void pin_configuration() {
    gpio_config(&config_output);
    gpio_config(&config_enca);
    gpio_config(&config_encb);
    std::cout << "Pinos configurados" << std::endl;
    esp_task_wdt_reset();
}