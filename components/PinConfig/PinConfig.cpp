#include <stdio.h>
#include <iostream>
#include "PinConfig.h"
#include "esp_task_wdt.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          22 // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (128) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (10000) // Frequency in Hertz. Set frequency at 4 kH
                                       //



void setup_timer_ledc(int gpio_num, int timer,int channel){

  ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = (ledc_timer_t)timer,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
  ledc_channel_config_t ledc_channel = {
        .gpio_num       = gpio_num,
        .speed_mode     = LEDC_MODE,
        .channel        = (ledc_channel_t)channel,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ledc_timer_config(&ledc_timer);
    ledc_channel_config(&ledc_channel);
}
void configure_pins_input(unsigned long long bit_mask) 
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
    setup_timer_ledc(19,0,0);
    // configure_pins_input(ENCA_GERAL);
    // configure_pins_input(ENCB_GERAL);
    // gpio_config(&config_enca);
    // gpio_config(&config_encb);
    std::cout << "Pinos configurados" << std::endl;
}
