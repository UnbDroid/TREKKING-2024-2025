#include <stdio.h>
#include <iostream>
#include "PinConfig.h"
#include "esp_task_wdt.h"
                                       //
void configure_pwm(int gpio_num, int timer,int channel){

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

void configure_pins_input_enca(unsigned long long bit_mask) 
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

void configure_pins_input_encb(unsigned long long bit_mask) 
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
    configure_pwm(L_PWM_DIREITO_FRENTE,0,0);
    configure_pwm(R_PWM_DIREITO_FRENTE,0,1);
    configure_pins_input_enca(ENCA_GERAL);
    configure_pins_input_encb(ENCB_GERAL);
    // gpio_config(&config_enca);
    // gpio_config(&config_encb);
    std::cout << "Pinos configurados" << std::endl;
}
