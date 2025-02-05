#include "PinConfig.h"
#include "esp_task_wdt.h"
#include "hal/gpio_types.h"
#include <iostream>
#include <stdio.h>
//
// filepath:
// /home/caldo/Projetos/TREKKING-2024-2025/components/PinConfig/PinConfig.cpp
void configure_pwm(int gpio_num, int timer, int channel) {
  ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_MODE,
      .duty_resolution = LEDC_DUTY_RES,
      .timer_num = (ledc_timer_t)timer,
      .freq_hz = LEDC_FREQUENCY, // Set output frequency at 10 kHz
      .clk_cfg = LEDC_AUTO_CLK};

  ledc_channel_config_t ledc_channel = {.gpio_num = gpio_num,
                                        .speed_mode = LEDC_MODE,
                                        .channel = (ledc_channel_t)channel,
                                        .intr_type = LEDC_INTR_DISABLE,
                                        .timer_sel = LEDC_TIMER,
                                        .hpoint = 0};

  ledc_timer_config(&ledc_timer);
  ledc_channel_config(&ledc_channel);
}

void configure_pins_input_enca(unsigned long long bit_mask) {
  gpio_config_t config_output = {.pin_bit_mask = bit_mask,
                                 .mode = GPIO_MODE_INPUT,
                                 .pull_up_en = GPIO_PULLUP_ENABLE,
                                 .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                 .intr_type = GPIO_INTR_POSEDGE};
  gpio_config(&config_output);
}

void configure_pins_input_encb(unsigned long long bit_mask) {
  gpio_config_t config_output = {.pin_bit_mask = bit_mask,
                                 .mode = GPIO_MODE_INPUT,
                                 .pull_up_en = GPIO_PULLUP_ENABLE,
                                 .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                 .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&config_output);
}

void configure_pins_output(unsigned long long bit_mask) {
  gpio_config_t config_output = {.pin_bit_mask = bit_mask,
                                 .mode = GPIO_MODE_OUTPUT,
                                 .pull_up_en = GPIO_PULLUP_DISABLE,
                                 .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                 .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&config_output);
}

void pin_configuration() {
  configure_pins_output(OUTPUT_LEFT_FRONT);
  configure_pins_output(OUTPUT_LEFT_BACK);
  configure_pins_output(OUTPUT_RIGHT_FRONT);
  configure_pins_output(OUTPUT_RIGHT_BACK);
  configure_pwm(R_PWM_RIGHT_FRONT, 0, LEDC_CHANNEL_RIGHT_FRONT_R_PWM);
  configure_pwm(L_PWM_RIGHT_FRONT, 0, LEDC_CHANNEL_RIGHT_FRONT_L_PWM);
  configure_pwm(R_PWM_LEFT_BACK, 0, LEDC_CHANNEL_LEFT_BACK_R_PWM);
  configure_pwm(L_PWM_LEFT_BACK, 0, LEDC_CHANNEL_LEFT_BACK_L_PWM);

  configure_pwm(R_PWM_RIGHT_BACK, 0, LEDC_CHANNEL_RIGHT_BACK_R_PWM);
  configure_pwm(L_PWM_RIGHT_BACK, 0, LEDC_CHANNEL_RIGHT_BACK_L_PWM);
  configure_pwm(R_PWM_LEFT_FRONT, 0, LEDC_CHANNEL_LEFT_FRONT_R_PWM);
  configure_pwm(L_PWM_LEFT_FRONT, 0, LEDC_CHANNEL_LEFT_FRONT_L_PWM);
  configure_pins_input_enca(1ULL << ENCA_LEFT_FRONT);
  configure_pins_input_enca(1ULL << ENCA_LEFT_BACK);
  configure_pins_input_enca(1ULL << ENCA_RIGHT_FRONT);
  // configure_pins_input_enca(1ULL << ENCA_RIGHT_BACK);

  // configure_pins_input_encb(1ULL << ENCB_RIGHT_BACK);
  configure_pins_input_encb(1ULL << ENCB_RIGHT_FRONT);

  configure_pins_input_encb(1ULL << ENCB_LEFT_BACK);
  configure_pins_input_encb(1ULL << ENCB_LEFT_FRONT);

  gpio_config_t gpio_34_config = {
      .pin_bit_mask = (1ULL << GPIO_NUM_34), // Apenas GPIO 34
      .mode = GPIO_MODE_INPUT,               // Garantindo que seja só input
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en =
          GPIO_PULLDOWN_DISABLE, // Usando pull-down, pois GPIO 34 é input-only
      .intr_type = GPIO_INTR_DISABLE}; // Desabilita interrupção

  gpio_config(&gpio_34_config);

  gpio_config_t gpio_35_config = {
      .pin_bit_mask = (1ULL << GPIO_NUM_35), // Apenas GPIO 34
      .mode = GPIO_MODE_INPUT,               // Garantindo que seja só input
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en =
          GPIO_PULLDOWN_DISABLE, // Usando pull-down, pois GPIO 34 é input-only
      .intr_type = GPIO_INTR_DISABLE}; // Desabilita interrupção

  gpio_config(&gpio_35_config);

  // configure_pins_input_encb(ENCB_GERAL);
  std::cout << "Pinos configurados" << std::endl;
}
