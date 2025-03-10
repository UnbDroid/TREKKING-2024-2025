#ifndef __PIN_CONFIG_H__
#define __PIN_CONFIG_H__

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_mac.h"

// DO 6 AO 11 NÃO PODE USAR
// 0 1 e 3

// Used pins: 2, 4, 5, 10, 12, 13, 14, 16, 17, 19, 21, 22, 25, 26, 27, 32, 33,
// 34, 35

// #define L_EN_LEFT_FRONT 23  // placeholders
#define L_PWM_LEFT_FRONT 22 // placeholders
#define R_PWM_LEFT_FRONT 23 // placeholders
#define OUTPUT_LEFT_FRONT                                                      \
  ((1ULL << L_PWM_LEFT_FRONT) | (1ULL << R_PWM_LEFT_FRONT))
#define ENCA_LEFT_FRONT 25 // placeholders
#define ENCB_LEFT_FRONT 26 // placeholders

// #define L_EN_LEFT_BACK 22  // placeholders FUNCIONANDO
#define L_PWM_LEFT_BACK 21 // placeholders
#define R_PWM_LEFT_BACK 19 // placeholders
#define OUTPUT_LEFT_BACK ((1ULL << R_PWM_LEFT_BACK) | (1ULL << L_PWM_LEFT_BACK))
#define ENCA_LEFT_BACK 32 // placeholders
#define ENCB_LEFT_BACK 33 // placeholders

// #define L_EN_RIGHT_FRONT 26  // placeholders
#define L_PWM_RIGHT_FRONT 27 // placeholders
#define R_PWM_RIGHT_FRONT 14 // placeholders
#define OUTPUT_RIGHT_FRONT                                                     \
  ((1ULL << L_PWM_RIGHT_FRONT) | (1ULL << R_PWM_RIGHT_FRONT))
#define ENCA_RIGHT_FRONT 16 // placeholders 17
#define ENCB_RIGHT_FRONT 17 // placeholders 16

// #define L_EN_RIGHT_BACK 10
#define L_PWM_RIGHT_BACK 13
#define R_PWM_RIGHT_BACK 12
#define OUTPUT_RIGHT_BACK                                                      \
  ((1ULL << L_PWM_RIGHT_BACK) | (1ULL << R_PWM_RIGHT_BACK))
#define ENCA_RIGHT_BACK 5  // placeholders
#define ENCB_RIGHT_BACK 18 // placeholders

#define ENCA_GERAL                                                             \
  ((1ULL << ENCA_LEFT_FRONT) | (1ULL << ENCA_LEFT_BACK) |                      \
   (1ULL << ENCA_RIGHT_FRONT) | (1ULL << ENCA_RIGHT_BACK))
#define ENCB_GERAL                                                             \
  ((1ULL << ENCB_LEFT_FRONT) | (1ULL << ENCB_LEFT_BACK) |                      \
   (1ULL << ENCB_RIGHT_FRONT) | (1ULL << ENCB_RIGHT_BACK))

// Configuração GPIO dos pinos {
// --------------------------------------------------------------------------------

void configure_pins_output(unsigned long long bit_mask);
void configure_pins_input_enca(unsigned long long bit_mask);
void configure_pins_input_encb(unsigned long long bit_mask);

void pin_configuration();

// --------------------------------------------------------------------------------
// } Configuração GPIO dos pinos

// Configuração LEDC {
// --------------------------------------------------------------------------------

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_LEFT_FRONT_L_PWM LEDC_CHANNEL_0
#define LEDC_CHANNEL_LEFT_FRONT_R_PWM LEDC_CHANNEL_1
#define LEDC_CHANNEL_LEFT_BACK_L_PWM LEDC_CHANNEL_2
#define LEDC_CHANNEL_LEFT_BACK_R_PWM LEDC_CHANNEL_3
#define LEDC_CHANNEL_RIGHT_FRONT_L_PWM LEDC_CHANNEL_4
#define LEDC_CHANNEL_RIGHT_FRONT_R_PWM LEDC_CHANNEL_5
#define LEDC_CHANNEL_RIGHT_BACK_L_PWM LEDC_CHANNEL_6
#define LEDC_CHANNEL_RIGHT_BACK_R_PWM LEDC_CHANNEL_7
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY (128)        // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY (10000) // Frequency in Hertz. Set frequency at 4 kH

void configure_pwm(int gpio_num, int timer, int channel);

// --------------------------------------------------------------------------------
// } Configuração LEDC

#endif
