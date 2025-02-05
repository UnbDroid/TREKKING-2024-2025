#ifndef __PIN_CONFIG_H__
#define __PIN_CONFIG_H__
#endif
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_mac.h"

// DO 6 AO 11 NÃO PODE USAR
// 0 1 e 3

#define L_EN_LEFT_FRONT 2  // placeholders
#define L_PWM_LEFT_FRONT 4 // placeholders
#define R_PWM_LEFT_FRONT 5 // placeholders
#define OUTPUT_LEFT_FRONT                                                      \
  ((1ULL << L_EN_LEFT_FRONT) | (1ULL << L_PWM_LEFT_FRONT) |                    \
   (1ULL << R_PWM_LEFT_FRONT))
#define ENCA_LEFT_FRONT 12 // placeholders
#define ENCB_LEFT_FRONT 13 // placeholders

#define L_EN_LEFT_BACK 22  // placeholders FUNCIONANDO
#define L_PWM_LEFT_BACK 19 // placeholders
#define R_PWM_LEFT_BACK 21 // placeholders
#define OUTPUT_LEFT_BACK                                                       \
  ((1ULL << L_EN_LEFT_BACK) | (1ULL << R_PWM_LEFT_BACK) |                      \
   (1ULL << L_PWM_LEFT_BACK))
#define ENCA_LEFT_BACK 16 // placeholders
#define ENCB_LEFT_BACK 17 // placeholders

#define L_EN_RIGHT_FRONT 26  // placeholders
#define L_PWM_RIGHT_FRONT 14 // placeholders
#define R_PWM_RIGHT_FRONT 27 // placeholders
#define OUTPUT_RIGHT_FRONT                                                     \
  ((1ULL << L_EN_RIGHT_FRONT) | (1ULL << L_PWM_RIGHT_FRONT) |                  \
   (1ULL << R_PWM_RIGHT_FRONT))
#define ENCA_RIGHT_FRONT 32 // placeholders
#define ENCB_RIGHT_FRONT 33 // placeholders

#define L_EN_RIGHT_BACK 35
#define L_PWM_RIGHT_BACK 25
#define R_PWM_RIGHT_BACK 26
#define OUTPUT_RIGHT_BACK                                                      \
  ((1ULL << L_EN_RIGHT_BACK) | (1ULL << L_PWM_RIGHT_BACK) |                    \
   (1ULL << R_PWM_RIGHT_BACK))
#define ENCA_RIGHT_BACK 10 // placeholders
#define ENCB_RIGHT_BACK 34 // placeholders

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
#define LEDC_CHANNEL_LEFT_FRONT LEDC_CHANNEL_0
#define LEDC_CHANNEL_LEFT_BACK LEDC_CHANNEL_1
#define LEDC_CHANNEL_RIGHT_FRONT LEDC_CHANNEL_2
#define LEDC_CHANNEL_RIGHT_BACK LEDC_CHANNEL_3
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY (128)        // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY (10000) // Frequency in Hertz. Set frequency at 4 kH

void configure_pwm(int gpio_num, int timer, int channel);

// --------------------------------------------------------------------------------
// } Configuração LEDC
