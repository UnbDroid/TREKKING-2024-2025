#ifndef __PIN_CONFIG_H__
#define __PIN_CONFIG_H__
#endif
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_mac.h"

#define L_EN_ESQUERDO_FRENTE 1  //placeholders
#define L_PWM_ESQUERDO_FRENTE 2 //placeholders
#define R_PWM_ESQURDO_FRENTE 3 //placeholders
#define OUTPUT_ESQUERDO_FRENTE ((1ULL<<L_EN_ESQUERDO_FRENTE) | (1ULL<<L_PWM_ESQUERDO_FRENTE) | (1ULL<<R_PWM_ESQURDO_FRENTE))
#define ENCA_ESQUERDO_FRENTE 4 //placeholders
#define ENCB_ESQUERDO_FRENTE 5 //placeholders

#define L_EN_ESQUERDO_TRAS GPIO_NUM_22 //placeholders
#define L_PWM_ESQUERDO_TRAS 7 //placeholders
#define R_PWM_ESQURDO_TRAS 8 //placeholders
#define OUTPUT_ESQUERDO_TRAS ((1ULL<<L_EN_ESQUERDO_TRAS) | (1ULL<<L_PWM_ESQUERDO_TRAS) | (1ULL<<R_PWM_ESQURDO_TRAS))
#define ENCA_ESQUERDO_TRAS 9 //placeholders
#define ENCB_ESQUERDO_TRAS 10 //placeholders

#define L_EN_DIREITO_FRENTE 11 //placeholders
#define L_PWM_DIREITO_FRENTE 12 //placeholders
#define R_PWM_DIREITO_FRENTE 13 //placeholders
#define OUTPUT_DIREITO_FRENTE ((1ULL<<L_EN_DIREITO_FRENTE) | (1ULL<<L_PWM_DIREITO_FRENTE) | (1ULL<<R_PWM_DIREITO_FRENTE))
#define ENCA_DIREITO_FRENTE 14 //placeholders``
#define ENCB_DIREITO_FRENTE 15 //placeholders

#define L_EN_DIREITO_TRAS 16 //placeholders
#define L_PWM_DIREITO_TRAS 17 //placeholders
#define R_PWM_DIREITO_TRAS 18 //placeholders
#define OUTPUT_DIREITO_TRAS ((1ULL<<L_EN_DIREITO_TRAS) | (1ULL<<L_PWM_DIREITO_TRAS) | (1ULL<<R_PWM_DIREITO_TRAS))
#define ENCA_DIREITO_TRAS 19 //placeholders
#define ENCB_DIREITO_TRAS 20 //placeholders

#define ENCA_GERAL ((1ULL<<ENCA_ESQUERDO_FRENTE) | (1ULL<<ENCA_ESQUERDO_TRAS) | (1ULL<<ENCA_DIREITO_FRENTE) | (1ULL<<ENCA_DIREITO_TRAS))
#define ENCB_GERAL ((1ULL<<ENCB_ESQUERDO_FRENTE) | (1ULL<<ENCB_ESQUERDO_TRAS) | (1ULL<<ENCB_DIREITO_FRENTE) | (1ULL<<ENCB_DIREITO_TRAS))

// Configuração GPIO dos pinos { --------------------------------------------------------------------------------
void pin_configuration();

// -------------------------------------------------------------------------------- } Configuração GPIO dos pinos

// Configuração LEDC { --------------------------------------------------------------------------------

