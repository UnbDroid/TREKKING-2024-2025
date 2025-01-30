#ifndef __PIN_CONFIG_H__
#define __PIN_CONFIG_H__
#endif
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_mac.h"

// DO 6 AO 11 NÃO PODE USAR
// 0 1 e 3

#define L_EN_ESQUERDO_FRENTE 2  //placeholders
#define L_PWM_ESQUERDO_FRENTE 4 //placeholders
#define R_PWM_ESQUERDO_FRENTE 5 //placeholders
#define OUTPUT_ESQUERDO_FRENTE ((1ULL<<L_EN_ESQUERDO_FRENTE) | (1ULL<<L_PWM_ESQUERDO_FRENTE) | (1ULL<<R_PWM_ESQUERDO_FRENTE))
#define ENCA_ESQUERDO_FRENTE 12 //placeholders
#define ENCB_ESQUERDO_FRENTE 13 //placeholders

#define L_EN_ESQUERDO_TRAS 22 //placeholders FUNCIONANDO
#define L_PWM_ESQUERDO_TRAS 14 //placeholders
#define R_PWM_ESQUERDO_TRAS 15 //placeholders
#define OUTPUT_ESQUERDO_TRAS ((1ULL<<L_EN_ESQUERDO_TRAS)| (1ULL<<R_PWM_ESQUERDO_TRAS)|(1ULL<<L_PWM_ESQUERDO_TRAS))
#define ENCA_ESQUERDO_TRAS 16 //placeholders
#define ENCB_ESQUERDO_TRAS 17 //placeholders

#define L_EN_DIREITO_FRENTE 18 //placeholders
#define L_PWM_DIREITO_FRENTE 19 //placeholders
#define R_PWM_DIREITO_FRENTE 20 //placeholders
#define OUTPUT_DIREITO_FRENTE ((1ULL<<L_EN_DIREITO_FRENTE ) | (1ULL<<L_PWM_DIREITO_FRENTE) | (1ULL<<R_PWM_DIREITO_FRENTE))
#define ENCA_DIREITO_FRENTE 21 //placeholders
#define ENCB_DIREITO_FRENTE 23 //placeholders
//pino 24 nao da para usar?
#define L_EN_DIREITO_TRAS 27
#define L_PWM_DIREITO_TRAS 25
#define R_PWM_DIREITO_TRAS 26
#define OUTPUT_DIREITO_TRAS ((1ULL<<L_EN_DIREITO_TRAS ) | (1ULL<<L_PWM_DIREITO_TRAS) | (1ULL<<R_PWM_DIREITO_TRAS))
#define ENCA_DIREITO_TRAS 27 //placeholders
#define ENCB_DIREITO_TRAS 28 //placeholders

#define ENCA_GERAL ((1ULL<<ENCA_ESQUERDO_FRENTE) | (1ULL<<ENCA_ESQUERDO_TRAS) | (1ULL<<ENCA_DIREITO_FRENTE) | (1ULL<<ENCA_DIREITO_TRAS))
#define ENCB_GERAL ((1ULL<<ENCB_ESQUERDO_FRENTE) | (1ULL<<ENCB_ESQUERDO_TRAS) | (1ULL<<ENCB_DIREITO_FRENTE) | (1ULL<<ENCB_DIREITO_TRAS))

// Configuração GPIO dos pinos { --------------------------------------------------------------------------------
void pin_configuration();

// -------------------------------------------------------------------------------- } Configuração GPIO dos pinos

// Configuração LEDC { --------------------------------------------------------------------------------

