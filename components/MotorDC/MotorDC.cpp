#include <stdio.h>
#include "MotorDC.h"
#include "driver/gpio.h"
#include "PinConfig.h"
MotorDC::MotorDC(const int ENCA, const int ENCB, const int L_EN, const int L_PWM, const int R_PWM)
{
    this->ENCA = ENCA;
    this->ENCB = ENCB;
    this->L_EN = L_EN;
    this->L_PWM = L_PWM;
    this->R_PWM = R_PWM;
    configure_pins_output((1ULL<<L_PWM)|(1ULL<<R_PWM));
    configure_pwm(L_PWM,0,0);
    configure_pwm(R_PWM,0,1);

}

void MotorDC::parar()
{
    gpio_set_level((gpio_num_t) L_EN, 0);
    gpio_set_level((gpio_num_t) L_PWM, 0);
    gpio_set_level((gpio_num_t) R_PWM, 0);
}

void MotorDC::congirurar(int ticks_por_volta, float kp, float ki, float kd)
{
    this->encoder_volta = ticks_por_volta;
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void MotorDC::ligar_motor(int direcao, int pwmVal)
{
    if (direcao == 1)
    {
        gpio_set_level((gpio_num_t) L_EN, 1);
        gpio_set_level((gpio_num_t) L_PWM, pwmVal);
        gpio_set_level((gpio_num_t) R_PWM, 0);
    }
    else
    {
        gpio_set_level((gpio_num_t) L_EN, 1);
        gpio_set_level((gpio_num_t) L_PWM, 0);
        gpio_set_level((gpio_num_t) R_PWM, pwmVal);
    }
}

void MotorDC::ler_encoder()
{
    int enca = gpio_get_level((gpio_num_t) ENCA);
    int encb = gpio_get_level((gpio_num_t) ENCB);
    if (enca == 1 && encb == 0)
    {
        this->posi++;
    }
    else if (enca == 0 && encb == 1)
    {
        this->posi--;
    }
    else if (enca == 0 && encb == 0)
    {
        this->posi--;
    }
    else if (enca == 1 && encb == 1)
    {
        this->posi++;
    }
}

void MotorDC::resetar_encoder()
{
    this->posi = 0;
}

void MotorDC::andar_reto(int velocidade_rpm)
{
    this->rpm_referencia = velocidade_rpm;
    this->rps = this->rpm_referencia / 60;
    this->voltas = this->posi / this->encoder_volta;
    float erro = this->rps - this->voltas;
    float p = erro * this->kp;
    float i = this->eintegral * this->ki;
    float d = (erro - this->eprev) * this->kd;
    this->eintegral += erro;
    this->eprev = erro;
    int pwm = p + i + d;
    if (pwm > 255)
    {
        pwm = 255;
    }
    else if (pwm < 0)
    {
        pwm = 0;
    }
    this->ligar_motor(1, pwm);
}
