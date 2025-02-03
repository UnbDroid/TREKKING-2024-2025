#include <stdio.h>
#include "MotorDC.h"
#include <iostream>
#include "driver/gpio.h"
#include "PinConfig.h"
#include "driver/ledc.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

QueueHandle_t MotorDC::gpio_evt_queue = NULL;

MotorDC::MotorDC(const int ENCA, const int ENCB, const int L_EN, const int L_PWM, const int R_PWM)
{
    this->ENCA = ENCA;
    this->ENCB = ENCB;
    this->L_EN = L_EN;
    this->L_PWM = L_PWM;
    this->R_PWM = R_PWM;
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
}

void MotorDC::stop_motor()
{
    gpio_set_level((gpio_num_t) this->L_EN, 0);
    gpio_set_level((gpio_num_t) this->L_PWM, 0);
    gpio_set_level((gpio_num_t) this->R_PWM, 0);
}

void MotorDC::configure_motor(int ticks_per_turn, float kp, float ki, float kd)
{
    this->ticks_per_turn = ticks_per_turn;
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void MotorDC::set_motor(int direcao, int pwmVal)
{
    if (direcao == 1)
    {
        gpio_set_level((gpio_num_t) this->L_EN, 1);
        gpio_set_level((gpio_num_t) this->L_PWM, pwmVal);
        gpio_set_level((gpio_num_t) this->R_PWM, 0);
    }
    else
    {
        gpio_set_level((gpio_num_t) this->L_EN, 1);
        gpio_set_level((gpio_num_t) this->L_PWM, 0);
        gpio_set_level((gpio_num_t) this->R_PWM, pwmVal);
    }
}

void MotorDC::read_encoder(void *arg)
{
    

    if (gpio_get_level((gpio_num_t) this->ENCB) == 1)
    {
        this->posi++;
    }
    else
    {
        this->posi--;
    }

    xQueueSendFromISR(gpio_evt_queue, &this->posi, NULL);

    if (this->ticks_per_turn != 0) {
        this->turns = this->posi / this->ticks_per_turn;
    } else {
        this->turns = 0;
    }

}

void MotorDC::reset_encoder()
{
    this->posi = 0;
}

void MotorDC::go_forward(int velocidade_rpm)
{
    this->reference_rpm = velocidade_rpm;
    this->rps = this->reference_rpm / 60;
    this->turns = this->posi / this->ticks_per_turn;
    float err = this->rps - this->turns;
    float p = err * this->kp;
    float i = this->integral_err * this->ki;
    float d = (err - this->prev_err) * this->kd;
    this->integral_err += err;
    this->prev_err = err;
    int pwm = p + i + d;
    if (pwm > 255)
    {
        pwm = 255;
    }
    else if (pwm < 0)
    {
        pwm = 0;
    }
    this->set_motor(1, pwm);
}
