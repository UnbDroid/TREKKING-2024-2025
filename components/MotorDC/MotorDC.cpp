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

MotorDC::MotorDC(const int ENCA, const int ENCB, const int L_EN, const int L_PWM, const int R_PWM, ledc_channel_t LEDC_CHANNEL)
{
    this->ENCA = ENCA;
    this->ENCB = ENCB;
    this->L_EN = L_EN;
    this->L_PWM = L_PWM;
    this->R_PWM = R_PWM;
    this->LEDC_CHANNEL = LEDC_CHANNEL;
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

int32_t MotorDC::return_posi(){
    return this->posi;
}

void MotorDC::set_motor(int direcao, int pwmVal)
{
    ledc_set_duty(LEDC_MODE, this->LEDC_CHANNEL, (uint32_t)(pwmVal));
    ledc_update_duty(LEDC_MODE, this->LEDC_CHANNEL);
    if (direcao == 1)
    {
        gpio_set_level((gpio_num_t) this->L_EN, 1);
        gpio_set_level((gpio_num_t) this->L_PWM, 1);
        gpio_set_level((gpio_num_t) this->R_PWM, 0);
        std::cout<<pwmVal<<std::endl;
    }
    else
    {
        gpio_set_level((gpio_num_t) this->L_EN, 1);
        gpio_set_level((gpio_num_t) this->L_PWM, 0);
        gpio_set_level((gpio_num_t) this->R_PWM, 1);
        std::cout<<pwmVal<<std::endl;
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

}

void MotorDC::reset_encoder()
{
    this->posi = 0;
}

void MotorDC::go_forward(int velocidade_rpm)
{

    if (this->ticks_per_turn != 0) {
        this->turns = this->posi / this->ticks_per_turn;
    } else {
        this->turns = 0;
    }

    prev_time = time_now;
    time_now = esp_timer_get_time();
    double delta_time = (time_now - prev_time) / 1000000;

    this->reference_rpm = velocidade_rpm;
    this->rps = this->reference_rpm / 60;
    this->turns = this->posi / this->ticks_per_turn;
    float turns_per_sec = (this->turns - this->prev_turns) / delta_time;
    float err = this->rps - turns_per_sec;
    float p = err * this->kp;
    float i = this->integral_err * this->ki;
    float d = (err - this->prev_err) * this->kd;
    this->integral_err += err;
    this->prev_err = err;
    int pwm = p + i + d;
    int dir = 1;
    if (pwm < 0)
    {
        dir = -1;
    }
    this->set_motor(dir, pwm);
}
