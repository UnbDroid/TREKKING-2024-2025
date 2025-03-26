#include "MotorDC.h"
#include "PinConfig.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "hal/ledc_types.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <iostream>
#include <stdio.h>

MotorDC::MotorDC(const int ENCA, const int ENCB, const int L_PWM,
                 const int R_PWM, ledc_channel_t LEDC_CHANNEL_L,
                 ledc_channel_t LEDC_CHANNEL_R) {
  this->ENCA = ENCA;
  this->ENCB = ENCB;
  this->L_PWM = L_PWM;
  this->R_PWM = R_PWM;
  this->LEDC_CHANNEL_L = LEDC_CHANNEL_L;
  this->LEDC_CHANNEL_R = LEDC_CHANNEL_R;
}

void MotorDC::stop_motor() {
  gpio_set_level((gpio_num_t)this->L_PWM, 0);
  gpio_set_level((gpio_num_t)this->R_PWM, 0);
}

void MotorDC::configure_motor(int tpt, float p, float i, float d) {
  this->ticks_per_turn = tpt;
  this->kp = p;
  this->ki = i;
  this->kd = d;
}

int32_t MotorDC::return_posi() { return this->posi; }

void MotorDC::set_motor(int direcao, double pwmVal)

{
  if (direcao == 1) {
    ledc_set_duty(LEDC_MODE, this->LEDC_CHANNEL_L, (uint32_t)(pwmVal));
    ledc_update_duty(LEDC_MODE, this->LEDC_CHANNEL_L);
    ledc_set_duty(LEDC_MODE, this->LEDC_CHANNEL_R, 0);
    ledc_update_duty(LEDC_MODE, this->LEDC_CHANNEL_R);
  } else {
    ledc_set_duty(LEDC_MODE, this->LEDC_CHANNEL_R, (uint32_t)(pwmVal));
    ledc_update_duty(LEDC_MODE, this->LEDC_CHANNEL_R);
    ledc_set_duty(LEDC_MODE, this->LEDC_CHANNEL_L, 0);
    ledc_update_duty(LEDC_MODE, this->LEDC_CHANNEL_L);
  }
}

void MotorDC::read_encoder(void *arg) {

  if (gpio_get_level((gpio_num_t)this->ENCB) == 1) {
    this->posi = this->posi + 1;
  } else {
    this->posi = this->posi - 1;
  }
}
void MotorDC::fetch_rpm() {
  this->current_time = esp_timer_get_time();
  long time = this->current_time;
  this->dt = (double)(time - this->last_time);
  this->dt = (this->dt / 1000000.0);
  double delta_posi = (double)this->posi - (double)this->last_posi;

  this->current_speed_rpm =
      (delta_posi / (double)this->ticks_per_turn) * 60 / this->dt;
  this->last_posi = this->posi;
  this->last_time = this->current_time;
}
void MotorDC::reset_encoder() { this->posi = 0; }

double MotorDC::return_speed() {
  double velocity = this->current_speed_rpm;
  return velocity;
}

float MotorDC::return_kp() { return this->kp; }

float MotorDC::return_ki() { return this->ki; }

float MotorDC::return_kd() { return this->kd; }

void MotorDC::tweak_pid(int variable, float diff) {
  if (variable == 0) {
    this->kp += diff;
  } else if (variable == 1) {
    this->ki += diff;
  } else {
    this->kd += diff;
  }
}

void MotorDC::go_forward(int desired_speed_rpm) {

  double error = desired_speed_rpm - this->current_speed_rpm;
  double p = this->kp * error;
  this->accumulated_error += error * this->dt;
  double i = this->ki * this->accumulated_error;
  double d = this->kd * (error - this->last_error) / dt;
  this->last_error = error;

  double pwm = p + i + d;

  double initial_pwm = ((double)desired_speed_rpm / 625) * 255;

  pwm = pwm * 255 / 625;

  double final_pwm = initial_pwm + pwm;

  int dir = 1;

  if (final_pwm < 0) {
    final_pwm = -final_pwm;
    dir = -1;
  }

  this->set_motor(dir, final_pwm);
}
