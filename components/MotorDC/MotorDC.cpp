#include "MotorDC.h"
#include "PinConfig.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "hal/ledc_types.h"
#include <cmath>
#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <iostream>
#include <stdio.h>

MotorDC::MotorDC(const int ENCA, const int PWM, const int L_IN, const int R_IN,
                 ledc_channel_t LEDC_CHANNEL) {
  this->ENCA = ENCA;
  this->PWM = PWM;
  this->L_IN = L_IN;
  this->R_IN = R_IN;
  this->LEDC_CHANNEL = LEDC_CHANNEL;
}

void MotorDC::stop_motor() {
  ledc_set_duty(LEDC_MODE, this->LEDC_CHANNEL, (uint32_t)(0));
  ledc_update_duty(LEDC_MODE, this->LEDC_CHANNEL);
  gpio_set_level((gpio_num_t)this->L_IN, 0);
  gpio_set_level((gpio_num_t)this->R_IN, 0);
}

void MotorDC::configure_motor(int tpt, float p, float i, float d) {
  this->ticks_per_turn = tpt;
  this->kp = p;
  this->ki = i;
  this->kd = d;
}

double MotorDC::return_posi() { return this->posi; }

void MotorDC::set_direction_pwm(int direcao, double pwmVal)

{
  if (direcao == 1) {
    ledc_set_duty(LEDC_MODE, this->LEDC_CHANNEL, (uint32_t)(pwmVal));
    ledc_update_duty(LEDC_MODE, this->LEDC_CHANNEL);
    gpio_set_level((gpio_num_t)this->L_IN, 1);
    gpio_set_level((gpio_num_t)this->R_IN, 0);
  } else {
    ledc_set_duty(LEDC_MODE, this->LEDC_CHANNEL, (uint32_t)(pwmVal));
    ledc_update_duty(LEDC_MODE, this->LEDC_CHANNEL);
    gpio_set_level((gpio_num_t)this->L_IN, 0);
    gpio_set_level((gpio_num_t)this->R_IN, 1);
  }
}

void MotorDC::read_encoder(void *arg) {

  // if (this->desired_speed_rpm > 0) {
  //   this->posi++;
  // } else if (this->desired_speed_rpm < 0) {
  //   this->posi--;
  // }
  this->posi++;
}

void MotorDC::fetch_rpm() {
  this->current_time = esp_timer_get_time();
  long time = this->current_time;
  this->dt = (time - this->last_time);
  // this->dt = (this->dt / 1000000.0);
  int32_t delta_posi = this->posi - this->last_posi;

  this->current_speed_rpm =
      (delta_posi / this->ticks_per_turn) * 60000000 / this->dt;

  this->last_posi = this->posi;
  this->last_time = this->current_time;

  if (this->current_speed_rpm > 0) {
    incrementing = true;
  } else if (this->current_speed_rpm < 0) {
    incrementing = false;
  } else {
    if (this->desired_speed_rpm > 0) {
      incrementing = true;
    } else if (this->desired_speed_rpm < 0) {
      incrementing = false;
    }
  }
}

void MotorDC::reset_encoder() {
  this->posi = 0;
  this->last_posi = 0;
}

double MotorDC::return_speed() {
  double velocity = this->current_speed_rpm;
  return velocity;
}

float MotorDC::return_kp() { return this->kp; }
double MotorDC::getAngularPosition() {
  double angular_position = (this->return_posi() * 2 * 3.1415 / ticks_per_turn);
  return angular_position;
}
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

void MotorDC::move_pid(int desired_speed_rpm) {

  float posi = this->posi;
  float last_posi = this->last_posi;

  this->current_time = esp_timer_get_time();
  this->dt = ((float)(this->current_time - this->last_time)) / 1.0e6;

  float velocity = (posi - last_posi) / this->dt;
  float vel = velocity / this->ticks_per_turn * 60;

  this->v1Filt = 0.854 * this->v1Filt + 0.0728 * vel + 0.0728 * this->v1Prev;
  this->v1Prev = vel;

  this->current_speed_rpm = this->v1Filt;
  this->last_posi = posi;
  this->last_time = this->current_time;

  this->desired_speed_rpm = fabs(desired_speed_rpm);
  int dir = 1;

  if (desired_speed_rpm < 0) {
    dir = -1;
  }

  this->error = this->desired_speed_rpm - this->current_speed_rpm;

  double p = this->kp * this->error;
  this->accumulated_error += this->error;
  double i = this->ki * this->accumulated_error;
  double d = this->kd * (this->error - this->last_error) / this->dt;
  this->last_error = this->error;

  double u = p + i + d;

  double initiaL_IN = ((double)this->desired_speed_rpm / 300) * 255;

  // pwm = pwm * 255 / 625;

  this->pwm = initiaL_IN + u;
  if (this->pwm < 0) {
    this->pwm = 0;
  } else if (this->pwm > 255) {
    this->pwm = 255;
  }

  if (this->desired_speed_rpm == 0) {
    this->stop_motor();
  } else {
    this->set_direction_pwm(dir, this->pwm);
  }

}
