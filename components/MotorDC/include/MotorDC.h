#ifndef _MOTORDC_H
#define _MOTORDC_H

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hal/ledc_types.h"
#include "inttypes.h"
#include "iostream"
#include "stdio.h"
#define WHEEL_RADIUS_METERS 0.06272
class MotorDC {
public:
  MotorDC(const int ENCA, const int ENCB, const int L_PWM, const int R_PWM,
          ledc_channel_t LEDC_CHANNEL_L,
          ledc_channel_t LEDC_CHANNEL_R); // Construtor da classe MotorDC
  void stop_motor();
  void configure_motor(int ticks_per_turn, float kp, float ki,
                       float kd); // Função para configurar o motor
  void set_motor(int direcao, double pwmVal);
  void IRAM_ATTR read_encoder(void *arg);
  void set_encoder();
  void reset_encoder();
  void go_forward(int desired_speed_rpm);
  int32_t return_posi();
  double return_speed();
  double wheel_lenght =
      2 * 3.1415 * WHEEL_RADIUS_METERS; // TODO: medir o raio da roda real
  float return_kp();
  float return_ki();
  float return_kd();
  void tweak_pid(int variable, float diff);

private:
  int ENCA; // Cabo amarelo
  int ENCB; // Cabo branco
  int L_EN;
  int L_PWM;
  int R_PWM;
  ledc_channel_t LEDC_CHANNEL_L;
  ledc_channel_t LEDC_CHANNEL_R;
  int ticks_per_turn; // valor de encoder referente a uma volta completa da roda
  float kp;           // valor de kp para o PID
  float ki;           // valor de ki para o PID
  float kd;           // valor de kd para o PID
  double dt = 0;

  volatile double current_speed_rpm = 0;
  volatile double last_error = 0;        // erro anterior para o PID
  volatile double accumulated_error = 0; // erro acumulado para o PID
  volatile int32_t posi = 0;             // posição do motor em ticks do encoder
  int32_t last_posi = 0;                 // posição do motor em ticks do encoder
  volatile double current_time = 0;
  volatile double last_time = 0;
};

#endif
