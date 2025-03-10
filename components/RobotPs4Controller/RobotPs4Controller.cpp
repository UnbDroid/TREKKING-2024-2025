#include "RobotPs4Controller.h"
#include "MotorDC.h"
#include "PS4BT.h"
#include "btd_vhci.h"
#include "controllerEnums.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "include/RobotPs4Controller.h"

RobotPs4Controller::RobotPs4Controller(MotorDC *right_front_motor,
                                       MotorDC *right_back_motor,
                                       MotorDC *left_front_motor,
                                       MotorDC *left_back_motor) {
  this->right_front_motor = right_front_motor;
  this->right_back_motor = right_back_motor;
  this->left_front_motor = left_front_motor;
  this->left_back_motor = left_back_motor;
};
RobotPs4Controller::RobotPs4Controller() {}
void RobotPs4Controller::move(DIRECTION direction, int pwm_right_motors,
                              int pwm_left_motors) {
  this->right_back_motor->set_motor(direction, pwm_right_motors);
  this->right_front_motor->set_motor(direction, pwm_right_motors);
  this->left_back_motor->set_motor(direction, pwm_left_motors);
  this->left_front_motor->set_motor(direction, pwm_left_motors);
};
void RobotPs4Controller::set_controller(PS4BT *PS4) { this->PS4 = PS4; }
void RobotPs4Controller::rotate(TRIGGER_BOTTON triggerBoton, int pwm_value) {
  if (triggerBoton == TRIGGER_BOTTON::R2_TRIGGERED) {
    this->right_front_motor->set_motor(-1, pwm_value);
    this->left_front_motor->set_motor(1, pwm_value);

    this->right_back_motor->set_motor(-1, pwm_value);
    this->left_back_motor->set_motor(1, pwm_value);
  } else {
    this->right_front_motor->set_motor(1, pwm_value);
    this->left_front_motor->set_motor(-1, pwm_value);

    this->right_back_motor->set_motor(1, pwm_value);
    this->left_back_motor->set_motor(-1, pwm_value);
  }
}
int map_R2_and_L2_to_pwm(int value) {
  float proportional = float(MAX_VALUE_PWM) / MAX_VALUE_R2_L2;
  int new_value_scalled = proportional * value;
  return new_value_scalled;
}
int map_analogHat(DIRECTION direction, int value) {

  if (direction == DIRECTION::FOWARD) {
    int slliced_value = value * -1;
    slliced_value = slliced_value + START_FOWARD_ANALOG_HAT_VALUE;
    float proportional = float(MAX_VALUE_PWM) / (START_FOWARD_ANALOG_HAT_VALUE -
                                                 END_FOWARD_ANALOG_HAT_VALUE);
    int new_value_scalled = proportional * slliced_value;
    return new_value_scalled;
  }
  int slliced_value = value - 138;
  float proportional = float(MAX_VALUE_PWM) / (END_BACKWARD_ANALOG_HAT_VALUE -
                                               START_BACKWARD_ANALOG_HAT_VALUE);
  return proportional * slliced_value;
}
void RobotPs4Controller::controll_robot() {
  if (this->PS4->getAnalogHat(LeftHatY) > START_BACKWARD_ANALOG_HAT_VALUE ||
      this->PS4->getAnalogHat(LeftHatY) < START_FOWARD_ANALOG_HAT_VALUE) {

    int value = this->PS4->getAnalogHat(LeftHatY);
    DIRECTION direction = DIRECTION::BACKWARD;
    direction = value < START_FOWARD_ANALOG_HAT_VALUE ? DIRECTION::FOWARD
                                                      : DIRECTION::BACKWARD;
    value = map_analogHat(direction, value);
    ESP_LOGI(LOG_TAG, "VALOR ANALOGICO: %d", value);
    int left_velocity_motors = value;
    int right_velocity_motors = value;
    if (this->PS4->getAnalogButton(L2)) {
      int valor = this->PS4->getAnalogButton(L2);
      int scalled_value = map_R2_and_L2_to_pwm(valor);
      right_velocity_motors = right_velocity_motors - 2 * scalled_value;
    } else if (this->PS4->getAnalogButton(R2)) {
      int valor = this->PS4->getAnalogButton(R2);
      int scalled_value = map_R2_and_L2_to_pwm(valor);
      left_velocity_motors = left_velocity_motors - 2 * scalled_value;
    }
    move(direction, right_velocity_motors, left_velocity_motors);
  } else if (this->PS4->getAnalogButton(L2)) {

    int valor = this->PS4->getAnalogButton(L2);
    int scalled_value = map_R2_and_L2_to_pwm(valor);
    ESP_LOGI(LOG_TAG, " Valor L2 = %d , VALOR L2 MAPEADO = %d", valor,
             scalled_value);
    rotate(L2_TRIGGERED, scalled_value);
  } else if (this->PS4->getAnalogButton(R2)) {

    int valor = this->PS4->getAnalogButton(R2);
    int scalled_value = map_R2_and_L2_to_pwm(valor);
    ESP_LOGI(LOG_TAG, " Valor R2 = %d , VALOR R2 MAPEADO = %d", valor,
             scalled_value);
    rotate(R2_TRIGGERED, scalled_value);
  } else {
    rotate(R2_TRIGGERED, 0);
    move(FOWARD, 0, 0);
  }
}

void RobotPs4Controller::task_robot_controll(void *tasks_param) {

  while (1) {
    btd_vhci_mutex_lock();

    controll_robot();
    btd_vhci_mutex_unlock();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
