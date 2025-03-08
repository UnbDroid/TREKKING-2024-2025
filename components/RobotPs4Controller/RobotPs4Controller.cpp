#include "RobotPs4Controller.h"
#include "MotorDC.h"
#include "PS4BT.h"
#include "btd_vhci.h"
#include "controllerEnums.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "include/RobotPs4Controller.h"
#include "iostream"
#include <iostream>
#include <stdio.h>
static const char *LOG_TAG = "main";
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
void RobotPs4Controller::move_foward() {
  this->right_back_motor->set_motor(1, 110);
  this->right_front_motor->set_motor(1, 110);
  this->left_back_motor->set_motor(1, 110);
  this->left_front_motor->set_motor(1, 110);
};
void RobotPs4Controller::set_controller(PS4BT *PS4) { this->PS4 = PS4; }
void RobotPs4Controller::move_backward() {

  this->right_back_motor->set_motor(-1, 110);
  this->right_front_motor->set_motor(-1, 110);
  this->left_back_motor->set_motor(-1, 110);
  this->left_front_motor->set_motor(-1, 110);
}
void RobotPs4Controller::controll_robot() {
  if (this->PS4->getAnalogHat(LeftHatY) > 137 ||
      this->PS4->getAnalogHat(LeftHatY) < 117) {
    if (this->PS4->getAnalogButton(L2) && this->PS4->getAnalogButton(R2)) {
      ESP_LOGI(LOG_TAG, "OS DOIS DENTRO DO WHILE Valor L2 = %d Valor R2 = %d",
               this->PS4->getAnalogButton(L2), this->PS4->getAnalogButton(R2));
    } else if (this->PS4->getAnalogButton(L2)) {
      // std::cout << this->PS4->getAnalogButton(L2) << std::endl;
      ESP_LOGI(LOG_TAG, " DENTRO DO WHILE Valor L2 = %d",
               this->PS4->getAnalogButton(L2));
    } else if (this->PS4->getAnalogButton(R2)) {
      // std::cout << this->PS4->getAnalogButton(L2) << std::endl;
      ESP_LOGI(LOG_TAG, " DENTRO DO WHILE Valor R2 = %d",
               this->PS4->getAnalogButton(R2));
    } else {
      ESP_LOGI(LOG_TAG, "%d", this->PS4->getAnalogHat(LeftHatY));
    }
  } else if (this->PS4->getAnalogButton(L2) && this->PS4->getAnalogButton(R2)) {
    ESP_LOGI(LOG_TAG, "Valor L2 = %d Valor R2 = %d",
             this->PS4->getAnalogButton(L2), this->PS4->getAnalogButton(R2));
  } else if (this->PS4->getAnalogButton(L2)) {
    // std::cout << this->PS4->getAnalogButton(L2) << std::endl;
    ESP_LOGI(LOG_TAG, "Valor L2 = %d", this->PS4->getAnalogButton(L2));
  } else if (this->PS4->getAnalogButton(R2)) {
    // std::cout << this->PS4->getAnalogButton(L2) << std::endl;
    ESP_LOGI(LOG_TAG, "Valor R2 = %d", this->PS4->getAnalogButton(R2));
  }
}

void RobotPs4Controller::task_robot_controll(void *tasks_param) {
  while (1) {
    btd_vhci_mutex_lock();
    controll_robot();
    btd_vhci_mutex_unlock();
    vTaskDelay(3);
  }

  // btd_vhci_mutex_lock();
  // btd_vhci_autoconnect(&this->ps4);
}
