#include "RobotPs4Controller.h"
#include "MotorDC.h"
#include "PS4BT.h"
#include "btd_vhci.h"
#include "esp_err.h"
#include "freertos/idf_additions.h"
#include "include/RobotPs4Controller.h"
#include "iostream"
#include <iostream>
#include <stdio.h>
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

  if (this->PS4->getAnalogButton(L2)) {
    std::cout << this->PS4->getAnalogButton(L2) << std::endl;
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
