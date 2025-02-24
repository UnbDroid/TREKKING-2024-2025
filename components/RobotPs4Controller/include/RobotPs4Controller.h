#ifndef ROBOT_PS4_CONTROLLER
#define ROBOT_PS4_CONTROLLER
#include "MotorDC.h"
#include "PS4BT.h"
#include "btd_vhci.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#pragma once

class RobotPs4Controller {
public:
  RobotPs4Controller(MotorDC *right_front_motor, MotorDC *right_back_motor,
                     MotorDC *left_front_motor, MotorDC *left_back_motor);
  void move_foward();
  void move_backward();
  void stop();

private:
  const char *mac_addr_ps4_controller = "702084757537";
  MotorDC *right_front_motor;
  MotorDC *right_back_motor;
  MotorDC *left_front_motor;
  MotorDC *left_back_motor;
};

#endif
