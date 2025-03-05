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
  RobotPs4Controller();
  RobotPs4Controller(MotorDC *right_front_motor, MotorDC *right_back_motor,
                     MotorDC *left_front_motor, MotorDC *left_back_motor);
  void task_robot_controll(void *tasks_param);
  void set_controller(PS4BT *PS4B);

private:
  const char *mac_addr_ps4_controller = "702084757537";
  void move_foward();
  void move_backward();
  void controll_robot();
  void stop();
  MotorDC *right_front_motor;
  MotorDC *right_back_motor;
  MotorDC *left_front_motor;
  MotorDC *left_back_motor;
  PS4BT *PS4;
};

#endif
