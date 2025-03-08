#ifndef ROBOT_PS4_CONTROLLER
#define ROBOT_PS4_CONTROLLER
#include "MotorDC.h"
#include "PS4BT.h"
#include "btd_vhci.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#pragma once

static const char *LOG_TAG = "PS4-ROBOT";
typedef enum direction_enum { FOWARD = 1, BACKWARD = -1 } DIRECTION;
typedef enum trigger_bottom_enum {
  R2_TRIGGERED = 1,
  L2_TRIGGERED = -1
} TRIGGER_BOTTON;
#define MAX_VALUE_PWM 160
#define MAX_VALUE_R2_L2 255
#define START_FOWARD_ANALOG_HAT_VALUE 116
#define END_FOWARD_ANALOG_HAT_VALUE 0
#define START_BACKWARD_ANALOG_HAT_VALUE 138
#define END_BACKWARD_ANALOG_HAT_VALUE 255

class RobotPs4Controller {
public:
  RobotPs4Controller();
  RobotPs4Controller(MotorDC *right_front_motor, MotorDC *right_back_motor,
                     MotorDC *left_front_motor, MotorDC *left_back_motor);
  void task_robot_controll(void *tasks_param);
  void set_controller(PS4BT *PS4B);

private:
  const char *mac_addr_ps4_controller = "702084757537";
  void move(DIRECTION direction, int pwm_right_motors, int pwm_left_motors);
  void rotate(TRIGGER_BOTTON triggerBoton, int pwm_value);
  void controll_robot();
  MotorDC *right_front_motor;
  MotorDC *right_back_motor;
  MotorDC *left_front_motor;
  MotorDC *left_back_motor;
  PS4BT *PS4;
};

#endif
