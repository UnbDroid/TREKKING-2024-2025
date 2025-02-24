#include "RobotPs4Controller.h"
#include "MotorDC.h"
#include "include/RobotPs4Controller.h"
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

void RobotPs4Controller::move_foward() {};
