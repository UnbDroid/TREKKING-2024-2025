#ifndef _ROBOTPROPERTIES_H
#define _ROBOTPROPERTIES_H

#include "MotorDC.h"
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

typedef struct {
  float x = 0;
  float y = 0;
  float anguloTheta = 0;
} VectorPosition;

typedef struct {
  double rpm_left_velocity_mean = 0;
  double rpm_right_velocity_mean = 0;
  VectorPosition vectorPosition;
} RoboVirtual;
#define DISTANCE_BETWEEN_WHEELS_METERS 0.075

// RoboVirtual roboVirtual;
class RobotProperties {
public:
  RobotProperties(MotorDC *right_front_motor, MotorDC *right_back_motor,
                  MotorDC *left_front_motor, MotorDC *left_back_motor);
  RoboVirtual compute_vector_position();

private:
  RoboVirtual robo_virtual;
  MotorDC *right_front_motor;
  MotorDC *right_back_motor;
  MotorDC *left_front_motor;
  MotorDC *left_back_motor;
  double last_time = 0;
};
#endif
