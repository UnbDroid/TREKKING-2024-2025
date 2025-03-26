#include "RobotProperties.h"
#include "MotorDC.h"
#include "esp_log.h"
#include "math.h"
#define ANGULO_TESTE_CHECAR_UNIDADE 0
RobotProperties::RobotProperties(MotorDC *right_front_motor,
                                 MotorDC *right_back_motor,
                                 MotorDC *left_front_motor,
                                 MotorDC *left_back_motor) {

  this->left_front_motor = left_front_motor;
  this->left_back_motor = left_back_motor;
  this->right_front_motor = right_front_motor;
  this->right_back_motor = right_back_motor;
}
RoboVirtual RobotProperties::compute_vector_position() {
  unsigned long long current_time = esp_timer_get_time() / 1000000.0;
  this->robo_virtual.rpm_left_velocity_mean =
      (left_back_motor->return_speed() + left_front_motor->return_speed()) / 2;
  this->robo_virtual.rpm_right_velocity_mean =
      (right_back_motor->return_speed() + right_front_motor->return_speed()) /
      2;

  unsigned long long dt = current_time - this->last_time;
  unsigned long operation =
      dt *
      ((WHEEL_RADIUS_METERS * cos(ANGULO_TESTE_CHECAR_UNIDADE)) *
       (this->robo_virtual.rpm_left_velocity_mean +
        this->robo_virtual.rpm_right_velocity_mean)) /
      2;

  robo_virtual.vectorPosition.x = robo_virtual.vectorPosition.x + operation;

  ESP_LOGI("x", "Resultado da operacao: %f , velocidades: %d", operation,
           this->left_front_motor->return_speed());
  robo_virtual.vectorPosition.y =
      robo_virtual.vectorPosition.y +
      (unsigned long long)dt *
          ((WHEEL_RADIUS_METERS * sin(ANGULO_TESTE_CHECAR_UNIDADE)) *
           (this->robo_virtual.rpm_left_velocity_mean +
            this->robo_virtual.rpm_right_velocity_mean)) /
          2;

  robo_virtual.vectorPosition.anguloTheta =
      robo_virtual.vectorPosition.anguloTheta +
      dt *
          ((WHEEL_RADIUS_METERS *
            (this->robo_virtual.rpm_right_velocity_mean -
             this->robo_virtual.rpm_left_velocity_mean))) /
          2 * DISTANCE_BETWEEN_WHEELS_METERS;
  this->last_time = current_time;
  return robo_virtual;
}
