#include "RobotProperties.h"
#include "MotorDC.h"
#include "esp_log.h"
#include "math.h"
#include <cstdint>
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
void RobotProperties::compute_new_x_position(float medianRight,
                                             float medianLeft) {
  float thetaXContribuition =
      ((double)(WHEEL_RADIUS_METERS)) * cos(ANGULO_TESTE_CHECAR_UNIDADE);
  float distanceMoved = (medianRight + medianLeft);
  this->robo_virtual.vectorPosition.x = this->robo_virtual.vectorPosition.x +
                                        (thetaXContribuition * distanceMoved);
  ESP_LOGI("new_x_position", "x: %f ", this->robo_virtual.vectorPosition.x);
}
void RobotProperties::compute_new_y_position(float medianRight,
                                             float medianLeft) {
  float thetaYContribuition =
      ((double)(WHEEL_RADIUS_METERS)) * sin(ANGULO_TESTE_CHECAR_UNIDADE);
  float distanceMoved = (medianRight + medianLeft) / 2;
  this->robo_virtual.vectorPosition.y = this->robo_virtual.vectorPosition.y +
                                        (thetaYContribuition * distanceMoved);
  ESP_LOGI("new_y_position", "y: %f ", robo_virtual.vectorPosition.y);
}

float compute_median_delta_position_from_motors(MotorDC *front_motor,
                                                MotorDC *rear_motor) {
  float deltaAngularPositionFront =
      front_motor->getAngularPosition() - front_motor->last_angular_position;
  float deltaAngularPositionRear =
      rear_motor->getAngularPosition() - rear_motor->last_angular_position;
  return (deltaAngularPositionFront + deltaAngularPositionRear) / 2;
}
RoboVirtual RobotProperties::compute_vector_position() {
  float medianDeltaPosiRight = compute_median_delta_position_from_motors(
      right_front_motor, right_back_motor);
  float medianDeltaPosiLeft = compute_median_delta_position_from_motors(
      left_front_motor, left_back_motor);
  compute_new_x_position(medianDeltaPosiRight, medianDeltaPosiLeft);
  compute_new_y_position(medianDeltaPosiRight, medianDeltaPosiLeft);
  ESP_LOGI("new_position", "x: %f ,y: %f", robo_virtual.vectorPosition.x,
           robo_virtual.vectorPosition.y);
  return robo_virtual;
}
