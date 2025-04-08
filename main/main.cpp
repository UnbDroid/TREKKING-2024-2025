#include "MotorDC.h"
#include "PS4BT.h"
#include "PinConfig.h"
#include "RobotProperties.h"
#include "EspRaspRobot.h"
#include "RobotPs4Controller.h"
#include "btd_vhci.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "nvs_flash.h"
MotorDC left_front_motor(ENCA_LEFT_FRONT, ENCB_LEFT_FRONT, L_PWM_LEFT_FRONT,
                         R_PWM_LEFT_FRONT, LEDC_CHANNEL_LEFT_FRONT_L_PWM,
                         LEDC_CHANNEL_LEFT_FRONT_R_PWM);
MotorDC left_back_motor(ENCA_LEFT_BACK, ENCB_LEFT_BACK, L_PWM_LEFT_BACK,
                        R_PWM_LEFT_BACK, LEDC_CHANNEL_LEFT_BACK_L_PWM,
                        LEDC_CHANNEL_LEFT_BACK_R_PWM);
MotorDC right_front_motor(ENCA_RIGHT_FRONT, ENCB_RIGHT_FRONT, L_PWM_RIGHT_FRONT,
                          R_PWM_RIGHT_FRONT, LEDC_CHANNEL_RIGHT_FRONT_L_PWM,
                          LEDC_CHANNEL_RIGHT_FRONT_R_PWM);
MotorDC right_back_motor(ENCA_RIGHT_BACK, ENCB_RIGHT_BACK, L_PWM_RIGHT_BACK,
                         R_PWM_RIGHT_BACK, LEDC_CHANNEL_RIGHT_BACK_L_PWM,
                         LEDC_CHANNEL_RIGHT_BACK_R_PWM);

void IRAM_ATTR read_encoder_left_front(void *arg) {
  left_front_motor.read_encoder(arg);
}

void IRAM_ATTR read_encoder_left_back(void *arg) {
  left_back_motor.read_encoder(arg);
}

void IRAM_ATTR read_encoder_right_front(void *arg) {
  right_front_motor.read_encoder(arg);
}
// ta dando problema nisso aqui
void IRAM_ATTR read_encoder_right_back(void *arg) {
  right_back_motor.read_encoder(arg);
}

void robot_setup() {
  pin_configuration();
  gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
  gpio_isr_handler_add((gpio_num_t)ENCA_LEFT_FRONT, read_encoder_left_front,
                       (void *)ENCA_LEFT_FRONT);
  gpio_isr_handler_add((gpio_num_t)ENCA_LEFT_BACK, read_encoder_left_back,
                       (void *)ENCA_LEFT_BACK);
  gpio_isr_handler_add((gpio_num_t)ENCA_RIGHT_FRONT, read_encoder_right_front,
                       (void *)ENCA_RIGHT_FRONT);
  gpio_isr_handler_add((gpio_num_t)ENCA_RIGHT_BACK, read_encoder_right_back,
                       (void *)ENCA_RIGHT_BACK);
  left_front_motor.configure_motor(300, 1.4, 1.2, 0.00001);
  left_back_motor.configure_motor(300, 1.8, 0.5, 0);
  right_front_motor.configure_motor(300, 1.3, 0.3, 0);
  right_back_motor.configure_motor(300, 1.3, 0.3, 0);
}
int vel = 120;
PS4BT PS4;
RobotPs4Controller robo(&right_front_motor, &right_back_motor,
                        &left_front_motor, &left_back_motor);
RobotProperties robotProperties(&right_front_motor, &right_back_motor,
                                &left_front_motor, &left_back_motor);
EspRaspRobot autonomousRobot(
    &right_front_motor, &right_back_motor, &left_front_motor, &left_back_motor,
    &robotProperties); // Pass the RobotProperties object to the constructor
void task_controll(void *task_params) { robo.task_robot_controll(task_params); }
void task_velocity(void *task_params) {
  while (1) {
    left_front_motor.go_forward(80);
    left_back_motor.go_forward(80);
    right_front_motor.go_forward(80);
    right_back_motor.go_forward(80);
    left_back_motor.fetch_rpm();
    left_front_motor.fetch_rpm();
    right_back_motor.fetch_rpm();
    right_front_motor.fetch_rpm();
    // ESP_LOGI("v", "%f %f %f %f", left_front_motor.current_speed_rpm,
    //          left_back_motor.current_speed_rpm,
    //          right_front_motor.current_speed_rpm,
    //          right_back_motor.current_speed_rpm);
    vTaskDelay(pdMS_TO_TICKS(30));
  }
}
void rosStuff(void *task_params) {
  autonomousRobot.update_posi_and_speed();
  autonomousRobot.micro_ros_run();
  vTaskDelay(pdMS_TO_TICKS(30));
}

extern "C" void app_main(void) {
  
  autonomousRobot.micro_ros_setup();
  esp_err_t ret;
  robot_setup();
  // initialize flash
  // ret = nvs_flash_init();
  // ret = btd_vhci_init();
  // btd_vhci_autoconnect(&PS4);
  // robo.set_controller(&PS4);
  // //   left_front_motor.reset_encoder();
  // //  right_front_motor.reset_encoder();
  // //  left_back_motor.reset_encoder();
  // //  right_back_motor.reset_encoder();
  // xTaskCreatePinnedToCore(task_controll, "ps4_loop_task", 10 * 1024, NULL, 2,
  //                         NULL, 1);
  // xTaskCreatePinnedToCore(task_velocity, "velocity", 10 * 1024, NULL, 2, NULL,
  //                         1);

  // Set all motors to go forward at 120
  xTaskCreate(task_velocity, "velocity", 10 * 1024, NULL, 2, NULL);

  xTaskCreate(rosStuff, "ros", 10 * 1024, NULL, 2, NULL);
  // float RADIO_IN_METERS = 0.06272;

  /*
  
  while (1) {
    RoboVirtual resultado = robotProperties.compute_vector_position();

    int distancia_metros_left =

        (left_front_motor.return_posi() * 2 * 3.1415 * R / 300);

    int distancia_metros_right =

        ((right_front_motor.return_posi() * 2 * 3.1415 * R / 293) +

         (right_back_motor.return_posi() * 2 * 3.1415 * R / 280)) /

        2;

    float distLF = (left_front_motor.return_posi() * 2 * 3.1415 *
                    WHEEL_RADIUS_METERS / 300);

    float distRF = (right_front_motor.return_posi() * 2 * 3.1415 *
                    WHEEL_RADIUS_METERS / 300);

    float distLB = (left_back_motor.return_posi() * 2 * 3.1415 *
                    WHEEL_RADIUS_METERS / 300);

    float distRB = (right_back_motor.return_posi() * 2 * 3.1415 *
                    WHEEL_RADIUS_METERS / 300);
    ESP_LOGI("TICKS", "LEF_FRONT %d RIGHT_FRONT %d LEFT_BACK %d RIGHT_BACK %d",
             (int)left_front_motor.return_posi(),
             (int)right_front_motor.return_posi(),
             (int)left_back_motor.return_posi(),
             (int)right_back_motor.return_posi());

    // while (distRB < 2.7 || distRF < 2.7 || distLF < 2.7 || distLB < 2.7) {

    ESP_LOGI("DISTANCIAS2",

             "LEFT_FRONT %f RIGHT_FRONT %f LEFT_BACK %f RIGHT_BACK %f", distLF,
             distRF, distLB, distRB);
    //  right_front_motor.set_motor(1, 66);
    // right_back_motor.set_motor(1, 66);
    // left_front_motor.set_motor(1, 66);
    // left_back_motor.set_motor(1, 66);
    distLF = (left_front_motor.return_posi() * 2 * 3.1415 *
              WHEEL_RADIUS_METERS / 300);

    distRF = (right_front_motor.return_posi() * 2 * 3.1415 *
              WHEEL_RADIUS_METERS / 300);

    distLB = (left_back_motor.return_posi() * 2 * 3.1415 * WHEEL_RADIUS_METERS /
              300);

    distRB = (right_back_motor.return_posi() * 2 * 3.1415 *
              WHEEL_RADIUS_METERS / 300);
    //}
    // right_front_motor.set_motor(1, 0);
    // right_back_motor.set_motor(1, 0);
    // left_front_motor.set_motor(1, 0);
    // left_back_motor.set_motor(1, 0);
    // left_front_motor.go_forward(vel);
    // left_back_motor.go_forward(vel);
    // right_front_motor.go_forward(vel);
    // right_back_motor.go_forward(vel);
    distLF = (left_front_motor.return_posi() * 2 * 3.1415 *
              WHEEL_RADIUS_METERS / 300);

    distRF = (right_front_motor.return_posi() * 2 * 3.1415 *
              WHEEL_RADIUS_METERS / 300);

    distLB = (left_back_motor.return_posi() * 2 * 3.1415 * WHEEL_RADIUS_METERS /
              300);

    distRB = (right_back_motor.return_posi() * 2 * 3.1415 *
              WHEEL_RADIUS_METERS / 300);

    ESP_LOGI("DISTANCIAS2",

             "PRONTO LEF_FRONT %f RIGHT_FRONT %f LEFT_BACK %f RIGHT_BACK %f",
             distLF, distRF, distLB, distRB);

    //  ESP_LOGI("robo", "distancia em x: %f", resultado.vectorPosition.x);
    //  double velocidade = left_front_motor.return_speed();
    //  ESP_LOGI("vel", "vel: %lf", velocidade);
    //     ESP_LOGI("Vel", "%f", right_front_motor.current_speed_rpm);
    }
    
    */
   
   vTaskDelay(pdMS_TO_TICKS(10));

}
