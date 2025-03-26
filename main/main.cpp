#include "MotorDC.h"
#include "PS4BT.h"
#include "PinConfig.h"
#include "RobotPs4Controller.h"
#include "btd_vhci.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
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

void read_encoder_left_front(void *arg) { left_front_motor.read_encoder(arg); }

void read_encoder_left_back(void *arg) { left_back_motor.read_encoder(arg); }

void read_encoder_right_front(void *arg) {
  right_front_motor.read_encoder(arg);
}

void read_encoder_right_back(void *arg) { right_back_motor.read_encoder(arg); }

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
  left_front_motor.configure_motor(300, 1, 1, 0);
  left_back_motor.configure_motor(300, 1, 1, 0);
  right_front_motor.configure_motor(300, 2, 1, 0);
  right_back_motor.configure_motor(300, 2, 1, 0);
}

PS4BT PS4;
RobotPs4Controller robo(&right_front_motor, &right_back_motor,
                        &left_front_motor, &left_back_motor);

typedef struct {
  int x = 0;
  int y = 0;
  float anguloTheta = 0;
} VetorPosicao;

typedef struct {
  int vel_angular_right = 0;
  int vel_angular_left = 0;
  int pos_angular_right = 0;
  int pos_angular_left = 0;
  double distancia_metros_right = 0;
  double distancia_metros_left = 0;
  int vel_linear_robo = 0;
  int vel_linear_left = 0;
  int vel_linear_right = 0;
  int x_dot = 0;
  int y_dot = 0;
  float theta_dot = 0;
} RoboVirtual;

void task_controll(void *task_params) { robo.task_robot_controll(task_params); }
RoboVirtual robovirtual;
VetorPosicao vetorPosicao;

extern "C" void app_main(void) {
  esp_err_t ret;
  robot_setup();
  // initialize flash
  ret = nvs_flash_init();
  ret = btd_vhci_init();
  btd_vhci_autoconnect(&PS4);
  robo.set_controller(&PS4);
  vetorPosicao.x = 0;
  vetorPosicao.y = 0;
  vetorPosicao.anguloTheta = 0;
  left_front_motor.reset_encoder();
  right_front_motor.reset_encoder();
  left_back_motor.reset_encoder();
  right_back_motor.reset_encoder();
  xTaskCreatePinnedToCore(task_controll, "ps4_loop_task", 10 * 1024, NULL, 2,
                          NULL, 1);
  float RADIO_IN_METERS = 0.06272;
  while (1) {
    robovirtual.distancia_metros_left =
        ((left_back_motor.return_posi() * left_back_motor.wheel_lenght /
          TICKS_PER_ROTATIONS) +
         (left_front_motor.return_posi() * left_front_motor.wheel_lenght /
          TICKS_PER_ROTATIONS)) /
        2;
    robovirtual.distancia_metros_right =
        ((right_front_motor.return_posi() * right_front_motor.wheel_lenght /
          TICKS_PER_ROTATIONS) +
         (right_back_motor.return_posi() * right_back_motor.wheel_lenght /
          TICKS_PER_ROTATIONS)) /
        2;

    float theta = (robovirtual.distancia_metros_right -
                   robovirtual.distancia_metros_left) /
                  27.1;
    vetorPosicao.anguloTheta = theta;
    float distLF = (left_front_motor.return_posi() * 2 * 3.1415 *
                    WHEEL_RADIUS_METERS / 300);

    float distRF = (right_front_motor.return_posi() * 2 * 3.1415 *
                    WHEEL_RADIUS_METERS / 300);

    float distLB = (left_back_motor.return_posi() * 2 * 3.1415 *
                    WHEEL_RADIUS_METERS / 300);

    float distRB = (right_back_motor.return_posi() * 2 * 3.1415 *
                    WHEEL_RADIUS_METERS / 300);

    // ESSA PARTE EMBAIXO DEU ERRO NA COMPILAÇÃO
    // --------------------------------------

    // ESP_LOGI("DISTANCIAS",
    //          "LEFT_FRONT %d RIGHT_FRONT %d LEFT_BACK %d RIGHT_BACK %d",
    //          left_front_motor.return_posi(), right_front_motor.return_posi(),
    //          left_back_motor.return_posi(), right_back_motor.return_posi());
    ESP_LOGI("DISTANCIAS2", "LEFT %f RIGHT %f", (distLF + distLB) / 2,
             (distRF + distRB) / 2);

    // ESP_LOGI("POS_ANGULAR",
    //          "Posicao esquerda %d Posicao direita %d Angulo theta %f",
    //          right_back_motor.return_posi(), right_front_motor.return_posi(),
    //          vetorPosicao.anguloTheta * 180 / 3.1415);

    // --------------------------------------------------------------------------------
    // I (104311) distancias_robo_virtual: distancia em metros esquerda:
    // 0.788129, direita: -0.000626

    // ESP_LOGI("distancias_robo_virtual", "distancia em metros esquerda: %f,
    // direita: %f",
    // robovirtual.distancia_metros_left,robovirtual.distancia_metros_right);
    ESP_LOGI("ANGULO ", "Angulo %f", vetorPosicao.anguloTheta);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
