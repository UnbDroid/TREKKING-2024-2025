#include <stdio.h>
#include <PinConfig.h>
#include <iostream>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <MotorDC.h>
#include <inttypes.h>
#include <rtc_wdt.h>
#include <esp_timer.h>

MotorDC left_front_motor(ENCA_LEFT_FRONT, ENCB_LEFT_FRONT, L_EN_LEFT_FRONT, L_PWM_LEFT_FRONT, R_PWM_LEFT_FRONT);
MotorDC left_back_motor(ENCA_LEFT_BACK, ENCB_LEFT_BACK, L_EN_LEFT_BACK, L_PWM_LEFT_BACK, R_PWM_LEFT_BACK);
MotorDC right_front_motor(ENCA_RIGHT_FRONT, ENCB_RIGHT_FRONT, L_EN_RIGHT_FRONT, L_PWM_RIGHT_FRONT, R_PWM_RIGHT_FRONT);
MotorDC right_back_motor(ENCA_RIGHT_BACK, ENCB_RIGHT_BACK, L_EN_RIGHT_BACK, L_PWM_RIGHT_BACK, R_PWM_RIGHT_BACK);

void read_encoder_left_front(void *arg)
{
    left_front_motor.read_encoder(arg);
}

void read_encoder_left_back(void *arg)
{
    left_back_motor.read_encoder(arg);
}

void read_encoder_right_front(void *arg)
{
    right_front_motor.read_encoder(arg);
}

void read_encoder_right_back(void *arg)
{
    right_back_motor.read_encoder(arg);
}

void robot_setup() {
    pin_configuration();
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add((gpio_num_t) ENCA_LEFT_FRONT, read_encoder_left_front, (void *)ENCA_LEFT_FRONT);
    gpio_isr_handler_add((gpio_num_t) ENCA_LEFT_BACK, read_encoder_left_back, (void *)ENCA_LEFT_BACK);
    gpio_isr_handler_add((gpio_num_t) ENCA_RIGHT_FRONT, read_encoder_right_front, (void *)ENCA_RIGHT_FRONT);
    gpio_isr_handler_add((gpio_num_t) ENCA_RIGHT_BACK, read_encoder_right_back, (void *)ENCA_RIGHT_BACK);
}

extern "C" void app_main(void)
{
    
    robot_setup();

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    int sentido = 1;
    int timer = 5000000;

    while (1)
    {
        std::cout<<"Posição da roda da direita da frente: "<<right_front_motor.posi<<"Posição da roda da esquerda de trás: "<<left_back_motor.posi<<std::endl;
        right_front_motor.set_motor(sentido, 128);
        left_back_motor.set_motor(sentido, 128);
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}
