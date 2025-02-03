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

MotorDC left_front_motor(ENCA_ESQUERDO_FRENTE, ENCB_ESQUERDO_FRENTE, L_EN_ESQUERDO_FRENTE, L_PWM_ESQUERDO_FRENTE, R_PWM_ESQUERDO_FRENTE);
MotorDC left_back_motor(ENCA_ESQUERDO_TRAS, ENCB_ESQUERDO_TRAS, L_EN_ESQUERDO_TRAS, L_PWM_ESQUERDO_TRAS, R_PWM_ESQUERDO_TRAS);
MotorDC right_front_motor(ENCA_DIREITO_FRENTE, ENCB_DIREITO_FRENTE, L_EN_DIREITO_FRENTE, L_PWM_DIREITO_FRENTE, R_PWM_DIREITO_FRENTE);
MotorDC right_back_motor(ENCA_DIREITO_TRAS, ENCB_DIREITO_TRAS, L_EN_DIREITO_TRAS, L_PWM_DIREITO_TRAS, R_PWM_DIREITO_TRAS);

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
    gpio_isr_handler_add((gpio_num_t) ENCA_ESQUERDO_FRENTE, read_encoder_left_front, (void *)ENCA_ESQUERDO_FRENTE);
    gpio_isr_handler_add((gpio_num_t) ENCA_ESQUERDO_TRAS, read_encoder_left_back, (void *)ENCA_ESQUERDO_TRAS);
    gpio_isr_handler_add((gpio_num_t) ENCA_DIREITO_FRENTE, read_encoder_right_front, (void *)ENCA_DIREITO_FRENTE);
    gpio_isr_handler_add((gpio_num_t) ENCA_DIREITO_TRAS, read_encoder_right_back, (void *)ENCA_DIREITO_TRAS);
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
        right_front_motor.ligar_motor(sentido, 128);
        left_back_motor.ligar_motor(sentido, 128);
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}
