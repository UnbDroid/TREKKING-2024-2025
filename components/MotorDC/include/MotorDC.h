#ifndef _MOTORDC_H
#define _MOTORDC_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "hal/ledc_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "inttypes.h"
#include "stdio.h"
#include "iostream"

class MotorDC{
    public:
        MotorDC(const int ENCA, const int ENCB, const int L_PWM, const int R_PWM, ledc_channel_t LEDC_CHANNEL_L, ledc_channel_t LEDC_CHANNEL_R); // Construtor da classe MotorDC
        void stop_motor();
        void configure_motor(int ticks_per_turn, float kp, float ki, float kd); // Função para configurar o motor
        void set_motor(int direcao, double pwmVal);
        static QueueHandle_t gpio_evt_queue;
        void IRAM_ATTR read_encoder(void *arg);
        void set_encoder();
        void reset_encoder();
        void go_forward(int speed_rpm);
        int32_t return_posi();
        double return_speed();
        double wheel_lenght = 2 * 3.1415 * 6.272; //TODO: medir o raio da roda real

    private:
        int ENCA; // Cabo amarelo
        int ENCB; // Cabo branco
        int L_EN;
        int L_PWM;
        int R_PWM;
        ledc_channel_t LEDC_CHANNEL_L;
        ledc_channel_t LEDC_CHANNEL_R;
        const uart_port_t uart_num = UART_NUM_2;
        uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
            .rx_flow_ctrl_thresh = 122,
        };
        int ticks_per_turn; // valor de encoder referente a uma volta completa da roda
        float kp; // valor de kp para o PID
        float ki; // valor de ki para o PID
        float kd; // valor de kd para o PID
        double current_speed_pwm = 0;
        double last_error = 0; // erro anterior para o PID
        double accumulated_error = 0; // erro acumulado para o PID
        volatile int32_t posi = 0; // posição do motor em ticks do encoder
        int32_t last_posi = 0; // posição do motor em ticks do encoder
        double current_time = 0;
        double last_time = 0;

        
};


#endif