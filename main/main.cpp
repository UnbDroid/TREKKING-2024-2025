#include <stdio.h>
#include <PinConfig.h>
#include <iostream>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <MotorDC.h>
#include <inttypes.h>

static QueueHandle_t gpio_evt_queue = NULL;

uint32_t posicao = 0;

void IRAM_ATTR gpio_isr_handler(void *arg)
{
    posicao += 1;
    xQueueSendFromISR(gpio_evt_queue, &posicao, NULL);
}

void encoder_task(void *arg)
{
    uint32_t io_num;
    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            std::cout << "Posicao: " << io_num << std::endl;
        }
    }
}

MotorDC motor1(ENCA_ESQUERDO_TRAS, ENCB_ESQUERDO_TRAS, L_EN_ESQUERDO_TRAS, L_PWM_ESQUERDO_TRAS, R_PWM_ESQUERDO_TRAS);

extern "C" void app_main(void)
{
    pin_configuration();

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    xTaskCreate(encoder_task, "encoder_task", 4096, NULL, 10, NULL);

    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);

    gpio_isr_handler_add((gpio_num_t) ENCA_ESQUERDO_TRAS, gpio_isr_handler, (void *)ENCA_ESQUERDO_TRAS);

    motor1.ligar_motor(1, 255);
    

    // }
    
}