#include <stdio.h>
#include <PinConfig.h>
#include <iostream>

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          OUTPUT_ESQUERDO_FRENTE // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (128) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (128) // Frequency in Hertz. Set frequency at 4 kH
                                       //
extern "C" void app_main(void)
{   
    pin_configuration();
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    // while(1){
        std::cout << "Hello World" << std::endl;

    // }
    
}
