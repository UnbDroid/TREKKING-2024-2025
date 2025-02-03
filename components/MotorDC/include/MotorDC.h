#ifndef _MOTORDC_H
#define _MOTORDC_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <esp_log.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"

class MotorDC{
    public:
        MotorDC(const int ENCA, const int ENCB, const int L_EN, const int L_PWM, const int R_PWM); // Construtor da classe MotorDC
        void parar();
        void congirurar (int ticks_por_volta, float kp, float ki, float kd); // Função para configurar o motor
        void ligar_motor(int direcao, int pwmVal);
        static QueueHandle_t gpio_evt_queue;
        void IRAM_ATTR read_encoder(void *arg);
        void set_encoder();
        void resetar_encoder();
        void andar_reto(int velocidade_rpm);
        int32_t posi = 0; // posição do motor em ticks do encoder
        double rps = 0; // velocidade ATUAL do motor em rotações por segundo
        int encoder_volta; // valor de encoder referente a uma volta completa da roda
        double comprimento_roda = 2 * 3.1415 * 6.272; //TODO: medir o raio da roda real

    private:
        int ENCA; // Cabo amarelo
        int ENCB; // Cabo branco
        int L_EN;
        int L_PWM;
        int R_PWM;
        double voltas = 0; // número de voltas do motor
        double voltas_anterior = 0; // número de voltas do motor no instante anterior, para cálculo do erro
        float kp; // constante proporcional do controle PID
        float ki; // constante integral do controle PID
        float kd; // constante derivativa do controle PID
        int rpm_referencia; // velocidade desejada do motor, velocidade que ele buscará alcançar
        double rpm_max = 87; // velocidade máxima do motor (apenas por curiosidade, usar caso seja necessário)
        float eprev = 0;
        float eintegral = 0; // erro acumulado pro cálculo do ki
        int dir = 1; // 1 para frente, -1 para trás (pelo menos essa é a ideia)
        
};


#endif