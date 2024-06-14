#ifndef MotorDC_h
#define MotorDC_h
#pragma once
#include "Arduino.h"
#include "Tempo.h"

//* Este arquivo contém a declaração da classe MotorDC, que é responsável por
//* controlar o motor DC e fornecer os valores de velocidade e direção de giro do motor

class MotorDC{
    public:
        MotorDC(const int ENCA, const int ENCB, const int PWM, const int IN1, const int IN2); // Construtor da classe MotorDC

        void congirurar (int ticks_por_volta, float kp, float ki, float kd); // Função para configurar o motor
        void ligar_motor(int dir, int pwmVal);
        void ler_encoder();
        void resetar_encoder();
        void andar_reto(int velocidade_rpm);
        void andar_reto_cm(int distancia_cm, int velocidade_rpm);
        volatile double posi; // posição do motor em ticks do encoder
        double rps = 0; // velocidade ATUAL do motor em radianos por segundo
        double voltas = 0; // número de voltas do motor
        double voltas_anterior = 0; // número de voltas do motor no instante anterior, para cálculo do erro
        int encoder_volta; // valor de encoder referente a uma volta completa da roda
        float kp; // constante proporcional do controle PID
        float ki; // constante integral do controle PID
        float kd; // constante derivativa do controle PID
        int rpm_referencia; // velocidade desejada do motor, velocidade que ele buscará alcançar
        double rpm_max = 87; // velocidade máxima do motor (apenas por curiosidade, usar caso seja necessário)
        double raio_roda_cm = 6; // raio da roda em cm
        double comprimento_roda = 2 * M_PI * raio_roda_cm; //TODO: medir o comprimento da roda real
        float eprev = 0;
        float eintegral = 0; // erro acumulado pro cálculo do ki
        int dir = 1; // 1 para frente, -1 para trás (pelo menos essa é a ideia)

    private:
        int ENCA; // Cabo amarelo
        int ENCB; // Cabo branco
        int PWM;
        int IN1;
        int IN2;
        
};

#endif