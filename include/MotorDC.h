#ifndef MotorDC_h
#define MotorDC_h

#include "Arduino.h"
#include "Tempo.h"

//* Este arquivo contém a declaração da classe MotorDC, que é responsável por
//* controlar o motor DC e fornecer os valores de velocidade e direção de giro do motor

class MotorDC{
    public:
        MotorDC(const int ENCA, const int ENCB, const int PWM, const int IN1, const int IN2); // Construtor da classe MotorDC

        void ligar_encoder();
        void ligar_motor(int dir, int pwmVal);
        static void ler_encoder_static();
        static MotorDC* instance;
        void ler_encoder();
        void andar_reto(int velocidade_rpm);
        void andar_reto_cm(int distancia_cm, int velocidade_rpm);

    private:
        int ENCA; // Cabo amarelo
        int ENCB; // Cabo branco
        int PWM;
        int IN1;
        int IN2;
        
        volatile double posi; // posição do motor em radianos
        double voltas = 0; // número de voltas do motor
        double voltas_anterior = 0; // número de voltas do motor no instante anterior, para cálculo do erro
        double rps = 0; // velocidade ATUAL do motor em radianos por segundo
        int rpm_referencia = 80; // velocidade desejada do motor, velocidade que ele buscará alcançar
        double rps_max = 5.75; // velocidade máxima do motor (apenas por curiosidade, usar caso seja necessário)
        double raio_roda_cm = 6; // raio da roda em cm
        double comprimento_roda = 2 * M_PI * raio_roda_cm; //TODO: medir o comprimento da roda real
        float eprev = 0;
        float eintegral = 0; // erro acumulado pro cálculo do ki
        int dir = 1; // 1 para frente, -1 para trás (pelo menos essa é a ideia)
        const int encoder_volta = 1044; // valor de encoder referente a uma volta completa da roda
        const float kp = 5.0; // constante proporcional do controle PID
        const float ki = 3.0; // constante integral do controle PID
        const float kd = 0.0; // constante derivativa do controle PID
};

#endif