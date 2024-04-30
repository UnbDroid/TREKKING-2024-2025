#ifndef Robo_h
#define Robo_h

#include "Arduino.h"
#include "Pinos.h"
#include "MotorDC.h"
#include "Volante.h"
#include "Giroscopio.h"

//* Este arquivo contém a declaração da classe Robo, que é responsável por
//* controlar o robô e ter os comandos básicos de movimentação

class Robo {
    public:
        Robo(MotorDC& motor, Volante& volante, Giroscopio& giroscopio); // Construtor da classe Robo
        void andar_reto(int velocidade_rpm); // Função para fazer o robô andar reto indefinidamente
        void andar_reto_cm(int distancia_cm, int velocidade_rpm = 100); // Função para fazer o robô andar reto por uma distância específica
        void virar_robo(int angulo); // Função para fazer o robô virar para um ângulo específico
        MotorDC& motor; // Referência ao objeto motor da classe MotorDC
        Volante& volante; // Referência ao objeto volante da classe Volante
        Giroscopio& giroscopio; // Referência ao objeto giroscopio da classe Giroscopio
    private:
        float angulo_atual_x = 0.0; // Ângulo atual do robô no eixo x
        float angulo_atual_y = 0.0; // Ângulo atual do robô no eixo y
        float angulo_atual_z = 0.0; // Ângulo atual do robô no eixo z
};


#endif