#ifndef Robo_h
#define Robo_h

#include "Arduino.h"
#include "Pinos.h"
#include "MotorDC.h"
#include "Tempo.h"
#include "Volante.h"
#include "Giroscopio.h"

//* Este arquivo contém a declaração da classe Robo, que é responsável por
//* controlar o robô e ter os comandos básicos de movimentação

class Robo {
    public:
        Robo(MotorDC& motor_esquerdo, MotorDC& motor_direito, Volante& volante, Giroscopio& giroscopio); // Construtor da classe Robo
        void andarAteCone(float distanciaAteParar);
        void resetar_encoder();
        void ler_visao();
        float retornar_posicao_x_do_cone();
        float retornar_posicao_y_do_cone();
        void andar_reto(int velocidade_rpm);
        void andar_reto_cm(int distancia_cm, int velocidade_rpm = 100);
        void virar_robo(int angulo);
        void alinhar_com_cone();
        float getAnguloCone();
        //! Pode ser que esses objetos deem erro por causa do construtor
        //! Eu tô confiando 100% no Copilot aqui, porque ele falou que tá tudo certo :D 
        //! --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

        MotorDC& motor_esquerdo; // Referência ao objeto motor_esquerdo da classe MotorDC
        MotorDC& motor_direito; // Referência ao objeto motor_esquerdo da classe MotorDC
        Volante& volante; // Referência ao objeto volante da classe Volante
        Giroscopio& giroscopio; // Referência ao objeto giroscopio da classe Giroscopio

        //! --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    
    private:
        float cone_posicao_x = 0.0; // Posição do cone no eixo x
        float cone_posicao_y = 0.0; // Posição do cone no eixo y
        float angulo_atual_x = 0.0; // Ângulo atual do robô no eixo x
        float angulo_atual_y = 0.0; // Ângulo atual do robô no eixo y
        float angulo_atual_z = 0.0; // Ângulo atual do robô no eixo z
};


#endif