#ifndef Volante_h
#define Volante_h

#include "Arduino.h"
#include "Servo.h"

//* Este arquivo contém a declaração da classe Volante, que é responsável por
//* controlar o servo motor do câmbio de direção do robô

class Volante{
    public:
        Volante(const int SERVO);
        int return_angulo_inicial();
        void set_angulo_base(int angulo);
        void resetar_volante(); // Função para resetar o volante para a posição inicial
        void virar_volante(int angulo); // Função para virar o volante em ângulos
        void virar_volante_especifico(int angulo); // Função para virar o volante para a esquerda
    private:
        int SERVO;
        Servo s;
        int angulo_base = 0;
};

#endif

