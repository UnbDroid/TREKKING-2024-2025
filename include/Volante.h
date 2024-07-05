#ifndef Volante_h
#define Volante_h

#include "Arduino.h"
#include "Servo.h"

//* Este arquivo contém a declaração da classe Volante, que é responsável por
//* controlar o servo motor do câmbio de direção do robô

class Volante{
    public:
        Volante(const int SERVO);
        void setup();
        void definir_angulo_base(); // Função para definir o ângulo base do volante
        void resetar_volante(); // Função para resetar o volante para a posição inicial
        void virar_volante(int angulo); // Função para virar o volante em ângulos
    private:
        int SERVO;
        int angulo_base = 90;
        Servo s;
};

#endif

