#ifndef Volante_h
#define Volante_h

#include "Arduino.h"
#include "Servo.h"

//* Este arquivo contém a declaração da classe Volante, que é responsável por
//* controlar o servo motor do câmbio de direção do robô

class Volante{
    public:
        Volante(const int SERVO);
        void virar_volante(int angulo); // Função para virar o volante para um ângulo específico
    private:
        int SERVO;
        Servo volante;
};

#endif

