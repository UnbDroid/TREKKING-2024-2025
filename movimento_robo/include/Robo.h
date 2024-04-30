#ifndef Robo_h
#define Robo_h

#include "Arduino.h"
#include "Pinos.h"
#include "MotorDC.h"
#include "Volante.h"
#include "Giroscopio.h"

class Robo {
    public:
        Robo(MotorDC& motor, Volante& volante, Giroscopio& giroscopio);
        void andar_reto(int velocidade_rpm);
        void andar_reto_cm(int distancia_cm, int velocidade_rpm = 100);
        void virar_robo(int angulo);
    private:
        MotorDC& motor;
        Volante& volante;
        Giroscopio& giroscopio;
        float angulo_atual_x = 0.0; // Ângulo atual do robô
        float angulo_atual_y = 0.0; // Ângulo atual do robô
        float angulo_atual_z = 0.0; // Ângulo atual do robô
        unsigned long T = 0; // Tempo atual em milissegundos
        unsigned long prevT = 0; // Tempo anterior em milissegundos
        float dt = 0; // Tempo decorrido em segundos
};


#endif