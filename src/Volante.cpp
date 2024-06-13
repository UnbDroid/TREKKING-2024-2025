#include "Arduino.h"
#include "Volante.h"
#include "Wire.h"

//* Esse arquivo contém a implementação da classe Volante, que é responsável por controlar o servo motor que controla o volante do robô

Volante::Volante(const int SERVO)
{
    this->SERVO = SERVO;
}

void Volante::setup()
{
    s.attach(SERVO);
    s.write(90);
}

void Volante::resetar_volante()
{
    s.write(90);
}

void Volante::virar_volante(int angulo)
{;
    int angulo_final = (90 + angulo);
    s.write(angulo_final);
    Serial.println(s.read());
}