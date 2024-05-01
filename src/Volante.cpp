#include "Arduino.h"
#include "Volante.h"
#include "Wire.h"

//* Esse arquivo contém a implementação da classe Volante, que é responsável por controlar o servo motor que controla o volante do robô

Volante::Volante(const int SERVO)
{
    this->SERVO = SERVO;
    volante.attach(SERVO);
}

void Volante::virar_volante(int angulo)
{
    volante.write(angulo);
    //TODO: Verificar o ângulo máximo que o servo pode virar
}