#include "Arduino.h"
#include "Volante.h"
#include "Wire.h"

//* Esse arquivo contém a implementação da classe Volante, que é responsável por controlar o servo motor que controla o volante do robô

Volante::Volante(const int SERVO)
{
    this->SERVO = SERVO;
    s.attach(SERVO);
    s.write(return_angulo_inicial());
    set_angulo_base(return_angulo_inicial());
}

int Volante::return_angulo_inicial()
{
    return 90;
}

void Volante::set_angulo_base(int angulo)
{
    angulo_base = angulo;
}

void Volante::resetar_volante()
{
    s.write(angulo_base);
}

void Volante::virar_volante(int angulo)
{
    int angulo_inicial = return_angulo_inicial();
    int angulo_final = angulo_inicial + angulo;
    s.write(angulo_final);
    //TODO: Verificar o ângulo máximo que o servo pode virar
}
//virar_volante_
void Volante::virar_volante_especifico(int angulo)
{
    s.write(angulo + angulo_base);
}