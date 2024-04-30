#include "Arduino.h"
#include "Volante.h"
#include "Wire.h"

Volante::Volante(const int SERVO)
{
    this->SERVO = SERVO;
    volante.attach(SERVO);
}

void Volante::virar_volante(int angulo)
{
    volante.write(angulo);
}

