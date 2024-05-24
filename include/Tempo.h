#ifndef Tempo_h
#define Tempo_h

#include "Arduino.h"

//* Este arquivo contém a declaração das variáveis de tempo utilizadas no projeto

extern double T;
extern double prevT; 
extern double dt;

void atualizar_tempo();

#endif