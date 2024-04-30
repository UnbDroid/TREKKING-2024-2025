#ifndef Volante_h
#define Volante_h

#include "Arduino.h"
#include "Servo.h"

class Volante{
    public:
        Volante(const int SERVO);
        void virar_volante(int angulo);
    private:
        int SERVO;
        Servo volante;
};

#endif

