#ifndef MOTOR_H
#define MOTOR_H
#include<iostream>
class MotorDC{
    public:
    unsigned short RPWM;
    unsigned short LPWM;
    unsigned short L_EN;    
    unsigned short ENC_A;
    unsigned short ENC_B;
    MotorDC();


};
    void move_foward(short velocity);
#endif