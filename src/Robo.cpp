#include "MotorDC.h"
#include "Volante.h"
#include "Giroscopio.h"
#include "Robo.h"
#include "Pinos.h"
#include "Arduino.h"
#include "Wire.h"
#include "Tempo.h"

Robo::Robo(MotorDC& motor, Volante& volante, Giroscopio& giroscopio)
: motor(motor), volante(volante), giroscopio(giroscopio)
{
    Serial.println("Robo construido");
}

void Robo::andar_reto(int velocidade_rpm)
{
    //!
    //! Ainda não testada
    //!
    motor.rpm_referencia = velocidade_rpm; // Velocidade de referência
  
    double posi_atual = 0; // posição atual do encoder
    noInterrupts(); // desabilita interrupções
    posi_atual = motor.posi; // atualiza a posição atual do encoder
    interrupts(); // reabilita interrupções

    motor.voltas_anterior = motor.voltas; // atualiza o número de voltas anterior

    motor.voltas = posi_atual/motor.encoder_volta; // calcula o número de voltas do motor
    motor.rps = (motor.voltas - motor.voltas_anterior)/dt; // calcula a velocidade do motor em rps

    double e = motor.rpm_referencia - (motor.rps * 60); // calcula o erro da velocidade em rpm

    float p = motor.kp * e;

    motor.eintegral += e;

    float d = motor.kd * (e - motor.eprev)/dt;

    float u = p + (motor.ki * motor.eintegral)+ d;

    float pwmVal = fabs(u); // valor do pwm que será enviado ao motor
    if(pwmVal > 255) {
        pwmVal = 255;
    }

    if(u > 0){
        motor.dir = 1;
    }
    else if(u < 0){
        motor.dir = -1;
    }
    else{
        motor.dir = 0;
    }

    motor.ligar_motor(motor.dir, pwmVal);
}

void Robo::virar_robo(int angulo)
{
    //!
    //! Ainda não testada
    //!

    int giro_volante = 0; // Valor de giro do volante
    float valor_angulacao_inicial = giroscopio.get_yaw(); // Valor atual do ângulo de yaw (z)
    // Enquanto o robô não atingir o ângulo desejado
    while (static_cast<int>(giroscopio.get_yaw()) != static_cast<int>(valor_angulacao_inicial + angulo)) {
        if ((valor_angulacao_inicial + angulo) - giroscopio.get_yaw() > 35) { // Se a diferença entre o ângulo desejado e o atual for menor que 10 graus
          giro_volante = 35;
        } else if ((valor_angulacao_inicial + angulo) - giroscopio.get_yaw() < (-35)) { // Se a diferença entre o ângulo desejado e o atual for maior que 10 graus
          giro_volante = -35;
        } else {
          giro_volante = (valor_angulacao_inicial + angulo) - giroscopio.get_yaw(); // Gira o volante para o ângulo desejado
        }
        volante.virar_volante(giro_volante); // O volante gira para o ângulo desejado
        int velocidade_rpm = 80 + (abs(giro_volante) * 40 / 35); // Velocidade de referência
        motor.ligar_motor(1, velocidade_rpm); // O robô anda reto
    }
}