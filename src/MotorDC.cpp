#include "MotorDC.h"
#include "Arduino.h"
#include "Wire.h"

//* Esse arquivo contém a implementação da classe MotorDC, que é responsável por controlar o motor DC do robô
//* e fornecer os valores de velocidade e direção de giro do motor

// Construtor da classe MotorDC
MotorDC::MotorDC(const int ENCA, const int ENCB, const int PWM, const int IN1, const int IN2)
{
  this->ENCA = ENCA;
  this->ENCB = ENCB;
  this->PWM = PWM;
  this->IN1 = IN1;
  this->IN2 = IN2;
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
}

// Função para ligar o motor e definir a direção e a velocidade
void MotorDC::ligar_motor(int dir, int pwmVal)
{
  analogWrite(PWM, pwmVal); // (pino do pwm, valor do pwm (máximo = 255))
  if (dir == 1) { // 1 para frente
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (dir == -1) { // -1 para trás
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else { // 0 para parar
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }  
}

// Função para ler o encoder do motor
void MotorDC::ler_encoder()
{
  double b = digitalRead(ENCB);
  if (b > 0) { // Se ler pulso positivo do encoder
    posi++;
  } else { // Se ler pulso negativo do encoder
    posi--;
  }
}