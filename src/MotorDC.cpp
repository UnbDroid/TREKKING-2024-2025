#include "MotorDC.h"
#include "Arduino.h"
#include "Wire.h"
#include "Tempo.h"

//* Esse arquivo contém a implementação da classe MotorDC, que é responsável por controlar o motor DC do robô
//* e fornecer os valores de velocidade e direção de giro do motor

// MotorDC* MotorDC::instance = nullptr;/

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
  // instance = this;
}

void MotorDC::congirurar(int ticks_por_volta, float kp, float ki, float kd)
{
  this -> encoder_volta = ticks_por_volta;
  this -> kp = kp;
  this -> ki = ki;
  this -> kd = kd;
}

// Função para ligar o motor e definir a direção e a velocidade
void MotorDC::ligar_motor(int dir, int pwmVal)
{
  this -> dir = dir;
  analogWrite(PWM, pwmVal); // (pino do pwm, valor do pwm (máximo = 255))
  if (dir == 1)
  { // 1 para frente
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if (dir == -1)
  { // -1 para trás
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else
  { // 0 para parar
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}

// Função para ler o encoder do motor
void MotorDC::ler_encoder()
{
  volatile double b = digitalRead(ENCB);
  if (b > 0)
  { // Se ler pulso positivo do encoder, sentido horario
    posi++;
  }
  else
  { // Se ler pulso negativo do encoder, sentido anti-horario
    posi--;
  }
}

void MotorDC::andar_reto(int velocidade_rpm)
{
  //!
  //! Ainda não testada
  //!
  // TODO: Testar a função

  rpm_referencia = velocidade_rpm; // Velocidade de referência

  volatile double posi_atual = 0;      // posição atual do encoder
  noInterrupts();              // desabilita interrupções
  posi_atual = posi;          // atualiza a posição atual do encoder
  interrupts();               // reabilita interrupções

  voltas_anterior = voltas; // atualiza o número de voltas anterior

  voltas = posi_atual / encoder_volta;            // calcula o número de voltas do motor
  rps = (voltas - voltas_anterior) / dt; // calcula a velocidade do motor em rps

  double e = rpm_referencia - (rps * 60); // calcula o erro da velocidade em rpm

  float p = kp * e;

  eintegral += e;

  float d = kd * ((e - eprev) / dt);

  float u = p + (ki * eintegral * dt) + d; //p + (ki * eintegral*dt) + d;

  float pwmVal = fabs(u); // valor do pwm que será enviado ao motor

  if (pwmVal > 255) // Limita o valor do pwm para 255
  {
    pwmVal = 255;
  }

  // Define a direção do motor com base no valor de u
  if (u > 0)
  {
    dir = 1;
  }
  else if (u < 0)
  {
    dir = -1;
  }
  else
  {
    dir = 0;
  }

  ligar_motor(dir, pwmVal);

  eprev = e;
  
}

void MotorDC::andar_reto_cm(int distancia_cm, int velocidade_rpm)
{
  // TODO: Alterar o valor do comprimento da roda para o valor correto
  // TODO: Testar a função

  int voltas_inicio = posi / encoder_volta;
  if (distancia_cm > 0)
  {
    while (((posi / encoder_volta) - voltas_inicio)*comprimento_roda < distancia_cm)
    {
      atualizar_tempo();
      andar_reto(velocidade_rpm);
    }
  }
  else
  {
    while (((posi / encoder_volta) - voltas_inicio)*comprimento_roda > distancia_cm)
    {
      atualizar_tempo();
      andar_reto(velocidade_rpm);
    }
  }
}