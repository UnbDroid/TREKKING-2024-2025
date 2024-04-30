#include "Arduino.h"
#include "mpu9250.h"
#include "Wire.h"
#include "Volante.h"
#include "Servo.h"
#include "Giroscopio.h"
#include "MotorDC.h"
#include "Pinos.h"
#include "Tempo.h"

//TODO: Adicionar bibliotecas e funções necessárias para o ROSSerial (Raspberry Pi-Arduino)
//TODO: Mudar o README para ter um tutorial de como usar o código

// Declarações dos objetos -----------------------------------------------------------------------------------------------------------------------------------------------------------

MotorDC motor_dc(ENCA, ENCB, PWM, IN1, IN2);
Volante volante(SERVO);
Giroscopio giroscopio;

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


// Viáveis de ângulo ----------------------------------------------------------------------------------------------------------------------------------------------------------------

float angulo_atual_x = 0.0; // Ângulo atual do robô
float angulo_atual_y = 0.0; // Ângulo atual do robô
float angulo_atual_z = 0.0; // Ângulo atual do robô

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Funções principais do código ----------------------------------------------------------------------------------------------------------------------------------------------------

void setup() {

  // Início da comunicação serial
  Serial.begin(9600);

  // Declaração de pinos do encoder do motor {
    attachInterrupt(digitalPinToInterrupt(ENCA), [](){
      motor_dc.ler_encoder();
    }, RISING);

    //}

}

void loop() {
  // Cálculo do tempo decorrido
  T = millis(); // tempo atual em milissegundos
  dt = (T - prevT)/1000.0; // tempo decorrido em segundos em relação a última medição
  prevT = T; // atualiza o tempo anterior
}