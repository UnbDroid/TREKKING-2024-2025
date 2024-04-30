#include "Arduino.h"
#include "Volante.h"
#include "Giroscopio.h"
#include "MotorDC.h"
#include "Pinos.h"
#include "Robo.h"
#include "Tempo.h"
#include "ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"

//TODO: Mudar o README para ter um tutorial de como usar o código
//TODO: Adicionar comentários explicativos no código

// Declarações dos objetos -----------------------------------------------------------------------------------------------------------------------------------------------------------

MotorDC motor_dc(ENCA, ENCB, PWM, IN1, IN2);
Volante volante(SERVO);
Giroscopio giroscopio;
Robo robo(motor_dc, volante, giroscopio);

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Declaração das variáveis de tempo ------------------------------------------------------------------------------------------------------------------------------------------------

double T; // tempo atual em milissegundos
double prevT; // tempo anterior em milissegundos
double dt; // diferença de tempo em segundos

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Declaração da comunicação serial do ROS ------------------------------------------------------------------------------------------------------------------------------------------

//? Quais são os tipos de mensagens que precisarão ser enviadas pelo ROS?
//TODO: Verificar se é necessário enviar mensagens de outros tipos
//TODO: Criar Publisher e Subscriber para os tipos de mensagens necessários

ros::NodeHandle  nh;

std_msgs::Float32 outputMessage;

ros::Publisher pub("info_back", &outputMessage);

void callBackFunction(const std_msgs::Float32 &inputMessage){
  outputMessage.data = inputMessage.data/3;
  pub.publish(&outputMessage);
}

ros::Subscriber<std_msgs::Float32> sub("information", &callBackFunction);

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

  // Inicialização do ROS
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
}

void loop() {
  
  // Cálculo do tempo decorrido
  T = millis(); // tempo atual em milissegundos
  dt = (T - prevT)/1000.0; // tempo decorrido em segundos em relação a última medição
  prevT = T; // atualiza o tempo anterior

  // Recebe a mensagem do ROS
  nh.spinOnce();
  delay(1);

  // Ler encoder do motor
  robo.motor.ler_encoder();

}