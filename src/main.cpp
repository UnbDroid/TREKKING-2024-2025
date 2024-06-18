#include "Arduino.h"
#include "Volante.h"
#include "Giroscopio.h"
#include "MotorDC.h"
#include "Servo.h"
#include "Pinos.h"
#include "Robo.h"
#include "Tempo.h"



//* Este arquivo contém o código principal do robô, que junta todas as funções e estrutura a lógica principal do robô

//TODO: Mudar o README para ter um tutorial de como usar o código
//TODO: Adicionar comentários explicativos no código



// Declarações dos objetos -----------------------------------------------------------------------------------------------------------------------------------------------------------

  MotorDC motor_dc_esquerdo(ENCA_Esquerdo, ENCB_Esquerdo, PWM_Esquerdo, IN1_Esquerdo, IN2_Esquerdo);
  MotorDC motor_dc_direito(ENCA_Direito, ENCB_Direito, PWM_Direito, IN1_Direito, IN2_Direito);
  Volante volante(SERVO);
  Giroscopio giroscopio;
  Robo robo(motor_dc_esquerdo, motor_dc_direito, volante, giroscopio);

  Servo s;

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



// Declaração das variáveis de tempo ------------------------------------------------------------------------------------------------------------------------------------------------
  long T;
  long prevT; 
  double dt;
// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



// Funções de inicialização -------------------------------------------------------------------------------------------------------------------------------------------------------

  void interrupcao_encoder_esquerdo() {
    motor_dc_esquerdo.ler_encoder();
  }

  void interrupcao_encoder_direito() {
    motor_dc_direito.ler_encoder();
  }

  void ligar_robo() {
    giroscopio.ligar_mpu();
    volante.setup();
    motor_dc_esquerdo.congirurar(2100, 1.8, 1.3, 0);
    motor_dc_direito.congirurar(2100, 3.0, 2.0, 0);
    attachInterrupt(digitalPinToInterrupt(ENCA_Esquerdo), interrupcao_encoder_esquerdo, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA_Direito), interrupcao_encoder_direito, RISING);
  }

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



// Funções principais do código ----------------------------------------------------------------------------------------------------------------------------------------------------

  void setup() {

    //! Início da comunicação serial ---------------------------------------------------------------------------------------------------------------------------------------------------

      Serial.begin(115200);

    //! --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


    //! Inicialização dos objetos (oficial) -------------------------------------------------------------------------------------------------------------------------------------------
    
      ligar_robo();

      robo.andar_reto_cm(100, 80);
      delay(1000);
      robo.virar_robo(90);
      delay(1000);
      robo.andar_reto_cm(100, 80);
      delay(1000);
      robo.virar_robo(90);
      delay(1000);

    //! --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    // Funções de teste ---------------------------------------------------------------------------------------------------------------------------------------------------------------


      // while (millis() < 3000) {
      //   atualizar_tempo();

      //   motor_dc_esquerdo.andar_reto(30); //! Ajustar aqui a velocidade em RPM
      //   motor_dc_direito.andar_reto(30); //! Ajustar aqui a velocidade em RPM

      //   Serial.print("Encoder Esquerdo: ");
      //   Serial.print(motor_dc_esquerdo.rps * 60);
      //   Serial.print(" / ");
      //   Serial.print("Encoder Direito: ");
      //   Serial.println(motor_dc_direito.rps * 60);
      // }

      // while (millis() < 6000) {
      //   atualizar_tempo();

      //   motor_dc_esquerdo.andar_reto(50); //! Ajustar aqui a velocidade em RPM
      //   motor_dc_direito.andar_reto(50); //! Ajustar aqui a velocidade em RPM

      //   Serial.print("Encoder Esquerdo: ");
      //   Serial.print(motor_dc_esquerdo.rps * 60);
      //   Serial.print(" / ");
      //   Serial.print("Encoder Direito: ");
      //   Serial.println(motor_dc_direito.rps * 60);
      // }

      // while (millis() < 9000) {
      //   atualizar_tempo();

      //   motor_dc_esquerdo.andar_reto(20); //! Ajustar aqui a velocidade em RPM
      //   motor_dc_direito.andar_reto(20); //! Ajustar aqui a velocidade em RPM

      //   Serial.print("Encoder Esquerdo: ");
      //   Serial.print(motor_dc_esquerdo.rps * 60);
      //   Serial.print(" / ");
      //   Serial.print("Encoder Direito: ");
      //   Serial.println(motor_dc_direito.rps * 60);
      // }

      // while ((motor_dc_esquerdo.rps * 60) > 0) {
      //   atualizar_tempo();

      //   motor_dc_esquerdo.andar_reto(0); //! Ajustar aqui a velocidade em RPM
      //   motor_dc_direito.andar_reto(0); //! Ajustar aqui a velocidade em RPM

      //   Serial.print("Encoder Esquerdo: ");
      //   Serial.print(motor_dc_esquerdo.rps * 60);
      //   Serial.print(" / ");
      //   Serial.print("Encoder Direito: ");
      //   Serial.println(motor_dc_direito.rps * 60);
      // }

    // --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  }

  int var_teste = 0;
  
  void loop() {

    //! Núcleo do código ---------------------------------------------------------------------------------------------------------------------------------------------------------------
      
      // atualizar_tempo();

      // volante.virar_volante(25);
      // delay(1000);
      // volante.virar_volante(0);
      // delay(1000);
      // volante.virar_volante(-25);
      // delay(1000);
      // volante.virar_volante(0);
      // delay(1000);

    //! --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    // Funções de teste ---------------------------------------------------------------------------------------------------------------------------------------------------------------


      // if (Serial.available() > 0) {
      //   var_teste += Serial.parseInt();
      // }

      // Serial.println(var_teste);

      // robo.andar_reto(80); //! Ajustar aqui a velocidade em RPM

      // motor_dc_esquerdo.andar_reto(87); //! Ajustar aqui a velocidade em RPM
      // motor_dc_direito.andar_reto(87); //! Ajustar aqui a velocidade em RPM

      // Serial.print("Encoder Esquerdo: ");
      // Serial.print(motor_dc_esquerdo.rps * 60);
      // Serial.print(" / ");
      // Serial.print("Encoder Direito: ");
      // Serial.println(motor_dc_direito.rps * 60);

      // Serial.print("Z: ");
      // Serial.println(giroscopio.get_z());

      // delay(3000);
      // robo.virar_robo(90);

    // --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  }

// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------