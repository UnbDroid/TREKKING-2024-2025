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
  // Volante volante(SERVO);
  // Giroscopio giroscopio;
  // Robo robo(motor_dc_esquerdo, motor_dc_direito, volante, giroscopio);

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
    // robo.ligar_robo();
    // volante.setup();
    motor_dc_esquerdo.congirurar(732, 2.2, 1.2, 0);
    motor_dc_direito.congirurar(732, 2.2, 1.2, 0);
    attachInterrupt(digitalPinToInterrupt(ENCA_Esquerdo), interrupcao_encoder_esquerdo, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA_Direito), interrupcao_encoder_direito, RISING);
  }

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


// bool teste = true;


// Funções principais do código ----------------------------------------------------------------------------------------------------------------------------------------------------

  void setup() {

    //! Início da comunicação serial ---------------------------------------------------------------------------------------------------------------------------------------------------

      Serial.begin(9600);

    //! --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


    //! Inicialização dos objetos (oficial) -------------------------------------------------------------------------------------------------------------------------------------------
    
      ligar_robo();
      // s.attach(SERVO);

    //! --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



    // Inicialização dos objetos (teste) ---------------------------------------------------------------------------------------------------------------------------------------------

      // volante.inicializar_volante();
      // giroscopio.ligar_mpu();

    // --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



    // Funções de teste ---------------------------------------------------------------------------------------------------------------------------------------------------------------

      // motor_dc.andar_reto_cm(10, 80);
      // robo.virar_robo(90);

    // --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



  }

  void loop() {

    //! Núcleo do código ---------------------------------------------------------------------------------------------------------------------------------------------------------------
    
      atualizar_tempo();

      // robo.virar_robo(90);

    //! --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    // Funções de teste ---------------------------------------------------------------------------------------------------------------------------------------------------------------

      // robo.andar_reto(80);

      // motor_dc_esquerdo.andar_reto(50); //! Ajustar aqui a velocidade em RPM
      // motor_dc_direito.andar_reto(50); //! Ajustar aqui a velocidade em RPM
      
      // if (teste == true) {

        motor_dc_esquerdo.andar_reto(-50);
        motor_dc_direito.andar_reto(-50);

        Serial.print("Encoder Esquerdo: ");
        Serial.print(motor_dc_esquerdo.rps * 60);
        Serial.print(" / ");
        Serial.print("Encoder Direito: ");
        Serial.println(motor_dc_direito.rps * 60);


      //   teste = false;
      // }


      // s.write(90);
      // Serial.println (s.read());
      // delay(1000);
      // s.write(0);
      // Serial.println (s.read());
      // delay(1000);
      // s.write(180);
      // Serial.println (s.read());
      // delay(1000);

      // Serial.print("Yaw: ");
      // Serial.println(giroscopio.get_z());

    // --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  }

// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------