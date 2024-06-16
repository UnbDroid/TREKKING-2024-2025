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
  void interrupcao_encoder_esquerdo() {
    motor_dc_esquerdo.ler_encoder();
  }

  void interrupcao_encoder_direito() {
    motor_dc_direito.ler_encoder();
  }
  void interrupcao_encoder_direito() {
    motor_dc_direito.ler_encoder();
  }

  void ligar_robo() {
    // robo.ligar_robo();
    // volante.setup();
    motor_dc_esquerdo.congirurar(2100, 1.8, 1.3, 0);
    motor_dc_direito.congirurar(2100, 3.0, 2.5, 0);
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

      // Esquerda: 2200

      // while (motor_dc_esquerdo.posi < 2100) {
        //   atualizar_tempo();
      //   analogWrite(PWM_Esquerdo, 30);
      //   digitalWrite(IN1_Esquerdo, HIGH);
      //   digitalWrite(IN2_Esquerdo, LOW);
      //   Serial.println(motor_dc_esquerdo.posi);
      // }
      // analogWrite(PWM_Esquerdo, 0);
      // digitalWrite(IN1_Esquerdo, HIGH);
      // digitalWrite(IN2_Esquerdo, HIGH);
      // delay(5000);

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

        motor_dc_esquerdo.andar_reto(70);
        motor_dc_direito.andar_reto(70);

        // analogWrite(PWM_Esquerdo, 255);
        // digitalWrite(IN1_Esquerdo, HIGH);
        // digitalWrite(IN2_Esquerdo, LOW);

        // analogWrite(PWM_Direito, 255);
        // digitalWrite(IN1_Direito, HIGH);
        // digitalWrite(IN2_Direito, LOW);

        Serial.print("Encoder Esquerdo: ");
        Serial.print(motor_dc_esquerdo.rps * 60);
        Serial.print(" / ");
        Serial.print("Encoder Direito: ");
        Serial.println(motor_dc_direito.rps * 60);

        // 1 CM
        // 0,6 CM

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