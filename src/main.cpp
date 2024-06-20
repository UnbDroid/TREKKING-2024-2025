#include "Arduino.h"
#include "Volante.h"
#include "Giroscopio.h"
#include "MotorDC.h"
#include "Pinos.h"
#include "Robo.h"
#include "Tempo.h"

//* Este arquivo contém o código principal do robô, que junta todas as funções e estrutura a lógica principal do robô

//TODO: Mudar o README para ter um tutorial de como usar o código

//! Declarações dos objetos -----------------------------------------------------------------------------------------------------------------------------------------------------------

  MotorDC motor_dc_esquerdo(ENCA_Esquerdo, ENCB_Esquerdo, PWM_Esquerdo, IN1_Esquerdo, IN2_Esquerdo);
  MotorDC motor_dc_direito(ENCA_Direito, ENCB_Direito, PWM_Direito, IN1_Direito, IN2_Direito);
  Volante volante(SERVO);
  Giroscopio giroscopio;
  Robo robo(motor_dc_esquerdo, motor_dc_direito, volante, giroscopio);

//! --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



//! Declaração das variáveis de tempo ------------------------------------------------------------------------------------------------------------------------------------------------
  
  long T;
  long prevT; 
  double dt;

//! --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



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



//! Funções principais do código ----------------------------------------------------------------------------------------------------------------------------------------------------

  void setup() {

    //! Início da comunicação serial ---------------------------------------------------------------------------------------------------------------------------------------------------

      Serial.begin(115200);

    //! --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


    //! Inicialização dos objetos (oficial) -------------------------------------------------------------------------------------------------------------------------------------------
    
      ligar_robo();

    //! --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    //! Caminho do robô ---------------------------------------------------------------------------------------------------------------------------------------------------------------
      
 
      // while (!Serial) {
      // }
      robo.andar_reto_cm(86, 87);
      delay(1000);
      robo.virar_robo(-90);
      delay(1000);
      robo.alinhar_com_cone(15);
      volante.resetar_volante();

    //! --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  }

  void loop() {

    //! Núcleo do código ---------------------------------------------------------------------------------------------------------------------------------------------------------------
      
      //* No momento inutilizado
    
      // volante.virar_volante(0);
      // delay(1000);
      // volante.virar_volante(30);
      // delay(1000);
      // volante.virar_volante(-30);
      // delay(1000);


      // Serial.print("Posição X do cone: ");
      // Serial.println(robo.retornar_posicao_x_do_cone());
      // Serial.print("Posição Y do cone: ");
      // Serial.println(robo.retornar_posicao_y_do_cone());

      

    //! --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  }

//! -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------