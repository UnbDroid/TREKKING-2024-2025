//! Importação das bibliotecas -----------------------------------------------------------

  #include "Arduino.h"
  #include <Wire.h>
  #include <MPU6050_light.h>
  #include "Volante.h"
  #include "MotorDC.h"
  #include "Pinos.h"
  #include "Robo.h"

//! -------------------------------------------------------------------------------------

//! Criação dos objetos -----------------------------------------------------------------

  MPU6050 imu(Wire);
  MotorDC motor_dc_esquerdo(ENCA_Esquerdo, PWM_Esquerdo, IN1_Esquerdo, IN2_Esquerdo);
  MotorDC motor_dc_direito(ENCA_Direito, PWM_Direito, IN1_Direito, IN2_Direito);
  Volante volante(SERVO);
  Giroscopio giroscopio;
  Robo robo(motor_dc_esquerdo, motor_dc_direito, volante, imu);

//! -------------------------------------------------------------------------------------

//! Declaração das variáveis de tempo ---------------------------------------------------

  long T;
  long prevT; 
  double dt;

//! -------------------------------------------------------------------------------------

void interrupcao_encoder_esquerdo() {
  motor_dc_esquerdo.ler_encoder();
}

void interrupcao_encoder_direito() {
  motor_dc_direito.ler_encoder();
}

void ligar_robo() {
  Wire.begin();
  imu.begin();
  imu.calcOffsets();
  float ang_inicial=imu.getAngleZ();
  delay(500);
  float ang_atual=imu.getAngleZ();
  while (ang_atual-ang_inicial > 0.01 || ang_atual-ang_inicial < -0.01) {
    imu.calcOffsets();
    ang_inicial=imu.getAngleZ();
    delay(500);
    ang_atual = imu.getAngleZ();
  }
  volante.setup();
  motor_dc_esquerdo.congirurar(2100, 1.8, 1.3, 0);
  motor_dc_direito.congirurar(2100, 3.0, 2.0, 0);
  attachInterrupt(digitalPinToInterrupt(ENCA_Esquerdo), interrupcao_encoder_esquerdo, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_Direito), interrupcao_encoder_direito, RISING);
}

void setup() { 

  //! FAVOR NÃO COLOCAR FUNÇÕES DE TESTE FORA DA ÁREA DE TESTES

  //* Funções de Setup -----------------------------------------

    Serial.begin(115200);
    
    while (!Serial) {}

    Serial.setTimeout(100);

    ligar_robo();        

  //* ----------------------------------------------------------

  //* Caminho do robô ------------------------------------------
  
    robo.andar_reto_cm(550);
    // robo.alinhar_com_cone(60);
    // robo.virar_robo(tras, 120);
    // robo.alinhar_com_cone(60);

  //* ----------------------------------------------------------

  //! Funções de Teste -----------------------------------------


  //! ----------------------------------------------------------

}
 
void loop() {
  
  //! Usar somente para testes
  //! Deverá permanecer vazio durante as rodadas oficiais
    
}