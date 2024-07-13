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
  MotorDC motor_dc_esquerdo(ENCA_Esquerdo, ENCB_Esquerdo, PWM_Esquerdo, IN1_Esquerdo, IN2_Esquerdo);
  MotorDC motor_dc_direito(ENCA_Direito, ENCB_Direito, PWM_Direito, IN1_Direito, IN2_Direito);
  Volante volante(SERVO);
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
void interrupcaoImu(){
  imu.update();
}
void ligar_robo() {
  Wire.begin();
  Wire.setWireTimeout(5000);
  Serial.println("Wire begin");
  imu.begin();
  Serial.println("IMU begin");
  
  imu.calcOffsets();
  float ang_inicial=imu.getAngleZ();
  float ang_atual=imu.getAngleZ();

  while ((ang_atual-ang_inicial) > 0.01 || (ang_atual-ang_inicial) < -0.01) {
    imu.calcOffsets();
    ang_inicial=imu.getAngleZ();
    delay(500);
    ang_atual = imu.getAngleZ();
    // Serial.println("Calibrei");
  }
  volante.setup();
  motor_dc_esquerdo.congirurar(2100, 1.8, 1.3, 0);
  motor_dc_direito.congirurar(2100, 3.0, 2.0, 0);

  attachInterrupt(digitalPinToInterrupt(ENCA_Esquerdo), interrupcao_encoder_esquerdo, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_Direito), interrupcao_encoder_direito, RISING);
  pinMode(LED, OUTPUT);
}
void irConeEGirar(Direcao direcao, int giro){
    robo.alinhar_com_cone(60);
    digitalWrite(LED,HIGH);
    motor_dc_direito.parar();
    motor_dc_esquerdo.parar();
    delay(7000);
    digitalWrite(LED,LOW);
    robo.virar_robo(direcao, giro, 1);
    resetarImu();
}
void setup() { 

  //! FAVOR NÃO COLOCAR FUNÇÕES DE TESTE FORA DA ÁREA DE TESTES

  //* Funções de Setup -----------------------------------------
    
    Serial.begin(115200);
    
    Serial.setTimeout(100);
    
    ligar_robo();
    
    while (!Serial) {}

    while (Serial.available() > 0) {
      char flush = Serial.read();
    }

    digitalWrite(LED, HIGH);
    while (Serial.available() < 1) {}
    digitalWrite(LED, LOW);
  
  //* ----------------------------------------------------------

  //* Caminho do robô ------------------------------------------
    // robo.virar_robo(frente,3600);

    // Serial.println("vou andar");
    // robo.andar_reto_cm(200,80,0);
    // motor_dc_direito.parar();
    // motor_dc_esquerdo.parar();
    irConeEGirar(frente,35);
    // robo.alinhar_com_cone(60);
    // digitalWrite(LED,HIGH);
    // motor_dc_direito.parar();
    // motor_dc_esquerdo.parar();
    // delay(7000);
    // digitalWrite(LED,LOW);
    // robo.virar_robo(frente, 45, 1);
    // delay(1000);
    // resetarImu();
    irConeEGirar(frente,110);
    // robo.alinhar_com_cone(60);
    // digitalWrite(LED,HIGH);
    // motor_dc_direito.parar();
    // motor_dc_esquerdo.parar();
    // delay(7000);
    // digitalWrite(LED,LOW);
    // robo.virar_robo(frente, 90, 1);
    // delay(1000);
    // resetarImu();
    irConeEGirar(frente,-100);
    irConeEGirar(frente,-180);
    // irConeEGirar(frente,-140);
    // bno.getEvent(&leituraBno);
    // angulo_inicial=leituraBno.orientation.x;
    
    // while(angulo_inicial==0 || angulo_inicial==360){
    //     sensors_event_t leituraBno;
    //     bno.getEvent(&leituraBno);
    //     angulo_inicial=leituraBno.orientation.x;
    // };
    // robo.alinhar_com_cone(60);
    // digitalWrite(LED,HIGH);
    // motor_dc_direito.parar();
    // motor_dc_esquerdo.parar();
    // delay(7000);
    // digitalWrite(LED,LOW);
    // robo.virar_robo(frente, -90, 1);
    // delay(1000);
    // resetarImu();
    // robo.alinhar_com_cone(60);
    // digitalWrite(LED,HIGH);
    // motor_dc_direito.parar();
    // motor_dc_esquerdo.parar();
    // delay(7000);
    // digitalWrite(LED,LOW);

    // robo.andar_reto_cm(380);143.31 233.50
    
    // robo.virar_robo(frente,-355,1);
    // digitalWrite(LED,HIGH);
    // delay(2000);
    // robo.andar_reto_cm(700);
    // robo.virar_robo(frente, 90);
    // digitalWrite(LED, HIGH);
    // robo.resetar_encoder();
    // delay(2000);
    // robo.andar_reto_cm(50);
    
  //* ----------------------------------------------------------

  //! Funções de Teste -----------------------------------------


  //! ----------------------------------------------------------
  
}
 
void loop() {
  // volante.virar_volante(20);
  // delay(1000);
  // volante.virar_volante(0);
  // delay(1000);
  // volante.virar_volante(-20);
  // delay(1000);
  // volante.virar_volante(0);
  // delay(1000);
  
  //! Usar somente para testes
  //! Deverá permanecer vazio durante as rod  adas oficiais

}