//! Importação das bibliotecas -----------------------------------------------------------

  #include "Arduino.h"
  #include <Wire.h>
  #include <MPU6050_light.h>
  #include "Volante.h"
  #include "MotorDC.h"
  #include "Pinos.h"
  #include "Robo.h"
  #include<SPI.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BNO055.h>
  #include <utility/imumaths.h>
  #include<SoftwareSerial.h>

//! -------------------------------------------------------------------------------------

//! Criação dos objetos -----------------------------------------------------------------

  // MPU6050 imu(Wire);
  Adafruit_BNO055 bno = Adafruit_BNO055(55);
  MotorDC motor_dc_esquerdo(ENCA_Esquerdo, ENCB_Esquerdo, PWM_Esquerdo, IN1_Esquerdo, IN2_Esquerdo);
  MotorDC motor_dc_direito(ENCA_Direito, ENCB_Direito, PWM_Direito, IN1_Direito, IN2_Direito);
  Volante volante(SERVO);
  Robo robo(motor_dc_esquerdo, motor_dc_direito, volante, bno);

//! -------------------------------------------------------------------------------------

//! Declaração das variáveis de tempo ---------------------------------------------------

  unsigned long T;
  unsigned long prevT; 
  double dt;

//! -------------------------------------------------------------------------------------
void interrupcao_encoder_esquerdo() {
  motor_dc_esquerdo.ler_encoder();
}

void interrupcao_encoder_direito() {
  motor_dc_direito.ler_encoder();
}
void resetarImu(){
  sensors_event_t leituraBno;
  bno.getEvent(&leituraBno);
  float angulo_inicial=leituraBno.orientation.x;
  
  while(angulo_inicial==0 || angulo_inicial==360){
      sensors_event_t leituraBno;
      bno.getEvent(&leituraBno);
      angulo_inicial=leituraBno.orientation.x;
  };
}

void ligar_robo() {
  Wire.begin();
  Wire.setWireTimeout(5000);
  Serial.println("Wire begin");
  // imu.begin();
  Serial.println("IMU begin");
   if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  // imu.calcOffsets();
  // float ang_inicial=imu.getAngleZ();
  // float ang_atual=imu.getAngleZ();

  // while ((ang_atual-ang_inicial) > 0.01 || (ang_atual-ang_inicial) < -0.01) {
  //   imu.calcOffsets();
  //   ang_inicial=imu.getAngleZ();
  //   delay(500);
  //   ang_atual = imu.getAngleZ();
  //   // Serial.println("Calibrei");
  // }
  volante.setup();
  motor_dc_esquerdo.congirurar(2100, 1.8, 1.3, 0);
  motor_dc_direito.congirurar(2100, 3.0, 2.0, 0);

  attachInterrupt(digitalPinToInterrupt(ENCA_Esquerdo), interrupcao_encoder_esquerdo, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_Direito), interrupcao_encoder_direito, RISING);
  pinMode(LED, OUTPUT);
}

void setup() { 
    
  //! FAVOR NÃO COLOCAR FUNÇÕES DE TESTE FORA DA ÁREA DE TESTES

  //* Funções de Setup -----------------------------------------
    
    Serial.begin(115200);
    
    Serial.setTimeout(100);
    
    ligar_robo();
    bno.setExtCrystalUse(true);
    
    // while (!Serial) {}

    // while (Serial.available() > 0) {
    //   char flush = Serial.read();
    // }

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
    robo.alinhar_com_cone(60);
    digitalWrite(LED,HIGH);
    motor_dc_direito.parar();
    motor_dc_esquerdo.parar();
    delay(7000);
    digitalWrite(LED,LOW);
    robo.virar_robo(frente, 45, 1);
    delay(1000);
    resetarImu();
    robo.alinhar_com_cone(60);
    digitalWrite(LED,HIGH);
    motor_dc_direito.parar();
    motor_dc_esquerdo.parar();
    delay(7000);
    digitalWrite(LED,LOW);
    robo.virar_robo(frente, 90, 1);
    delay(1000);
    resetarImu();
    // bno.getEvent(&leituraBno);
    // angulo_inicial=leituraBno.orientation.x;
    
    // while(angulo_inicial==0 || angulo_inicial==360){
    //     sensors_event_t leituraBno;
    //     bno.getEvent(&leituraBno);
    //     angulo_inicial=leituraBno.orientation.x;
    // };
    robo.alinhar_com_cone(60);
    digitalWrite(LED,HIGH);
    motor_dc_direito.parar();
    motor_dc_esquerdo.parar();
    delay(7000);
    digitalWrite(LED,LOW);
    robo.virar_robo(frente, -90, 1);
    delay(1000);
    resetarImu();
    robo.alinhar_com_cone(60);
    digitalWrite(LED,HIGH);
    motor_dc_direito.parar();
    motor_dc_esquerdo.parar();
    delay(7000);
    digitalWrite(LED,LOW);

    // robo.andar_reto_cm(380);143.31 233.50
    
    // robo.virar_robo(frente,-355,1);
    // digitalWrite(LED,HIGH);
    // delay(2000);
    // robo.virar_robo(tras,-350,1);
    // robo.andar_reto_cm(200);
    // robo.virar_robo(frente,45,1);
    // robo.andar_reto_cm(650);
    // robo.virar_robo(frente, -45,1);
    // robo.andar_reto_cm(800);
    // robo.alinhar_com_cone(80);
    // digitalWrite(LED, HIGH);
    // robo.resetar_encoder();
    
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