// Inclui a biblioteca Wire que possui as funções da comunicação I2C:
#include "Arduino.h"
#include <Wire.h>
#include <MPU6050_light.h>
#include "Volante.h"
#include "Giroscopio.h"
#include "MotorDC.h"
#include "Pinos.h"
#include "Robo.h"
// #define SCREEN_WIDTH 128 // OLED display width, in pixels
// #define SCREEN_HEIGHT 64 // OLED display height, in pixels
// // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);
 
 
MPU6050 imu(Wire);
unsigned long timer = 0;
MotorDC motor_dc_esquerdo(ENCA_Esquerdo, ENCB_Esquerdo, PWM_Esquerdo, IN1_Esquerdo, IN2_Esquerdo);
MotorDC motor_dc_direito(ENCA_Direito, ENCB_Direito, PWM_Direito, IN1_Direito, IN2_Direito);
Volante volante(SERVO);
Giroscopio giroscopio;
Robo robo(motor_dc_esquerdo, motor_dc_direito, volante, imu);
long T;
long prevT; 
double dt;

 void interrupcao_encoder_esquerdo() {
    motor_dc_esquerdo.ler_encoder();
  }

  void interrupcao_encoder_direito() {
    motor_dc_direito.ler_encoder();
  }

  void ligar_robo() {
    // giroscopio.ligar_mpu();
    volante.setup();
    motor_dc_esquerdo.congirurar(2100, 1.8, 1.3, 0);
    motor_dc_direito.congirurar(2100, 3.0, 2.0, 0);
    attachInterrupt(digitalPinToInterrupt(ENCA_Esquerdo), interrupcao_encoder_esquerdo, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA_Direito), interrupcao_encoder_direito, RISING);
  }


void setup() {
  Serial.begin(115200);                          // Ensure serial monitor set to this value also             

  Wire.begin();
  imu.begin();
  Serial.println("Antes de calibrar");
  imu.calcGyroOffsets();
  Serial.println("Depois de calibrar");                          // This does the calibration;
  ligar_robo();  
//   display.println(F("Calculating gyro offset, do not move MPU6050"));
//   display.display();        
  
  
//   display.setTextSize(2);          
robo.virar_robo(frente,90);
}
 
void loop() {
  // mpu.update();  
  
  // if((millis()-timer)>10)                         // print data every 10ms
  // {                                           
  //   // display.clearDisplay();                       // clear screen
  //   // display.setCursor(0,0);                         
  //   Serial.print("P : ");
  //   Serial.print(mpu.getAngleX());
  //   Serial.print(" R : ");
  //   Serial.print(mpu.getAngleY());
  //   Serial.print(" Y : ");
  //   Serial.println(mpu.getAngleZ());
  //   // display.display();                            // display data
  //   timer = millis();  
  // }
}