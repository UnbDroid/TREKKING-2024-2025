#include "Arduino.h"
#include "Giroscopio.h"
#include "Wire.h"
#include "Tempo.h"

Giroscopio::Giroscopio()
{
  // Início da comunicação com o MPU9250 {

    while (!Serial) {} // Aguarda a comunicação serial ser estabelecida

    Wire.begin(); // Inicia a comunicação I2C
    Wire.setClock(400000); // Define a velocidade da comunicação I2C

    imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM); // Configura o sensor imu

  // Início da comunicação com o sensor imu

  if (!imu.Begin()) { // Se não conseguir iniciar a comunicação com o sensor imu
      Serial.println("Erro ao inicializar a comunicação com o sensor imu");
      while(1) {}
  }

  if (!imu.ConfigSrd(19)) { // Se não conseguir configurar o SRD
      Serial.println("Erro ao configurar o SRD");
      while(1) {}
  }

  // }
}

float Giroscopio::get_roll()
{
    imu.Read(); // Lê os dados do sensor imu
    gyro_x_rad = imu.gyro_x_radps();
    accel_x_g = imu.accel_x_mps2() / 9.81;
    roll_angle = atan2(accel_y_g, accel_z_g) * deg_to_rad;
    roll_angle = alpha_x * (roll_angle + gyro_x_rad * dt) + (1 - alpha_x) * roll_angle;
    return roll_angle;
}

float Giroscopio::get_pitch()
{
    imu.Read(); // Lê os dados do sensor imu
    gyro_y_rad = imu.gyro_y_radps();
    accel_y_g = imu.accel_y_mps2() / 9.81;
    pitch_angle = atan2(accel_x_g, accel_z_g) * deg_to_rad;
    pitch_angle = alpha_y * (pitch_angle + gyro_y_rad * dt) + (1 - alpha_y) * pitch_angle;
    return pitch_angle;
}

float Giroscopio::get_yaw()
{
    imu.Read(); // Lê os dados do sensor imu
    gyro_z_rad = imu.gyro_z_radps();
    if ((gyro_z_rad * dt) > 0.001 or (gyro_z_rad * dt) < (-0.001)) {yaw_angle += gyro_z_rad * dt;}
    return yaw_angle;
}