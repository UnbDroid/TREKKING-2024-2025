#include "Arduino.h"
#include "Giroscopio.h"
#include "Wire.h"
#include "Tempo.h"

//* Esse arquivo contém a implementação da classe Giroscopio, que é responsável por controlar o sensor MPU9250
//* e fornecer os valores de ângulos de roll(x), pitch(y) e yaw(z)

Giroscopio::Giroscopio() { // Construtor da classe Giroscopio

}

void Giroscopio::ligar_mpu() {

    // Início da comunicação com o MPU9250 --------------------------------------------------------------------------------

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

        imu.ConfigGyroRange(bfs::Mpu9250::GYRO_RANGE_250DPS); // Aumenta a precisão do sensor
        imu.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_2G); // Aumenta a precisão do sensor

    // --------------------------------------------------------------------------------------------------------------------

}

void Giroscopio::atualizar_leituras() {
  
  //TODO: Checar frequência de leitura do sensor MPU
  
  imu.Read();

  current_time = micros();

//   Serial.print("Current time: ");
//   Serial.println(current_time);

//   Serial.print(" | ");

//   Serial.print("Last time: ");
//   Serial.println(last_time);

//   Serial.print(" | ");

  dt_mpu = ((float) (current_time - last_time))/( 1.0e6 );

//   Serial.print("dt_mpu: ");
//   Serial.println(dt_mpu);

  last_time = current_time;

  // Get gyroscope data in radians
  gyro_x_rad = imu.gyro_x_radps();
  gyro_y_rad = imu.gyro_y_radps();
  gyro_z_rad = imu.gyro_z_radps();

  // Get accelerometer data in g-force
  accel_x_g = imu.accel_x_mps2() / 9.81;  // Convert m/s^2 to g
  accel_y_g = imu.accel_y_mps2() / 9.81;
  accel_z_g = imu.accel_z_mps2() / 9.81;

  // Calculate roll and pitch angles from accelerometer (initial estimate)
  // Consider using atan2 for proper quadrant handling
  pitch_angle = atan2(accel_x_g, accel_z_g) * deg_to_rad;
  roll_angle = atan2(accel_y_g, accel_z_g) * deg_to_rad;

  // Complementary filter update (adjust alpha as needed)
  if (primeira_leitura == false) {
    roll_angle = alpha_x * (roll_angle + gyro_x_rad * dt) + (1.0 - alpha_x) * pitch_angle;
    pitch_angle = alpha_y * (pitch_angle + gyro_y_rad * dt) + (1.0 - alpha_y) * roll_angle;
    if ((gyro_z_rad * dt) > 0.0001 or (gyro_z_rad * dt) < (-0.0001)) {yaw_angle += gyro_z_rad * dt * -1;}
  } else {
    primeira_leitura = false;
  }

}

float Giroscopio::get_x()
{
    atualizar_leituras();
    return (roll_angle * deg_to_rad); // Retorna o valor do ângulo de roll (x)
}

float Giroscopio::get_y()
{
    atualizar_leituras();
    return (pitch_angle * deg_to_rad); // Retorna o valor do ângulo de pitch (y)
}

float Giroscopio::get_z()
{
    atualizar_leituras();
    return (yaw_angle * deg_to_rad); // Retorna o valor de yaw (z)
}