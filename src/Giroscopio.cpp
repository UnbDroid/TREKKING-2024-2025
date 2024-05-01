#include "Arduino.h"
#include "Giroscopio.h"
#include "Wire.h"
#include "Tempo.h"

//* Esse arquivo contém a implementação da classe Giroscopio, que é responsável por controlar o sensor MPU9250
//* e fornecer os valores de ângulos de roll(x), pitch(y) e yaw(z)

Giroscopio::Giroscopio() { // Construtor da classe Giroscopio

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
    gyro_x_rad = imu.gyro_x_radps(); // Lê o valor do giroscópio no eixo x em radianos por segundo
    accel_x_g = imu.accel_x_mps2() / 9.81; // Lê o valor do acelerômetro no eixo x em g (aceleração da gravidade)
    roll_angle = atan2(accel_y_g, accel_z_g) * deg_to_rad; // Calcula o ângulo de roll em radianos
    roll_angle = alpha_x * (roll_angle + gyro_x_rad * dt) + (1 - alpha_x) * roll_angle; // Aplica o filtro de Kalman
    return roll_angle * deg_to_rad; // Retorna o valor do ângulo de roll
}

float Giroscopio::get_pitch()
{
    imu.Read(); // Lê os dados do sensor imu
    gyro_y_rad = imu.gyro_y_radps(); // Lê o valor do giroscópio no eixo y em radianos por segundo
    accel_y_g = imu.accel_y_mps2() / 9.81; // Lê o valor do acelerômetro no eixo y em g (aceleração da gravidade)
    pitch_angle = atan2(accel_x_g, accel_z_g) * deg_to_rad; // Calcula o ângulo de pitch em radianos
    pitch_angle = alpha_y * (pitch_angle + gyro_y_rad * dt) + (1 - alpha_y) * pitch_angle; // Aplica o filtro de Kalman
    return pitch_angle * deg_to_rad; // Retorna o valor do ângulo de pitch
}

float Giroscopio::get_yaw()
{
    imu.Read(); // Lê os dados do sensor imu
    gyro_z_rad = imu.gyro_z_radps(); // Lê o valor do giroscópio no eixo z em radianos por segundo
    if ((gyro_z_rad * dt) > 0.001 or (gyro_z_rad * dt) < (-0.001)) {yaw_angle += gyro_z_rad * dt;} // Calcula o ângulo de yaw em radianos
    return yaw_angle * deg_to_rad; // Retorna o valor do ângulo de yaw
}