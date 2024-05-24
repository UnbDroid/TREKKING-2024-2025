#ifndef Giroscopio_h
#define Giroscopio_h

#include "Arduino.h"
#include "Wire.h"
#include "Tempo.h"
#include "mpu9250.h"

//* Este arquivo contém a declaração da classe Giroscopio, que é responsável por 
//* controlar o sensor MPU9250 e fornecer os valores de roll(x), pitch(y) e yaw(z) do sensor

class Giroscopio{
    public:
        Giroscopio(); // Construtor da classe Giroscopio
        void ligar_mpu(); // Função para ligar o sensor
        void atualizar_leituras(); // Função para atualizar as leituras do sensor
        float get_x(); // Função para obter o valor de roll(x) do sensor
        float get_y(); // Função para obter o valor de pitch(y) do sensor
        float get_z(); // Função para obter o valor de yaw(z) do sensor
    private:
        bfs::Mpu9250 imu; // Criação do objeto imu da classe Mpu9250
        int status; // Status do sensor
        const double deg_to_rad = (180/3.1415); // Fator de conversão de graus para radianos (caso precisemos) (vulgo pi para os mais íntimos)
        // Parâmetros do filtro (constantes de controle dos valores de movimento dos eixos)
        const float alpha_x = 0.45;
        const float alpha_y = 0.45;
        const float alpha_z = 0.45;
        // Ângulos de roll(x), pitch(y) e yaw(z) em graus
        float roll_angle = 0.0;
        float pitch_angle = 0.0;
        float yaw_angle = 0.0;
        // Variáveis temporárias para armazenamento dos dados do sensor
        float prev_gyro_z_rad =0.0; // Valor anterior do giroscópio em radianos
        float gyro_x_rad, gyro_y_rad, gyro_z_rad; // Valores atuais do giroscópio em radianos
        float accel_x_g, accel_y_g, accel_z_g; // Valores atuais do acelerômetro em g (aceleração da gravidade)
};

#endif