#include "MotorDC.h"
#include "Volante.h"
#include "Giroscopio.h"
#include "Robo.h"
#include "Pinos.h"
#include "Arduino.h"
#include "Wire.h"
#include "Tempo.h"

//* Este arquivo contém a implementação da classe Robo, que é responsável por
//* controlar o robô e ter os comandos básicos de movimentação

// Construtor da classe Robo
Robo::Robo(MotorDC& motor_esquerdo, MotorDC& motor_direito, Volante& volante, Giroscopio& giroscopio)
: motor_esquerdo(motor_esquerdo), motor_direito(motor_direito), volante(volante), giroscopio(giroscopio){

}

// Função para zerar os valores dos encoderes
void Robo::resetar_encoder() {
    motor_esquerdo.resetar_encoder();
    motor_direito.resetar_encoder();
}

//Função responsável por ler e armazenar a posição do cone na visão recebida pela comunicação serial
void Robo::ler_visao() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        int commaIndex = input.indexOf(',');
        if (commaIndex != -1) {
            String float1Str = input.substring(0, commaIndex);
            String float2Str = input.substring(commaIndex + 1);
            cone_posicao_x = float1Str.toFloat(); // cone_posicao_x recebe o valor da posição x do cone
            cone_posicao_y = float2Str.toFloat(); // cone_posicao_y recebe o valor da posição y do cone
        }
    }
}

// Função para retornar a posição x do cone
float Robo::retornar_posicao_x_do_cone() { 
    Robo::ler_visao();
    return cone_posicao_x;
}

// Função para retornar a posição y do cone
float Robo::retornar_posicao_y_do_cone() {
    Robo::ler_visao();
    return cone_posicao_y;
}

// Função para fazer o robô andar reto indefinidamente
void Robo::andar_reto(int velocidade_rpm)
{
    motor_esquerdo.andar_reto(velocidade_rpm);
    motor_direito.andar_reto(velocidade_rpm);
}

// Função para fazer o robô andar reto por uma distância específica
void Robo::andar_reto_cm (int distancia_cm, int velocidade_rpm) {
    giroscopio.primeira_leitura = true; // Evitar a computação da primeira leitura do giroscópio, pois ela é estourada
    atualizar_tempo();
    float angulo_inicial = giroscopio.get_z();
    giroscopio.last_time = prevT;
    int enc_inicial_esquerdo = motor_esquerdo.posi;
    int enc_inicial_direito = motor_direito.posi;
    while (((motor_esquerdo.posi - enc_inicial_esquerdo)/motor_esquerdo.encoder_volta)*motor_esquerdo.comprimento_roda < distancia_cm && ((motor_direito.posi - enc_inicial_direito)/motor_direito.encoder_volta)*motor_direito.comprimento_roda < distancia_cm) {
        atualizar_tempo();
        andar_reto(velocidade_rpm);
        volante.virar_volante((int)(round((angulo_inicial - giroscopio.get_z())*0.5)*5));
    }
    andar_reto(0);
    volante.definir_angulo_base();
}

// Função para fazer o robô virar para um ângulo específico
void Robo::virar_robo(int angulo)
{
    int giro_volante = 0;
    giroscopio.primeira_leitura = true;
    float angulo_inicial = giroscopio.get_z();
    float angulo_final = angulo_inicial + angulo;
    float angulo_atual = giroscopio.get_z();
    int velocidade_rpm = 80 + (abs(giro_volante) * 40 / 35); // Velocidade de referência
    giroscopio.last_time = prevT;
    // Enquanto o robô não atingir o ângulo desejado, ele vira o volante e anda pra frente
    while (angulo_atual < (angulo_final - 3) or angulo_atual > (angulo_final + 3)) {
        angulo_atual = giroscopio.get_z();
        atualizar_tempo();
        Serial.print("Yaw: ");
        Serial.println(angulo_atual);
        if ((angulo_final - angulo_atual) > 0) {
            if ((angulo_final - angulo_atual) > 35) {
                giro_volante = 35;
            } else if ((angulo_final - angulo_atual) > 30) {
                giro_volante = 30;
            } else if ((angulo_final - angulo_atual) > 20) {
                giro_volante = 20;
            } else if ((angulo_final - angulo_atual) > 10) {
                giro_volante = 15;
            } else {
                giro_volante = 10;
            }
        } else if ((angulo_final - angulo_atual) < 0) {
            if ((angulo_final - angulo_atual) < -35) {
                giro_volante = -35;
            } else if ((angulo_final - angulo_atual) < -30) {
                giro_volante = -30;
            } else if ((angulo_final - angulo_atual) < -20) {
                giro_volante = -20;
            } else if ((angulo_final - angulo_atual) < -10) {
                giro_volante = -15;
            } else {
                giro_volante = -10;
            }
        }
        volante.virar_volante(giro_volante);
        if (velocidade_rpm != (80 + (abs(giro_volante) * 40 / 35))) {
            velocidade_rpm = 80 + (abs(giro_volante) * 40 / 35);
        }
        if (giro_volante > 0) {
            motor_esquerdo.andar_reto(velocidade_rpm);
            motor_direito.andar_reto(velocidade_rpm - 10);
        } else {
            motor_esquerdo.andar_reto(velocidade_rpm - 10);
            motor_direito.andar_reto(velocidade_rpm);
        }
    }
    
    volante.resetar_volante();
    motor_direito.andar_reto(0);
    motor_esquerdo.andar_reto(0);
}

// Função para fazer o robô alinhar com um cone (faz o mesmo que virar_robo, mas usando a visão do robô como referência para alinhar com o cone)
void Robo::alinhar_com_cone() {
    // Serial.begin(9600);
    // Serial.println("Alinhando");
    Robo::ler_visao();
    int giro_volante = 0;
    atualizar_tempo();
    float posicao_x = retornar_posicao_x_do_cone();
    int velocidade_rpm = 80 + (abs(giro_volante) * 40 / 35); // Velocidade de referência
    while (posicao_x > 0.05 or posicao_x < -0.05) { //! 0.05 é a tolerância, mas pode e deve ser ajustada
        atualizar_tempo();
        posicao_x = retornar_posicao_x_do_cone();
        if (posicao_x > 0.20) {
            giro_volante = -35;
        } else if (posicao_x < -0.20) {
            giro_volante = 35;
        } else if (posicao_x > 0.10) {
            giro_volante = static_cast<int>(round(posicao_x*(-100)));
        } else if (posicao_x < -0.10) {
            giro_volante = static_cast<int>(round(posicao_x*(100)));
        } else if (posicao_x > 0.05) {
            giro_volante = -10;
        } else if (posicao_x < -0.05) {
            giro_volante = 10;
        }
        volante.virar_volante(giro_volante);
        if (velocidade_rpm != (80 + (abs(giro_volante) * 40 / 35))) {
            velocidade_rpm = 80 + (abs(giro_volante) * 40 / 35);
        }
        if (giro_volante > 0) {
            motor_esquerdo.andar_reto(velocidade_rpm);
            motor_direito.andar_reto(velocidade_rpm - 10);
        } else {
            motor_esquerdo.andar_reto(velocidade_rpm - 10);
            motor_direito.andar_reto(velocidade_rpm);
        }

    }
    volante.resetar_volante();
    Serial.flush();
    if (Serial.availableForWrite() > 0) {
        Serial.println("Alinhado");
    } else {
        Serial.end();
        Serial.begin(9600);
        Serial.println("Alinhado");
    }
    Serial.end();
}