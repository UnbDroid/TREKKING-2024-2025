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
//! De novo, eu tô confiando 100% no Copilot aqui, porque ele falou que tá tudo certo :D
Robo::Robo(MotorDC& motor, Volante& volante, Giroscopio& giroscopio)
: motor(motor), volante(volante), giroscopio(giroscopio)
{

}

// Função para ligar todos os componentes do robô
void Robo::ligar_robo() {
    giroscopio.ligar_mpu();
    motor.ligar_encoder();
    volante.inicializar_volante();
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

// Função para fazer o robô andar reto indefinidamente
void Robo::andar_reto(int velocidade_rpm)
{
    motor.andar_reto(velocidade_rpm);
}

// Função para fazer o robô andar reto por uma distância específica
void Robo::andar_reto_cm (int distancia_cm, int velocidade_rpm) {
    motor.andar_reto_cm(distancia_cm, velocidade_rpm);
}

// Função para fazer o robô virar para um ângulo específico
void Robo::virar_robo(int angulo)
{
    //!
    //! Ainda não testada
    //!
    //TODO: Testar a função

    int giro_volante = 0;
    float valor_angulacao_inicial = giroscopio.get_z();
    float angulo_final = valor_angulacao_inicial + angulo;

    // Enquanto o robô não atingir o ângulo desejado, ele vira o volante e anda pra frente
    while (giroscopio.get_z() > (angulo_final + 3) or giroscopio.get_z() < (angulo_final - 3)) { //! Supostamente esse static_cast é pra converter de float pra int, mas de novo, eu tô confiando 100% no Copilot
        atualizar_tempo();
        float angulo_atual = giroscopio.get_z();
        if ((angulo_final - angulo_atual) > 0) {
            if ((angulo_final - angulo_atual) > 35) {
                giro_volante = 35;
            } else if ((angulo_final - angulo_atual) > 10) {
                giro_volante = static_cast<int>(round((angulo_final - angulo_atual)));
            } else {
                giro_volante = 10;
            }
        } else if ((angulo_final - angulo_atual) < 0) {
            if ((angulo_final - angulo_atual) < -35) {
                giro_volante = -35;
            } else if ((angulo_final - angulo_atual) < -10) {
                giro_volante = static_cast<int>(round((angulo_final - angulo_atual)));
            } else {
                giro_volante = -10;
            }
        }
        volante.virar_volante_especifico(giro_volante);
        int velocidade_rpm = 80 + (abs(giro_volante) * 40 / 35); // Velocidade de referência
        Robo::andar_reto(velocidade_rpm);
    }
    volante.resetar_volante();
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

void Robo::testar_visao() {
    Robo::ler_visao();
    if (cone_posicao_x > 0.05) {
        digitalWrite(7, HIGH);
    } else {
        digitalWrite(7, LOW);
    }
}

// Função para fazer o robô alinhar com um cone (faz o mesmo que virar_robo, mas usando a visão do robô como referência para alinhar com o cone)
void Robo::alinhar_com_cone() {
    Robo::ler_visao();
    int giro_volante = 0;
    atualizar_tempo();
    while (retornar_posicao_x_do_cone() > 0.05 or retornar_posicao_x_do_cone() < 0.05) { //! 0.05 é a tolerância, mas pode e deve ser ajustada
        float posicao_x = retornar_posicao_x_do_cone();
        atualizar_tempo();
        if (posicao_x > 0.20) {  // Se o cone estiver à direita
            giro_volante = -35;
        } else if (posicao_x < -0.20) { // Se o cone estiver à esquerda
            giro_volante = 35;
        } else if (posicao_x > 0.15) {
            giro_volante = -25;
        } else if (posicao_x < -0.15) {
            giro_volante = 25;
        } else if (posicao_x > 0.10) {
            giro_volante = -15;
        } else if (posicao_x < -0.10) {
            giro_volante = 15;
        } else if (posicao_x > 0.05) {
            giro_volante = -10;
        } else if (posicao_x < -0.05) {
            giro_volante = 10;
        }
        volante.virar_volante_especifico(giro_volante);
        Robo::andar_reto(80 + (abs(giro_volante) * 40 / 35));
    }
    volante.resetar_volante();
}