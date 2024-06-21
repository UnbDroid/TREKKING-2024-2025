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
    int tempoDeEspera = millis()+50;
    while (!Serial.available()&&millis()<tempoDeEspera) {
    }
    if(millis()>tempoDeEspera){
        cone_posicao_x=400; 
    }
    String input = Serial.readStringUntil('\n');
    int commaIndex = input.indexOf(',');
    if (commaIndex != -1) {
        String float1Str = input.substring(0, commaIndex);
        String float2Str = input.substring(commaIndex + 1);
        cone_posicao_x = float1Str.toFloat(); // cone_posicao_x recebe o valor da posição x do cone
        cone_posicao_y = float2Str.toFloat(); // cone_posicao_y recebe o valor da posição y do cone
    }
}

// Função para retornar a posição x do cone
float Robo::retornar_posicao_x_do_cone() { 
    ler_visao();
    return cone_posicao_x;
}

// Função para retornar a posição y do cone
float Robo::retornar_posicao_y_do_cone() {
    ler_visao();
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
    int velocidade_rpm = 87; // Velocidade de referência
    giroscopio.last_time = prevT;
    // Enquanto o robô não atingir o ângulo desejado, ele vira o volante e anda pra frente
    while (angulo_atual < (angulo_final - 3) or angulo_atual > (angulo_final + 3)) {
        angulo_atual = giroscopio.get_z();
        atualizar_tempo();
        if ((angulo_final - angulo_atual) > 0) {
            if ((angulo_final - angulo_atual) > 10) {
                giro_volante = 35;
            } else {
                giro_volante = 20;
            }
        } else if ((angulo_final - angulo_atual) < 0) {
            if ((angulo_final - angulo_atual) < -10) {
                giro_volante = -35;
            } else {
                giro_volante = -20;
            }
        }
        volante.virar_volante(giro_volante);
        if (giro_volante > 0) {
            motor_esquerdo.andar_reto(velocidade_rpm);
            motor_direito.andar_reto(velocidade_rpm - 5);
        } else {
            motor_esquerdo.andar_reto(velocidade_rpm - 5);
            motor_direito.andar_reto(velocidade_rpm);
        }
    }
    
    volante.resetar_volante();
    motor_direito.andar_reto(0);
    motor_esquerdo.andar_reto(0);
}
float Robo::getAnguloCone(){
    float catetoOposto = retornar_posicao_x_do_cone()*100;
    float catetoAdjacente = retornar_posicao_y_do_cone();
    float hipotenusa = pow((pow(catetoAdjacente,2)+pow(catetoOposto,2)),(1/2));
    float anguloCone=asin(catetoOposto/hipotenusa)*180/PI;
    return catetoOposto>0?anguloCone:-anguloCone;
}
void Robo::andarAteCone(float distanciaAteParar,int anguloCone){
    virar_robo(anguloCone);
    while(retornar_posicao_y_do_cone()>distanciaAteParar){
        alinhar_com_cone(distanciaAteParar);
    }
    motor_direito.ligar_motor(0,0);
    motor_esquerdo.ligar_motor(0,0);
    delay(500);
}

// Função para fazer o robô alinhar com um cone (faz o mesmo que virar_robo, mas usando a visão do robô como referência para alinhar com o cone)
void Robo::alinhar_com_cone(float distanciaAteParar) {
    // Serial.begin(9600);
    // Serial.println("Alinhando");
    int giro_volante = 0;
    atualizar_tempo();
    int tempoEspera = 50;
    float posicao_x = retornar_posicao_x_do_cone();
    while(posicao_x==400){
        atualizar_tempo();
        andar_reto_cm(3,87);
        posicao_x = retornar_posicao_x_do_cone();
    }
    int velocidade_rpm = 85; // Velocidade de referência
    float angulo_inicial = giroscopio.get_z();
    while (retornar_posicao_y_do_cone()>distanciaAteParar) { //! 0.05 é a tolerância, mas pode e deve ser ajustada
        atualizar_tempo();
        posicao_x = retornar_posicao_x_do_cone();
        giro_volante = (int) (round(posicao_x * 100));
        if(posicao_x>0.05 && posicao_x<0.1){
            volante.virar_volante(4);
        }
        else if(posicao_x>=0.13 && posicao_x<=0.17){
            volante.virar_volante(10);
        }
        else if(posicao_x>0.17){
            volante.virar_volante(13);
        }

        else if(posicao_x< -0.05 && posicao_x> -0.13){
            volante.virar_volante(-4);
        }
        else if(posicao_x<= -0.13 && posicao_x>= -0.17){
            volante.virar_volante(-10);
        }
        else if(posicao_x< -0.17){
            volante.virar_volante(-13);
        }
        else {
            volante.resetar_volante();
        }

        if (cone_posicao_x > 0.05 or cone_posicao_x < -0.05) {
            velocidade_rpm = 50;
            if (cone_posicao_x > 0.05) {
                motor_esquerdo.andar_reto(velocidade_rpm);
                motor_direito.andar_reto(velocidade_rpm - 10);
            }
            else {
                motor_esquerdo.andar_reto(velocidade_rpm - 10);
                motor_direito.andar_reto(velocidade_rpm);
            }
        } else {
            velocidade_rpm = 85;
            motor_esquerdo.andar_reto(velocidade_rpm);
            motor_direito.andar_reto(velocidade_rpm);
        }
        

        

        
        // if (giro_volante > 35) {
        //     giro_volante = 35;
        // } else if (giro_volante < -35) {
        //     giro_volante = -35;
        // }
        // volante.virar_volante(giro_volante);
        // if (giro_volante > 0) {
        //     motor_esquerdo.andar_reto(velocidade_rpm);
        //     motor_direito.andar_reto(velocidade_rpm - 10);
        // } else {
        //     motor_esquerdo.andar_reto(velocidade_rpm - 10);
        //     motor_direito.andar_reto(velocidade_rpm);
        // }
        // angulo_inicial = giroscopio.get_z();
    }
    volante.resetar_volante();
    // angulo_inicial = giroscopio.get_z();
    // while(retornar_posicao_y_do_cone()>distanciaAteParar){
    //     atualizar_tempo();
    //     andar_reto(velocidade_rpm);
    //     volante.virar_volante((int)(round((angulo_inicial - giroscopio.get_z())*0.5)*5));
    // }
    motor_direito.andar_reto(0);
    motor_esquerdo.andar_reto(0);
    delay(500);
}