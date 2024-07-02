#include "MotorDC.h"
#include "Volante.h"
#include "Giroscopio.h"
#include "Robo.h"
#include "Pinos.h"
#include "Arduino.h"
#include "Wire.h"
#include "Tempo.h"
#define NAOENCONTRADO 400
//* Este arquivo contém a implementação da classe Robo, que é responsável por
//* controlar o robô e ter os comandos básicos de movimentação

// Construtor da classe Robo
Robo::Robo(MotorDC& motor_esquerdo, MotorDC& motor_direito, Volante& volante, MPU6050&imu)
: motor_esquerdo(motor_esquerdo), motor_direito(motor_direito), volante(volante), imu(imu){

}

// Função para zerar os valores dos encoderes
void Robo::resetar_encoder() {

    motor_esquerdo.resetar_encoder();
    motor_direito.resetar_encoder();

}

//Função responsável por ler e armazenar a posição do cone na visão recebida pela comunicação serial
void Robo::ler_visao() {

    while (Serial.available() > 0) {
        Serial.read();
    }

    while (Serial.available() < 1) {
    }

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
    // unsigned long time =micros();
    // while(micros()-time<1000000){
        ler_visao();
    // }       
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

    atualizar_tempo();
    imu.update();
    float angulo_inicial = imu.getAngleZ();
    int enc_inicial_esquerdo = motor_esquerdo.posi;
    int enc_inicial_direito = motor_direito.posi;
    int rpm_referencia = velocidade_rpm;
    if (distancia_cm < 0) {
        rpm_referencia = (-1*rpm_referencia);
        while (((motor_esquerdo.posi - enc_inicial_esquerdo)/motor_esquerdo.encoder_volta)*motor_esquerdo.comprimento_roda > distancia_cm && ((motor_direito.posi - enc_inicial_direito)/motor_direito.encoder_volta)*motor_direito.comprimento_roda > distancia_cm) {
            atualizar_tempo();
            andar_reto(rpm_referencia);
            Serial.print(motor_esquerdo.posi);
            Serial.print(" ");
            Serial.print(motor_direito.posi);
            Serial.print(" ");
            Serial.println(imu.getAngleZ());
            imu.update();
            float yaw = imu.getAngleZ();
            int giro_volante = (int)(round(angulo_inicial - yaw)*-2.5);
            volante.virar_volante(giro_volante);
        }
    } else {
        while (((motor_esquerdo.posi - enc_inicial_esquerdo)/motor_esquerdo.encoder_volta)*motor_esquerdo.comprimento_roda < distancia_cm && ((motor_direito.posi - enc_inicial_direito)/motor_direito.encoder_volta)*motor_direito.comprimento_roda < distancia_cm) {
            atualizar_tempo();
            andar_reto(velocidade_rpm);
            imu.update();
            float yaw = imu.getAngleZ();
            int giro_volante = (int)(round(angulo_inicial - yaw)*2.5);
            volante.virar_volante(giro_volante);
        }
    }
    andar_reto(0);
    volante.definir_angulo_base();

}

void Robo::virar_robo(Direcao direcao, int angulo){

    int giro_volante = 0;
    imu.update();
    float angulo_final = imu.getAngleZ() + angulo;
    imu.update();
    int velocidade_rpm = 87*direcao; // Velocidade de referência

    // Enquanto o robô não atingir o ângulo desejado, ele vira o volante e anda pra frente
    while (imu.getAngleZ() < (angulo_final-3) or imu.getAngleZ() > (angulo_final+3)) {
        atualizar_tempo();
        if ((angulo_final - imu.getAngleZ()) > 0) {
            if ((angulo_final - imu.getAngleZ()) > 10) {
                giro_volante = 35;
            }
        } else if ((angulo_final - imu.getAngleZ()) < 0) {
            if ((angulo_final - imu.getAngleZ()) < -10) {
                giro_volante = -35;
            }
        }
        volante.virar_volante(giro_volante * direcao);
        if (giro_volante > 0) {
            motor_esquerdo.andar_reto(velocidade_rpm);
            motor_direito.andar_reto(velocidade_rpm - 25);
        } else {
            motor_esquerdo.andar_reto(velocidade_rpm - 25);
            motor_direito.andar_reto(velocidade_rpm);
        }
        imu.update();
    }
    
    volante.resetar_volante();
    motor_direito.andar_reto(0);
    motor_esquerdo.andar_reto(0);

}


void Robo::andarAteCone(float distanciaAteParar,int anguloCone){

    virar_robo(frente, anguloCone);
    delay(1000);
    alinhar_com_cone(distanciaAteParar);
    motor_direito.ligar_motor(0,0);
    motor_esquerdo.ligar_motor(0,0);
    delay(500);

}

float Robo::getAnguloCone(){
    float camera[2];
    float* ponteiro = camera;
    float catetoOposto = retornar_posicao_x_do_cone()*100;
    float catetoAdjacente = (float)retornar_posicao_y_do_cone();
    double primeiro = pow(catetoAdjacente,2);
    double segundo = pow(catetoOposto,2);
    double soma = primeiro + segundo;
    double hipotenusa = pow(soma, 0.5);
    double anguloCone=asin(catetoOposto/hipotenusa)*180/PI;
    camera[0]=anguloCone;
    camera[1]=catetoAdjacente;
    return anguloCone;

}

// Função para fazer o robô alinhar com um cone (faz o mesmo que virar_robo, mas usando a visão do robô como referência para alinhar com o cone)
void Robo::alinhar_com_cone(float distanciaAteParar) {

    unsigned long tempoDeEspera = millis();
    while (!Serial.available() && (millis()-tempoDeEspera)<500) {
    }
    if (millis()>tempoDeEspera) {
        cone_posicao_x=NAOENCONTRADO;
    }
    atualizar_tempo();
    imu.update();
    float angulo_inicial = imu.getAngleZ();
    float posicao_x = retornar_posicao_x_do_cone();
    float giro_volante = 0;
    int velocidade_rpm = 85; // Velocidade de referência
    while(posicao_x==NAOENCONTRADO) {
        atualizar_tempo();
        andar_reto(velocidade_rpm);
        imu.update();
        float anguloAtual = imu.getAngleZ();
        volante.virar_volante((int)(round((angulo_inicial - anguloAtual)*0.5)*5));
        posicao_x = retornar_posicao_x_do_cone();
    }

    while (retornar_posicao_y_do_cone()>distanciaAteParar) { //! 0.05 é a tolerância, mas pode e deve ser ajustada
        atualizar_tempo();
        posicao_x = retornar_posicao_x_do_cone();
        float giroVolante = getAnguloCone();
        // giro_volante = (int)(round(posicao_x*100));
    
        // if (giro_volante > 35) {
        //     giro_volante = 10;
        // } else if (giro_volante < -35) {
        //     giro_volante = -10;
        // } else if (giro_volante < 10 && giro_volante > 5) {
        //     giro_volante = 4;
        // } else if (giro_volante > -10 && giro_volante < -5) {
        //     giro_volante = -4;
        // } else if (giro_volante <= 5 && giro_volante >= -5) {
        //     giro_volante = 0;
        // }

        
        if (posicao_x > 0.04 || posicao_x < -0.04) {
            volante.virar_volante(giroVolante);
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
    }

    motor_direito.andar_reto(0);
    motor_esquerdo.andar_reto(0);
    volante.resetar_volante();
    
    delay(500);

}