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

// Função para ligar todos os componentes do robô
void Robo::ligar_robo() {
    giroscopio.ligar_mpu();
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
    int rpm_referencia = velocidade_rpm; // Velocidade de referência
  
    volatile double posi_atual_esquerdo = 0;      // posição atual do encoder do motor esquerdo
    volatile double posi_atual_direito = 0;      // posição atual do encoder do motor direito
    noInterrupts();              // desabilita interrupções
    posi_atual_esquerdo = motor_esquerdo.posi;          // atualiza a posição atual do encoder
    posi_atual_direito = motor_direito.posi;          // atualiza a posição atual do encoder
    interrupts();               // reabilita interrupções
  
    motor_esquerdo.voltas_anterior = motor_esquerdo.voltas; // atualiza o número de voltas anterior
    motor_direito.voltas_anterior = motor_direito.voltas; // atualiza o número de voltas anterior
  
    motor_esquerdo.voltas = posi_atual_esquerdo / motor_esquerdo.encoder_volta;            // calcula o número de voltas do motor
    motor_direito.voltas = posi_atual_direito / motor_direito.encoder_volta;            // calcula o número de voltas do motor
    motor_esquerdo.rps = (motor_esquerdo.voltas - motor_esquerdo.voltas_anterior) / dt; // calcula a velocidade do motor em rps
    motor_direito.rps = (motor_direito.voltas - motor_direito.voltas_anterior) / dt; // calcula a velocidade do motor em rps
  
    double e_esquerdo = rpm_referencia - (motor_esquerdo.rps * 60); // calcula o erro da velocidade em rpm
    double e_direito = rpm_referencia - (motor_direito.rps * 60); // calcula o erro da velocidade em rpm
  
    float p_esquerdo = motor_esquerdo.kp * e_esquerdo;
    float p_direito = motor_direito.kp * e_direito;
  
    motor_esquerdo.eintegral += e_esquerdo;
    motor_direito.eintegral += e_direito;
  
    float d_esquerdo = motor_esquerdo.kd * ((e_esquerdo - motor_esquerdo.eprev) / dt);
    float d_direito = motor_direito.kd * ((e_direito - motor_direito.eprev) / dt);
  
    float u_esquerdo = p_esquerdo + (motor_esquerdo.ki * motor_esquerdo.eintegral * dt) + d_esquerdo; //p + (ki * eintegral*dt) + d;
    float u_direito = p_direito + (motor_direito.ki * motor_direito.eintegral * dt) + d_direito; //p + (ki * eintegral*dt) + d;
  
    float pwmVal_esquerdo = fabs(u_esquerdo); // valor do pwm que será enviado ao motor
    float pwmVal_direito = fabs(u_direito); // valor do pwm que será enviado ao motor
  
    if (pwmVal_esquerdo > 255) // Limita o valor do pwm para 255
    {
        pwmVal_esquerdo = 255;
    }

    if (pwmVal_direito > 255) // Limita o valor do pwm para 255
    {
        pwmVal_direito = 255;
    }
  
    // Define a direção do motor com base no valor de u
    if (u_esquerdo > 0)
    {
        motor_esquerdo.dir = 1;
    }
    else if (u_esquerdo < 0)
    {
        motor_esquerdo.dir = -1;
    }
    else
    {
        motor_esquerdo.dir = 0;
    }
  
    if (u_direito > 0)
    {
        motor_direito.dir = 1;
    }
    else if (u_direito < 0)
    {
        motor_direito.dir = -1;
    }
    else
    {
        motor_direito.dir = 0;
    }
  
    motor_esquerdo.ligar_motor(motor_esquerdo.dir, pwmVal_esquerdo);
    motor_direito.ligar_motor(motor_direito.dir, pwmVal_direito);

    motor_esquerdo.eprev = e_esquerdo;
    motor_direito.eprev = e_direito;
}

// Função para fazer o robô andar reto por uma distância específica
void Robo::andar_reto_cm (int distancia_cm, int velocidade_rpm) {
    motor_esquerdo.andar_reto_cm(distancia_cm, velocidade_rpm);
    motor_direito.andar_reto_cm(distancia_cm, velocidade_rpm);
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
    float angulo_atual = giroscopio.get_z();
    // Enquanto o robô não atingir o ângulo desejado, ele vira o volante e anda pra frente
    while (angulo_atual < (angulo_final - 3) or angulo_atual > (angulo_final + 3)) {
        atualizar_tempo();
        angulo_atual = giroscopio.get_z();
        Serial.print("Yaw: ");
        Serial.println(angulo_atual);
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
        if (giro_volante > 0) {
            motor_esquerdo.andar_reto(velocidade_rpm);
            motor_direito.andar_reto(velocidade_rpm - 20);
        } else {
            motor_esquerdo.andar_reto(velocidade_rpm - 20);
            motor_direito.andar_reto(velocidade_rpm);
        }
    }
    volante.resetar_volante();
}

// Função para fazer o robô alinhar com um cone (faz o mesmo que virar_robo, mas usando a visão do robô como referência para alinhar com o cone)
void Robo::alinhar_com_cone() {
    Robo::ler_visao();
    int giro_volante = 0;
    atualizar_tempo();
    float posicao_x = retornar_posicao_x_do_cone();
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
        volante.virar_volante_especifico(giro_volante);
        if (giro_volante > 0) {
            motor_esquerdo.andar_reto(80 + (abs(giro_volante) * 40 / 35));
            motor_direito.andar_reto(60 + (abs(giro_volante) * 40 / 35));
        } else {
            motor_esquerdo.andar_reto(60 + (abs(giro_volante) * 40 / 35));
            motor_direito.andar_reto(80 + (abs(giro_volante) * 40 / 35));
        }

    }
    volante.resetar_volante();
}