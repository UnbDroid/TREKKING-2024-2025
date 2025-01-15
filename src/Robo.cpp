#include "MotorDC.h"
// #include "Volante.h"
// #include "Giroscopio.h"
#include "Robo.h"
#include "Pinos.h"
#include "Arduino.h"
#include "Wire.h"
#include "Tempo.h"
#define NAOENCONTRADO 10000
//* Este arquivo contém a implementação da classe Robo, que é responsável por
//* controlar o robô e ter os comandos básicos de movimentação

// Construtor da classe Robo
// Robo::Robo(MotorDC& motor_esquerdo, MotorDC& motor_direito, MPU6050& imu)
// : motor_esquerdo(motor_esquerdo), motor_direito(motor_direito), imu(imu) {

// }
Robo::Robo(MotorDC& motor_esquerdo, MotorDC& motor_direito, Adafruit_BNO055& bno)
: motor_esquerdo(motor_esquerdo), motor_direito(motor_direito), bno(bno) {

}

// Função para zerar os valores dos encoderes
void Robo::resetar_encoder() {
    motor_esquerdo.resetar_encoder();
    motor_direito.resetar_encoder();
}

//Função responsável por ler e armazenar a posição do cone na visão recebida pela comunicação Serial
void Robo::ler_visao() {
    while (Serial.available() < 1) {}

    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        if (input == "Nada") {
            cone_posicao_x = 0;
            cone_posicao_y = NAOENCONTRADO;
            return;
        } else {
            int commaIndex = input.indexOf(',');
            if (commaIndex != -1) {
                String float1Str = input.substring(0, commaIndex);
                String float2Str = input.substring(commaIndex + 1);
                cone_posicao_x = float1Str.toFloat(); // cone_posicao_x recebe o valor da posição x do cone
                cone_posicao_y = float2Str.toFloat(); // cone_posicao_y recebe o valor da posição y do cone
            }
        }
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
void Robo::andar_reto_cm (int distancia_cm, int velocidade_rpm, int flag){
    atualizar_tempo();
    sensors_event_t leituraBno;
    bno.getEvent(&leituraBno);
    float angulo_inicial = leituraBno.orientation.x;
    while(angulo_inicial==0 || angulo_inicial==360){
        sensors_event_t leituraBno;
        bno.getEvent(&leituraBno);
        angulo_inicial=leituraBno.orientation.x;
    };
    Serial.println(angulo_inicial);
    
    delay(2000);
    int enc_inicial_esquerdo = motor_esquerdo.posi;
    int enc_inicial_direito = motor_direito.posi;
    int rpm_referencia = velocidade_rpm;
    if (distancia_cm < 0) {
        rpm_referencia = (-1*rpm_referencia);
        while (((motor_esquerdo.posi - enc_inicial_esquerdo)/motor_esquerdo.encoder_volta)*motor_esquerdo.comprimento_roda > distancia_cm && ((motor_direito.posi - enc_inicial_direito)/motor_direito.encoder_volta)*motor_direito.comprimento_roda > distancia_cm) {
            atualizar_tempo();
            andar_reto(rpm_referencia);
            sensors_event_t leituraBno;
            bno.getEvent(&leituraBno);
            float yaw = leituraBno.orientation.x;
        }
    }
    else {
        int erroTotal,erroAtual =0;
        while (((motor_esquerdo.posi - enc_inicial_esquerdo)/motor_esquerdo.encoder_volta)*motor_esquerdo.comprimento_roda < distancia_cm && ((motor_direito.posi - enc_inicial_direito)/motor_direito.encoder_volta)*motor_direito.comprimento_roda < distancia_cm) {
            atualizar_tempo();
            andar_reto(rpm_referencia);
            sensors_event_t leituraBno;
            bno.getEvent(&leituraBno);
            float yaw = leituraBno.orientation.x;
            float ki = 0.9;
            erroAtual = angulo_inicial-yaw;
            erroTotal += erroAtual;
        }
    }
}
// void Robo::andar_reto_cm (int distancia_cm, int velocidade_rpm) {
//     atualizar_tempo();
//     unsigned long tempo = millis();
//     imu.update();
//     float angulo_inicial = imu.getAngleZ();
//     int enc_inicial_esquerdo = motor_esquerdo.posi;
//     int enc_inicial_direito = motor_direito.posi;
//     int rpm_referencia = velocidade_rpm;
//     if (distancia_cm < 0) {
//         rpm_referencia = (-1*rpm_referencia);
//         while (((motor_esquerdo.posi - enc_inicial_esquerdo)/motor_esquerdo.encoder_volta)*motor_esquerdo.comprimento_roda > distancia_cm && ((motor_direito.posi - enc_inicial_direito)/motor_direito.encoder_volta)*motor_direito.comprimento_roda > distancia_cm) {
//             atualizar_tempo();
//             andar_reto(rpm_referencia);
//             if (millis() - tempo > 30) {
//                 imu.update();
//                 tempo = millis();
//             }
//             float yaw = imu.getAngleZ();
//         }
//     } else {
//         int erroTotal,erroAtual =0;
//         while (((motor_esquerdo.posi - enc_inicial_esquerdo)/motor_esquerdo.encoder_volta)*motor_esquerdo.comprimento_roda < distancia_cm && ((motor_direito.posi - enc_inicial_direito)/motor_direito.encoder_volta)*motor_direito.comprimento_roda < distancia_cm) {
//             atualizar_tempo();
//             andar_reto(rpm_referencia);
//             if (millis() - tempo > 30) {
//                 imu.update();
//                 tempo = millis();
//             }
//             float yaw = imu.getAngleZ();
//             float ki = 0.9;
//             erroAtual = angulo_inicial-yaw;
//             erroTotal += erroAtual;
//         }
//     }
// }

// Função para fazer o robô virar
void Robo::virar_robo(Direcao direcao, int angulo,int flag){
    int giro_volante = 0;
    int velocidade_rpm = 87*direcao; // Velocidade de referência
    sensors_event_t leituraBno;
    bno.getEvent(&leituraBno);
    float anguloRobo=leituraBno.orientation.x ;
    
    while(anguloRobo==0.00 || anguloRobo==360.00){
        sensors_event_t leituraBno;
        bno.getEvent(&leituraBno);
        anguloRobo=leituraBno.orientation.x;
    };
    Serial.println("sai do IF");
    Serial.println(anguloRobo);
    float angulo_final = anguloRobo + angulo;
    int direcaoGiro=0;
    if(angulo_final>0){
        angulo_final>360?angulo_final-=360:angulo_final;
    }
    else{
        angulo_final<0?angulo_final+=360:angulo_final;
    }
    if(angulo>0){
        while(fabs(anguloRobo-angulo_final)>=2){
            Serial.print(angulo_final);
            Serial.print(" ");
            Serial.print(anguloRobo);
            Serial.println(fabs(anguloRobo-angulo_final));
            atualizar_tempo();
            motor_esquerdo.andar_reto(velocidade_rpm+50);
            motor_direito.andar_reto(velocidade_rpm);                                                                     
            sensors_event_t leituraBno;
            bno.getEvent(&leituraBno);
            anguloRobo=leituraBno.orientation.x;
        }
    }
    else{
        while(fabs(anguloRobo-angulo_final)>=2){
            Serial.println(anguloRobo);
            atualizar_tempo();
            motor_esquerdo.andar_reto(velocidade_rpm+50);
            motor_direito.andar_reto(velocidade_rpm);                                                                     
            sensors_event_t leituraBno;
            bno.getEvent(&leituraBno);
            anguloRobo=leituraBno.orientation.x;
        }
    }
    Serial.println("terminei");
    motor_direito.parar();
    motor_esquerdo.parar();
}

// void Robo::virar_robo(Direcao direcao, int angulo){
//     int giro_volante = 0;
//     unsigned long tempo = millis();
//     imu.update();
//     float angulo_final = imu.getAngleZ() + angulo;
//     int velocidade_rpm = 87*direcao; // Velocidade de referência

//     while (imu.getAngleZ() < (angulo_final-3) or imu.getAngleZ() > (angulo_final+3)) {
//         atualizar_tempo();
//         if ((angulo_final - imu.getAngleZ()) > 0) {
//             if ((angulo_final - imu.getAngleZ()) > 10) {
//                 giro_volante = 35;
//             }
//         } else if ((angulo_final - imu.getAngleZ()) < 0) {
//             if ((angulo_final - imu.getAngleZ()) < -10) {
//                 giro_volante = -35;
//             }
//         }
//         if (giro_volante > 0) {
//             if (direcao == tras) {
//                 motor_esquerdo.andar_reto(velocidade_rpm+50);
//                 motor_direito.andar_reto(velocidade_rpm);
//             } else {
//                 motor_esquerdo.andar_reto(velocidade_rpm);
//                 motor_direito.andar_reto(velocidade_rpm - 25);
//             }
//         } else {
//             if (direcao == tras) {
//                 motor_esquerdo.andar_reto(velocidade_rpm);
//                 motor_direito.andar_reto(velocidade_rpm+50);
//             } else {
//                 motor_esquerdo.andar_reto(velocidade_rpm - 25);
//                 motor_direito.andar_reto(velocidade_rpm);
//             }
//         }
//         if (millis() - tempo > 20) {
//             imu.update();
//             tempo = millis();
//         }
//         Serial.print(motor_esquerdo.rps*60);
//         Serial.print(" ");
//         Serial.print(motor_direito.rps*60);
//         Serial.print(" ");
//         Serial.println(imu.getAngleZ());
//     }
    
//     motor_direito.andar_reto(0);
//     motor_esquerdo.andar_reto(0);
// }

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
    sensors_event_t leituraBno;
    bno.getEvent(&leituraBno);
    float angulo_inicial=leituraBno.orientation.x;
    
    while(angulo_inicial==0 || angulo_inicial==360){
        sensors_event_t leituraBno;
        bno.getEvent(&leituraBno);
        angulo_inicial=leituraBno.orientation.x;
    };

    while (Serial.available() > 0) {
        char flush = Serial.read();
    }
    atualizar_tempo();
    cone_posicao_x=0;
    cone_posicao_y=NAOENCONTRADO;
    float posicao_x = 0;
    float giro_volante = 0;
    int velocidade_rpm = 70; // Velocidade de referência
    int contAchouCone = 0;
    unsigned long tempo = millis();
    int erroAtual =0;
    int erroTotal =0;   
    while (retornar_posicao_y_do_cone()>distanciaAteParar){ //! 0.05 é a tolerância, mas pode e deve ser ajustada
        atualizar_tempo();
        retornar_posicao_x_do_cone();
        while(contAchouCone<=5){
            atualizar_tempo();
            andar_reto(velocidade_rpm);
            sensors_event_t leituraBno;
            bno.getEvent(&leituraBno);
            float yaw=leituraBno.orientation.x;
            float ki = 0.9;
            erroAtual = angulo_inicial-yaw;
            erroTotal += erroAtual;
            int giro_volante = (int)(round(erroAtual)*3 +erroTotal*ki*dt);
            if(retornar_posicao_y_do_cone()!=NAOENCONTRADO){
                contAchouCone++;
            }
        }
        retornar_posicao_x_do_cone();
        if(cone_posicao_x>0.05 &&cone_posicao_x<0.1){
            giro_volante=3;
        }
        else if(cone_posicao_x>0.1 &&cone_posicao_x<0.15){
            giro_volante=6;
        }
        else if(cone_posicao_x>0.15 &&cone_posicao_x<0.2){
            giro_volante=9;
        }
        else if(cone_posicao_x>0.2 &&cone_posicao_x<0.3){
            giro_volante=15;
        }
        else if(cone_posicao_x>0.3 ){
            giro_volante=18;
        }
        else if(cone_posicao_x<-0.05&&cone_posicao_x>-0.1){
            giro_volante=-6;
        }
        else if(cone_posicao_x<-0.1 &&cone_posicao_x>-0.15){
            giro_volante=-12;
        }
        else if(cone_posicao_x<-0.15 &&cone_posicao_x>-0.2){
            giro_volante=-16;
        }
        else if(cone_posicao_x<-0.2 &&cone_posicao_x>-0.3){
            giro_volante=-22;
        }
        else if(cone_posicao_x<=-0.3 ){
            giro_volante=-28;
        }
        else if (cone_posicao_x > -0.05 && cone_posicao_x < 0.05) {
            giro_volante = 0;
        }

        if (cone_posicao_x > -0.05 && cone_posicao_x < 0.05) {
            motor_esquerdo.ligar_motor(1, 160);
            motor_direito.ligar_motor(1, 170);
        } else {
            motor_esquerdo.ligar_motor(1, 100);
            motor_direito.ligar_motor(1, 100);
        }
    }

    motor_direito.parar();
    motor_esquerdo.parar();
    delay(2000);
    delay(1000);
}
