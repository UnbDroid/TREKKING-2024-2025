#include "Arduino.h"
#include "mpu9250.h"
#include "Wire.h"

// Declarações do motor -----------------------------------------------------------------------------------------------------------------------------------------------------------

#define ENCA 3 // YELLOW
#define ENCB 8 // WHITE
#define PWM 5
#define IN1 6
#define IN2 7

volatile double posi = 0; // posição real do encoder do motor, usado no cálculo da posição que utilizaremos de fato no código (posi_atual)

double voltas = 0; // número de voltas do motor
double voltas_anterior = 0; // número de voltas do motor no instante anterior, para cálculo do erro
double rps = 0; // velocidade ATUAL do motor em radianos por segundo
int rpm_referencia = 80; // velocidade desejada do motor, velocidade que ele buscará alcançar
double rps_max = 5.75; // velocidade máxima do motor (apenas por curiosidade, usar caso seja necessário)

double comprimento_roda = 6*2*3.1415; // Usado para cálculo dos valores em cm (velocidade e espaço percorrido)

long prevT = 0; // tempo anterior para calcular a velocidade do motor
float eprev = 0;
float eintegral = 0; // ki do controle PID

int dir = 1; // 1 para frente, -1 para trás (pelo menos essa é a ideia)

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Declarações do MPU9250 ---------------------------------------------------------------------------------------------------------------------------------------------------------

bfs::Mpu9250 imu; // Criação do objeto imu da classe Mpu9250
int status; // Status do sensor

const double deg_to_rad = 3.14159265358979; // Fator de conversão de graus para radianos (caso precisemos)

// Parâmetros do filtro (constantes de controle dos valores de movimento dos eixos)
const float alpha_x = 0.45;
const float alpha_y = 0.45;
const float alpha_z = 0.45;

// Ângulos de roll(x), pitch(y) e yaw(z) em graus
float roll_angle = 0.0;
float pitch_angle = 0.0;
float yaw_angle = 0.0;

// Variáveis temporárias para armazenamento dos dados do sensor
float prev_gyro_z_rad = 0.0; // Valor anterior do giroscópio em radianos
float gyro_x_rad, gyro_y_rad, gyro_z_rad; // Valores atuais do giroscópio em radianos
float accel_x_g, accel_y_g, accel_z_g; // Valores atuais do acelerômetro em g (aceleração da gravidade)

// Funções adicionais do código -----------------------------------------------------------------------------------------------------------------------------------------------------

// Função para controle da direção e velocidade do motor

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal); // (pino do pwm, valor do pwm (máximo = 255))
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}


// Função para leitura do encoder
void readEncoder(){
  double b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}

// Funções principais do código ----------------------------------------------------------------------------------------------------------------------------------------------------

void setup() {

  // Início da comunicação serial
  Serial.begin(9600);

  // Declaração de pinos dos motores {
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    pinMode(PWM, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  //}

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

}

void loop() {
  imu.Read(); // Lê os dados do sensor imu

  int encoder_volta = 1044; // valor de encoder referente a uma volta completa da roda

  // Constantes do PID
  float kp = 5.0;
  float ki = 3.0;
  float kd = 0.0;
  // Todos sujeitos a mudanças conforme necessário

  // Cálculo do tempo decorrido
  long T = millis(); // tempo atual em milissegundos
  float dt = (T - prevT)/1000.0; // tempo decorrido em segundos em relação a última medição
  prevT = T; // atualiza o tempo anterior

  // Definir valores do giroscópio em radianos
  gyro_x_rad = imu.gyro_x_radps();
  gyro_y_rad = imu.gyro_y_radps();
  gyro_z_rad = imu.gyro_z_radps();

  // Definir valores do acelerômetro em g
  accel_x_g = imu.accel_x_mps2() / 9.81;
  accel_y_g = imu.accel_y_mps2() / 9.81;
  accel_z_g = imu.accel_z_mps2() / 9.81;

  // Cálculo do ângulo de roll e pitch usando o acelerômetro (estimativa inicial)
  roll_angle = atan2(accel_y_g, accel_z_g) * deg_to_rad;
  pitch_angle = atan2(accel_x_g, accel_z_g) * deg_to_rad;

  // Filtro de Kalman para o ângulo de roll, pitch e yaw (x, y e z respectivamente)
  roll_angle = alpha_x * (roll_angle + gyro_x_rad * dt) + (1 - alpha_x) * roll_angle;
  pitch_angle = alpha_y * (pitch_angle + gyro_y_rad * dt) + (1 - alpha_y) * pitch_angle;
  if ((gyro_z_rad * dt) > 0.001 or (gyro_z_rad * dt) < (-0.001)) {yaw_angle += gyro_z_rad * dt;}
  delay(10); // delay para evitar problemas de leitura do encoder e do MPU9250

  // Leitura do encoder
  double posi_atual = 0; // posição atual do encoder
  noInterrupts(); // desabilita interrupções (não sei exatamente o porquê de usar isso, mas sei que funciona assim ent neh galerinha kkkkk)
  posi_atual = posi; // atualiza a posição atual do encoder
  interrupts(); // reabilita interrupções (mesma coisa aqui rapazeadinha)

  voltas_anterior = voltas; // atualiza o número de voltas anterior

  // Cálculo do número de voltas e velocidade em rps do motor
  voltas = posi_atual/encoder_volta; // calcula o número de voltas do motor
  rps = (voltas - voltas_anterior)/dt; // calcula a velocidade do motor em rps com base nos valores estabelecidos antes

  // Cálculo do erro
  double e = rpm_referencia - (rps * 60); // calcula o erro da velocidade em rpm

  // Cálculo do controle PID {

    // Proporcional
    float p = kp * e;

    // Integral
    eintegral += e;

    // Derivativo
    float d = kd * (e - eprev)/dt;

  //}

  // Sinal de controle (valor do pwm que será enviado ao motor)
  float u = p + (ki * eintegral)+ d;

  // Controle do motor
  float pwmVal = fabs(u); // valor do pwm que será enviado ao motor
  if(pwmVal > 255) {
    pwmVal = 255;
  }

  // Direção do motor
  if(u > 0){
    dir = 1;
  }
  else if(u < 0){
    dir = -1;
  }
  else{
    dir = 0;
  }

  // Envio do sinal de controle ao motor
  setMotor(dir, pwmVal, PWM, IN1, IN2);

}