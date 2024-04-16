#include <Arduino.h>
#include<ros.h>
#include<TwoInts.h>
#include<std_msgs/Char.h>
ros::NodeHandle nh;
std_msgs::Char msgChar;
/*
Criacao do tópico chamado tópicoValor que tipado com o msgChar
*/
ros::Publisher pb("topicoValor",&msgChar);
void setup() {
  nh.initNode();
  nh.advertise(pb);
  
}

void loop() {
  msgChar.data=4;
  pb.publish(&msgChar);
  nh.spinOnce();
  delay(2000);

  // put your main code here, to run repeatedly:
}

