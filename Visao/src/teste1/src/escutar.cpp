#include<ros/ros.h>
#include<std_msgs/String.h>
#include<iostream>

//essa função sempre deve ser void e receber um const com um contPtr que é a pŕopria msg
void funcaoDeEscutaTodaVezQueAMsgVimPeloTopico(const std_msgs::StringConstPtr& msg){

    std::cout<<"Valor recebido no nó orelha pelo tópico chatter: "<<msg->data<<std::endl;

}
int main(int argc, char ** argv){
    //criação do nó, chamamos ele de orelha, bem aqui que será o nome do nó quando rodamos um rosnode list
    ros::init(argc,argv,"orelha");
    //criamos o carteiro
    ros::NodeHandle nh;
    ros::Subscriber subscriber= nh.subscribe("chatter",100, funcaoDeEscutaTodaVezQueAMsgVimPeloTopico);
    ros::spin();
    return 0;
}