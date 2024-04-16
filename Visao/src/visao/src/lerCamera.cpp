#include<iostream>

#include<opencv2/highgui/highgui.hpp>
#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
int tecla = 0;
void receberImagemCam(const sensor_msgs::ImageConstPtr &msg){
    try{
        ROS_INFO("ANTES DO IMSHOW");
        //32 é o espaço
        
        cv::imshow("Imagem",cv_bridge::toCvShare(msg,"bgr8")->image);
        cv::waitKey(1);       
        ROS_INFO("DEPOIS DO WAITKEY %d",tecla);

    }
    catch(cv_bridge::Exception &e){
        ROS_ERROR("Erro ao ler o topico");
    }
}
int main(int argc, char ** argv){
    ros::init(argc,argv,"noLer");
    ros::NodeHandle nh;
    ROS_INFO("TO AQUI NO INICIO DA MAIN");
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub= it.subscribe("cameraAoVivo",1,receberImagemCam);
    ros::spin();
    ROS_INFO("ANTES DO DESTROYALLWINDOWS");
    cv::destroyAllWindows();
    ROS_INFO("DEPOIS");
    return 0;
}