#include<ros/ros.h>
#include<std_msgs/String.h>
#include<sstream>
#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>

int main(int argc, char **argv){
	//CRIAÇÃO DO NÓ
	ros::init(argc,argv,"talker");
	cv::Mat a(30,30,CV_8UC1);
	a = cv::Scalar(255);
	cv::imshow("Teste", a);
	cv::waitKey(0);
	ros::NodeHandle nh;
	//CRIO O TOPICO CHAMADO DE CHATTER
	ros::Publisher chatterPub = nh.advertise<std_msgs::String>("chatter",100);
	//continua a rodar até eu dar crtl c
    ros::Rate loop_rate = 10;
	while(nh.ok()){
		std_msgs::String msg;
		std::string string="meu deus que tristeza";
		msg.data=string;
        //o que eu printo quando rodo o rosrun, ou seja, quando eu rodo nó
		// ROS_INFO("TALKER, I PUBLISH %s\n",msg.data.c_str());

		chatterPub.publish(msg);
		ros::spinOnce();
        loop_rate.sleep();

	}
	return 0;

}
