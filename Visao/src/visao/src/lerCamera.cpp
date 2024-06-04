#include<iostream>

#include<opencv2/highgui/highgui.hpp>
#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include"Segmento.hpp"
#include"RastreiaCone.hpp"
int tecla = 0;
int comando=0;
Point inicio, fim;
int pontos;
int quadro;
//variaveis dos paineis
bool p1 = false;
bool p2 = false;
bool p3 = false;
bool p4 = false;
bool p5 = false;
bool p6 = false;
bool p7 = false;

#define usuario
RastreiaCone cone;
void sonda(Mat img);
void nothing(){
    return ;
}
int lH =100;
int lS =100;
int lV =100;
int uH=255;
int uS=255;
int uV=255;
void painelHsv(){
    namedWindow("PAINEL 2", WINDOW_NORMAL);
    createTrackbar( "L-H", "PAINEL 2", &lH, 179);
	createTrackbar( "L-S", "PAINEL 2", &lS, 255);
	createTrackbar( "L-V", "PAINEL 2", &lV, 255);
	createTrackbar( "U-H", "PAINEL 2", &uH, 179);
	createTrackbar( "U-S", "PAINEL 2", &uS, 255);
	createTrackbar( "U-V", "PAINEL 2", &uV, 255);
}

int Comando(Mat source, VideoCapture* cap, bool *aberta){
    // std::cout<<comando   <<"  ";
    // if((comando ==0 || comando==-1) && *aberta){
    comando =waitKey(1);
    // }
    // else{
        switch (comando){
        case -1:
            sonda(source);
            break;
        case 'p':
            std::cout<<"to aqui";
            // sonda(source);
            break;
        case 'q':
            //para fechar tudo
            cv::destroyAllWindows();
            // std::cout<<"to no Q"<<std::endl;
            break;
        case 'e':
            std::cout<<*aberta<<std::endl;
            *aberta= !(*aberta);
            break;
        }
    // }
    // std::cout<<comando<<std::endl;
    
    // std::cout<<"fui retornado"<<std::endl;
    return 0;
}
cv::Mat imgCamera;

void normaliza(Mat source, Mat* dest){
	float B,G,R,Sum, value, Bn, Gn, Rn;
	for(int i= 0; i<source.cols; i++){
		for(int j = 0; j<source.rows; j++){
			B = ((float)source.at<Vec3b>(Point(i, j))[0]);
			G = ((float)source.at<Vec3b>(Point(i, j))[1]);
			R = ((float)source.at<Vec3b>(Point(i, j))[2]);
			Sum = B+G+R;
			value = max(max(B,G), R);
			Bn = (255.0*B)/Sum;
			Gn = (255.0*G)/Sum;
			Rn = (255.0*R)/Sum;
			(*dest).at<Vec4b>(Point(i, j))[0] = (int)Bn;
			(*dest).at<Vec4b>(Point(i, j))[1] = (int)Gn;
			(*dest).at<Vec4b>(Point(i, j))[2] = (int)Rn;
			(*dest).at<Vec4b>(Point(i, j))[3] = (int)value;
		}
	}
}
Mat mask;
bool convex_hull_pointing_up(const std::vector<cv::Point>& ch) {
    // Obtém o retângulo limitador para o conjunto de pontos
    cv::Rect boundingBox = cv::boundingRect(ch);
    double aspect_ratio = static_cast<double>(boundingBox.width) / boundingBox.height;

    // Se o retângulo não é estreito, retorna falso
    if (aspect_ratio >= 0.8) {
        return false;
    }

    std::vector<cv::Point> points_above_center;
    std::vector<cv::Point> points_below_center;
    double vertical_center = boundingBox.y + boundingBox.height / 2.0;

    // Classifica cada ponto como acima ou abaixo do centro vertical
    for (const auto& point : ch) {
        if (point.y < vertical_center) {
            points_above_center.push_back(point);
        } else {
            points_below_center.push_back(point);
        }
    }

    if (points_below_center.empty()) {
        return false; // Evitar divisão por zero no próximo passo
    }

    // Determina os pontos extremos esquerdo e direito abaixo do centro
    int left_x = points_below_center[0].x;
    int right_x = points_below_center[0].x;

    for (const auto& point : points_below_center) {
        if (point.x < left_x) {
            left_x = point.x;
        }
        if (point.x > right_x) {
            right_x = point.x;
        }
    }

    // Verifica se algum ponto acima do centro está fora do "base"
    for (const auto& point : points_above_center) {
        if (point.x < left_x || point.x > right_x) {
            return false;
        }
    }

    return true;
}
void sonda(Mat img){
	Mat temp;
	Mat hsv;
	Mat gray;
	Mat intensity(img.size(), CV_8UC1);
	Mat norm(img.size(), CV_8UC4);
	// normaliza(img, &norm);
    // cv::imshow("Imagem Normalizada",norm);
    waitKey(1);
	//temp = img.clone();
	float ratio_original = (float)img.cols/(float)img.rows;
	resize(img, temp, Size((int)(((float)cone.tamanho0)*ratio_original), cone.tamanho0 ));
    //imagem temporaria é so a imgNormal porem ajustada o tamanho   
    // cv::imshow("Imagem temporaria",temp);
    
	cvtColor(temp, hsv, CV_BGR2HSV);
    lH =getTrackbarPos( "L-H", "PAINEL 2");
    lS =getTrackbarPos( "L-S", "PAINEL 2");
    lV =getTrackbarPos( "L-V", "PAINEL 2");
    uH =getTrackbarPos( "U-H", "PAINEL 2");
    uS =getTrackbarPos( "U-S", "PAINEL 2");
    uV =getTrackbarPos( "U-V", "PAINEL 2");
    Mat trackBar;
    inRange(hsv,Scalar(lH,lS,lV),Scalar(uH,uS,uV),trackBar);
    
    inRange(hsv,Scalar(0,135,135),Scalar(15,255,255),mask);
    Mat mask2;
    inRange(hsv,Scalar(159,135,80),Scalar(179,255,255),mask2);
    Mat final;
    bitwise_or(mask,mask2,final);
    //operacao de abertura para retirar pequenos ruídos
    morphologyEx(final,final,MORPH_OPEN,(5,5),Point(-1,-1),3);
    medianBlur(final,final,5);
    
    std::vector<std::vector<Point2d>> edges;
    Mat a;
    //80 160
    Canny(final,a,80,160,3);

    std::vector<std::vector<Point>> countors;
    findContours(a,countors,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
    Mat imgCon(a.size(),CV_8UC3,Scalar(0,0,0));
    Mat imgContornos;
    drawContours(imgCon,countors,-1,Scalar(255,255,0),1);
    std::vector<std::vector<Point>> countorsPorPoligonos = countors;
    // std::cout<<countors.size();
    cv::namedWindow("Inicio", cv::WINDOW_GUI_EXPANDED);
    cv::imshow("Inicio", imgCon);
    waitKey(1);
    for(int i=0;i<(int)countors.size();i++){
        approxPolyDP(countors[i],countorsPorPoligonos[i],4,1);
    }

    Mat imgPoligonos(a.size(),CV_8UC3,Scalar(0,0,0));
    Mat imgConvex(a.size(),CV_8UC3,Scalar(0,0,0));
    
    drawContours(imgPoligonos,countorsPorPoligonos,-1,Scalar(0,255,0),1);

    std::vector<std::vector<Point>> convex(countorsPorPoligonos.size());
    for (int i =0;i<(int)countorsPorPoligonos.size();i++){
        convexHull(countorsPorPoligonos[i],convex[i]);
    }

    drawContours(imgConvex,convex,-1,Scalar(0,150,150),1);
    
    std::vector<std::vector<Point>> convex2;
    std::vector<std::vector<Point>>instancia(convex.size());
       for(int i=0;i<(int)convex.size();i++){
        // std::cout<<convex[i];
        if(3<=(int)convex[i].size() && (int)convex[i].size()<=10){
            convexHull(Mat(convex[i]),instancia[i]);
            convex2.push_back(instancia[i]);
        }
    }
    Mat imgConve2(a.size(),CV_8UC3,Scalar(0,0,0));


    std::vector<std::vector<cv::Point>> cones;
    std::vector<cv::Rect> bounding_rects;

    // Check each convex hull if it points up
    for (const auto& ch : convex2) {
        if (convex_hull_pointing_up(ch)) {
            cones.push_back(ch);
            cv::Rect rect = cv::boundingRect(ch);
            bounding_rects.push_back(rect);
        }
    }

    // std::cout<<"  Convex2: "<<(int)convex2.size()<<std::endl;
    // if(convex2.size()==0){
    //     std::cout<<"rapaz";
        drawContours(temp,cones,-1,Scalar(255,255,255),1);

     for (const auto& rect : bounding_rects) {
        cv::rectangle(temp, rect, cv::Scalar(0, 255, 0), 2);
    }

    // Display the image using OpenCV's built-in functionality
    cv::namedWindow("Detected Cones", cv::WINDOW_GUI_EXPANDED);
    cv::imshow("Detected Cones", temp);
    waitKey(1);
    // }
    
	// imshow("ORIGINAL",temp );
    // waitKey(1);
    // // imshow("TESTE COM BITWISE", final);
    // // waitKey(1);
    // imshow("CONTORNOS", imgCon);
    // waitKey(1);
    imshow("CONTORNOS POR POLIGONOS", imgPoligonos);
    waitKey(1);
    // imshow("CONTORNOS CONVEXHULL", imgConvex);
    // waitKey(1);
    

    // imshow("PAINEL 2", trackBar);
    // waitKey(1);
    // return ;
    // comando = waitKey(1);
    // if(comando<0){
    //     return ;
    // }
}
bool aberta = true;
int main(int argc, char ** argv){
    ros::init(argc,argv,"noLer");
    // Segmento a;
    // painelHsv();
    ros::Time::init();
    int videoSource = 2;
	cv::VideoCapture cap(videoSource,cv::CAP_V4L);
    // RastreiaCone RastreiaCone();
	if(!cap.isOpened()) return 1;	
	cv::Mat frame;
    ros::Rate rate(30);
    
	while(ros::ok()){
		cap>>imgCamera;
		cv::resize(imgCamera,frame, cv::Size(350,350), 0, 0);
        // cv::imshow("AAAAAAA", frame);
        #ifdef usuario
        // sonda(imgCamera);
        Comando(frame,&cap,&aberta);
        
        #endif
        // std::cout<<"AQQQQQQQQQQQQQQQQQQ"<<std::endl;
		ros::spinOnce();
        rate.sleep();
        
	}


    
    return 0;
}
