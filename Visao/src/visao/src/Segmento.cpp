#include"Segmento.hpp"


Segmento::Segmento(){
	num_pontos = 0;
	altura = 0;
	largura = 0;
}
Cone::Cone()
{
	tamanho = 0.5;
}

void Segmento::calc_centro()
{
	if(num_pontos > 0)
	{
		int i;
		int x = 0;
		int y = 0;
		for(i=0;i<num_pontos;i++)
		{
			x+=pontos[i].x;
			y+=pontos[i].y;
		}
		centro.x = x/num_pontos;
		centro.y = y/num_pontos;
	}
}

void Segmento::rotate()
{
	int i;
	rotated.clear();
	Point aux;
	Vec4f matriz(0,0,0,0);

	//inv_rotacao.create(2,2,CV_32FC1);
	
	//cout<<centro<<endl;
	calc_centro();
	//cout<<centro<<endl;

	//centraliza pontos na origem
	for(i = 0; i<num_pontos; i++){
		aux.x = pontos[i].x - centro.x;
		aux.y = pontos[i].y - centro.y;
		rotated.push_back(aux);
		matriz[0] += aux.x*aux.x;
		matriz[1] += aux.x*aux.y;
		matriz[3] += aux.y*aux.y;
	}

	//obtem matriz de rotação
	Mat inercia(2,2,CV_32FC1);
	inercia.at<float>(Point(0,0)) = matriz[0]/num_pontos;
	inercia.at<float>(Point(1,0)) = matriz[1]/num_pontos;
	inercia.at<float>(Point(0,1)) = inercia.at<float>(Point(1,0));
	inercia.at<float>(Point(1,1)) = matriz[3]/num_pontos;
	Mat autovetores(2,2,CV_32FC1);
	Mat autovalores(2,1,CV_32FC1);
	eigen(inercia,autovalores, autovetores);	
	//Mat rotacao = (autovetores.inv());
	float gira_data[4] = {0,-1,1,0}; 
	Mat gira(2,2,CV_32FC1,gira_data);
	Mat rotacao = gira*autovetores.inv();
	inv_rotacao = rotacao.inv();
	inv_rotacao.copyTo(inv_rotacao);


	int max_x = -1;
	int max_y = -1;
	int min_x = 9999999;
	int min_y = 9999999;
	//rotaciona pontos
	int x,y;
	for(i = 0; i<num_pontos; i++){
		x = rotacao.at<float>(Point(0,0))*rotated[i].x + rotacao.at<float>(Point(1,0))*rotated[i].y ;
		y = rotacao.at<float>(Point(0,1))*rotated[i].x + rotacao.at<float>(Point(1,1))*rotated[i].y ;
		rotated[i] = Point(x,y);
		if(rotated[i].x > max_x)
		{
			max_x = rotated[i].x;
		}
		if(rotated[i].x < min_x)
		{
			min_x = rotated[i].x;
		}
		if(rotated[i].y > max_y)
		{
			max_y = rotated[i].y;
		}
		if(rotated[i].y < min_y)
		{
			min_y = rotated[i].y;
		}
	}
	altura = max_y-min_y+1;
	largura = max_x-min_x+1;
	anchor.x = min_x;
	anchor.y = min_y;

	Mat mostra(altura,largura,CV_8UC1);
	mostra = Scalar(0);
	for(i=0;i<num_pontos;i++)
	{
		mostra.at<uchar>(rotated[i].y-min_y,rotated[i].x-min_x) = 255;
	}
	imshow("rotated seg",mostra);
	
}