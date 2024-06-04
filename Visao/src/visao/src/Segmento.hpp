#pragma once
#ifndef SEGMENTO_H
#define SEGMENTO_H

#include<iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

using namespace cv;
using namespace std;

class Segmento{
	public:
		int ajudae_qual_o_sentido_disso;
		Point inicio, fim, canto, anchor;
		vector<Point> pontos, rotated;
		Point centro;
		int num_pontos;
		Vec3b cor_centro;
		vector<uchar> avg_cor;
		vector<Point2f> esquerda, direita;
		int altura;
		int largura;
		Mat inv_rotacao;
	private:
		// int porque;
	public:
		Segmento();
		void calc_centro();
		void rotate();
		void limites();
		void addPoint(Point);
		void addPoint(Point,vector<uchar>);
		void addVector(vector<Point>);
		void addVector(vector<Point>,vector<uchar>,Point);
		void bordas();
		void clear();
		float ratio();
		float Forma(vector<Point>*);

};


class Cone{
	public:
		float tamanho;
		float qualidade;
		float angulo;
		cv::Point inicio,fim,centro;
		std::vector<cv::Point> cantos;
		float X,Y;

		Cone();
		void Posicao(int,float,int,float);
};


#endif