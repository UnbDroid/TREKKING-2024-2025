#include "RastreiaCone.hpp"
#include "Segmento.hpp"

RastreiaCone::RastreiaCone(){
	delta = 0.2;
	/*
	subidaL = 0;
	step1_L = 0;
	altoL = 90;
	descidaL = 140;
	step2_L = 0;
	baixoL = 256;

	subidaA = 125;
	step1_A = 0;
	altoA = 145;
	descidaA = 160;
	step2_A = 0;
	baixoA = 180;

	subidaB = 140;
	step1_B = 0;
	altoB = 155;
	descidaB = 175;
	step2_B = 0;
	baixoB = 190;
	*/

	/*
	minL = 50;
	maxL = 256;
	minB = 5;
	maxB = 70;
	minG = 40;
	maxG = 90;
	minR = 100;
	maxR = 200;

	minLo = 40;
	maxLo = 200;
	minBo = 20;
	maxBo = 45;
	minGo = 65;
	maxGo = 73;
	minRo = 140;
	maxRo = 165;
	*/

	//copo
	/*
	minL = 50;
	maxL = 256;
	minB = 10;
	maxB = 240;
	minG = 130;
	maxG = 210;
	minR = 130;
	maxR = 205;

	minLo = 40;
	maxLo = 200;
	minBo = 50;
	maxBo = 180;
	minGo = 155;
	maxGo = 180;
	minRo = 155;
	maxRo = 185;
	*/


	//teste
	/*
	minB = 0;
	maxB = 240;
	minG = 100;
	maxG = 165;
	minR = 60;
	maxR = 150;

	minBo = 30;
	maxBo = 150;
	minGo = 135;
	maxGo = 155;
	minRo = 60;
	maxRo = 115;
	*/

	//cone
	/*
	minL = 50;
	maxL = 256;
	minB = 10;
	maxB = 240;
	minG = 130;
	maxG = 175;
	minR = 130;
	maxR = 180;

	minLo = 40;
	maxLo = 200;
	minBo = 50;
	maxBo = 180;
	minGo = 135;
	maxGo = 155;
	minRo = 140;
	maxRo = 165;
	*/
	//coneCasaWebCam
	/*
	minL = 50;
	maxL = 256;
	minB = 10;
	maxB = 240;
	minG = 100;
	maxG = 200;
	minR = 130;
	maxR = 185;

	minLo = 40;
	maxLo = 200;
	minBo = 50;
	maxBo = 180;
	minGo = 155;
	maxGo = 175;
	minRo = 145;
	maxRo = 180;
	*/

	
	minB = 0;
	maxB = 240;
	minG = 135;//140;
	maxG = 200;
	minR = 130;//140;
	maxR = 200;


	minBo = 20;
	maxBo = 140;
	minGo = 145;//150;
	maxGo = 190;
	minRo = 140;//160;
	maxRo = 200;




	difLo = 10;
	difLb = 20;
	difAo = 3;
	difAb = 8;
	difBo = 3;
	difBb = 10;

	max_dif = 6;

	sizeCanny = 0;

	minCanny = 30;
	maxCanny = 60;


	aceita_final = 140;//160;
	min_aceita = 200;

	//dif_pixel = 15;
	//dif_centroide = 140;
	//max_dif_pixel = 15;
	//max_dif_centroide = 140;

	//max_dif_pixelL = 8;
	//max_dif_centroideL = 80;

	angulo_ideal = 20.10;



	min_segment_size = 50;

	//PreencheTabela();

	atualiza = true;

	tamanho0 = 240;

	linha_centro = tamanho0/2;

	max_search_area =16000;

	PreencheTabela();


	filtro = false;
	#ifdef filmar
	video_init = false;
	#endif
	
}
void RastreiaCone::PreencheTabela(){

	//cout<<"atualizando"<<endl;

	int i, j, k;
	float incremento;
	float valor;

	//Preenche tabela de cor

	//Filtro da Luminancia
	for(i = 0; i<minB; i++){
		L[i] = 0;
	}
	//incremento = (1.0- (((float)aceita_final)/255) )/(float)(minBo - minB);
	//valor = aceita_final;
	incremento = (1.0 )/(float)(minBo - minB);
	valor = 0;
	
	for( i = minB; i<minBo; i++){
		valor += incremento;
		L[i] = valor;
	}
	for(i = minBo ; i<maxBo; i++){
		L[i] = 1;
	}
	valor = 1;
	//incremento = (1.0-(((float)aceita_final)/255) )/(float)(maxBo - maxB);
	incremento = (1.0 )/(float)(maxBo - maxB);
	for(i = maxBo; i<maxB; i++){
		valor += incremento;
		L[i] = valor;
	}
	for(i = maxB; i<256; i++){
		L[i] = 0;
	}
	
	//Filtro da Saturação
	for(i = 0; i<minG; i++){
		A[i] = 0;
	}
	//incremento = (1.0-(((float)aceita_final)/255) )/(float)(minGo - minG);
	//valor = aceita_final;
	incremento = (1.0 )/(float)(minGo - minG);
	valor = 0;
	
	for( i = minG; i<minGo; i++){
		valor += incremento;
		A[i] = valor;
	}
	for(i = minGo ; i<maxGo; i++){
		A[i] = 1;
	}
	valor = 1;
	//incremento = (1.0 -(((float)aceita_final)/255) )/(float)(maxGo - maxG);
	incremento = (1.0 )/(float)(maxGo - maxG);
	for(i = maxGo; i<maxG; i++){
		valor += incremento;
		A[i] = valor;
	}
	for(i = maxG; i<256; i++){
		A[i] = 0;
	}
	

	//Filtro do Value
	for(i = 0; i<minR; i++){
		B[i] = 0;
	}
	//incremento = (1.0 -(((float)aceita_final)/255) )/(float)(minRo - minR);
	//valor = aceita_final;
	incremento = (1.0  )/(float)(minRo - minR);
	valor = 0;
	
	for( i = minR; i<minRo; i++){
		valor += incremento;
		B[i] = valor;
	}
	for(i = minRo ; i<maxRo; i++){
		B[i] = 1;
	}
	valor = 1;
	//incremento = (1.0 -(((float)aceita_final)/255) )/(float)(maxRo - maxR);
	incremento = (1.0 )/(float)(maxRo - maxR);
	if(baixoB > 256)
		baixoB = 256;
	for(i = maxRo; i<maxR; i++){
		valor += incremento;
		B[i] = valor;
	}
	for(i = maxR; i<256; i++){
		B[i] = 0;
	}

	
	for(k = 0; k<256; k++){
		for(i = 0; i<256; i++){
			for(j = 0; j<256; j++){
				Tabela[i][j][k] = (uchar) 255*L[i]*A[j]*B[k];
			}
		}
	}

	for(i=0;i<50;i++)
	{
		for(j=0;j<50;j++)
		{
			for(k=0;k<50;k++)
			{
				difTable[i][j][k] = sqrt(i*i+j*j+k*k);
			}
		}
	}


	//max_fdif = (float)max_dif/100;
	max_fdif = (float)max_dif;
}

