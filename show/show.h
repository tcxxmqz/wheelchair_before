#ifndef _SHOW_H_
#define _SHOW_H_
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>  
#include <Windows.h>
using namespace std;
//using namespace cv;

#define WINDOW_NAME1 "������ͼ1��"        //Ϊ���ڱ��ⶨ��ĺ� 
#define WINDOW_NAME2 "������ͼ2��"        //Ϊ���ڱ��ⶨ��ĺ� 
#define WINDIV   150      // ���ڱ�ԵԤ��   
#define WINDOW_WIDTH 600 //���崰�ڴ�С�ĺ�
#define DETECT_MAX_DiS 2000 //������������
#define X_SCALE_RATIO (WINDOW_WIDTH)/(2*DETECT_MAX_DiS)
#define Y_SCALE_RATIO (WINDOW_WIDTH - 150 )/2500   // 500�Ǽ����500mm����
#define MAX_SPEED 0.3 // ����������ٶ� ��m/s��
class Show
{
	public:
		void DrawEllipse(Mat img, double angle, float x, float y, int a, int b);//������Բ
		void DrawFilledCircle(Mat img, Point center);//����Բ
		void DrawPolygon(Mat img);//���ƶ����
		void DrawLine(Mat img, Point start, Point end);//�����߶�
		void DisplayFoot(vector<float> foot, float *footgoal, float *newfootgoal, int goalangle, int footangle, float speed);

};

#endif