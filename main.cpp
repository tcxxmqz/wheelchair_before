#include "stdafx.h"

#include <strsafe.h>
#include "resource.h"
#include <iostream>
#include <fstream>
#include <windows.h>
#include <iomanip>
#include <omp.h>
#include "stdlib.h"
#pragma comment(lib, "Winmm.lib") 
#include <mmsystem.h>

#include "Usart.h"
#include "Robot_Process.h"
#include "Laser.h"
#include <vector>
#include <math.h> 

#include <thread>
#include <condition_variable>
#include <mutex>

#include<winsock.h>
#include<stdio.h>
 
#include <functional> 
#include <Eigen/Dense> 
#include <map>
using namespace Eigen;
using namespace std;

#define NUM_THREADS 2


/************************************************************************************
					Description:��ʱ�� 1 �ص�����
					Author: Chunwei Yu
					Date:2017-07-16
***************************************************************************************/

//float d_D[30][2] = {0}; // ����������ӳ������ ��һ�� ���� �ڶ��� ����
char Start_Stop = 0;
extern BOOL S_T;

void cutStr(int start1, int end1, int start2, int end2, unsigned char* scrStr, unsigned char* resulStr1, unsigned char* resulStr2)
{
	if (scrStr[0] == '2' && scrStr[1] == '_' && scrStr[2] == 'S')//֡ͷ
	{
		for (int i = start1; i < end1; i++)
		{
			resulStr1[i] = scrStr[i];
			//cout << scrStr[i] << endl;
		}
		int j = 0;
		for (int i = start2; i < end2; i++)
		{
			resulStr2[j] = scrStr[i];
			j++;
			//cout << resulStr2[j];
			if (j == 42) break;
		}
		//cout << endl;
	}
}
// �����Ƿ����ݵ������ļ�
#define FILE_OUT true
unsigned char STOP_FLAG = 0x01; // ֹͣ��־λ
/* 0x01 : ��������
   0x02 : ��������
   0x04 : ���ɼ�һ����������
   0x08 : ֹͣ״̬
   0x10 : Ҫ�ɼ����һ������ */
void CALLBACK onTimeFunc(HWND hWnd, UINT uMsg, UINT idEvent, DWORD dwTime)
{
	vector<int> PressureData;

	static int count = 0;  //��ʱ���������
	unsigned char temStr[80] = { 0 };

	unsigned char pressStr[43] = { 0 };
	pressStr[0] = '4';
	pressStr[1] = '_';
	pressStr[2] = 'L';
	pressStr[3] = 'S';
	pressStr[4] = ':';
	//pressStr[34] = '\r';
	//pressStr[35] = '\n';
	float speed[4] = { 0 }, PressAve = 0;
	unsigned char speed_Order[21] = { 0 };

	//		float sa[2];//sa[1] = ģ�ͼ�����ٶ� sa[2] = ģ�ͼ���ĽǶ�

	while (!Usart.Read_Data(pressStr, 42, hCOM5));//��ȡ��ѹ��+�������ַ�����Ϊ78  ѹ��Ϊ:42

	PressureData = Usart.getPressData(pressStr, FILE_OUT);  //�õ�ѹ������ Ϊһ������

	ofstream SaveFile("RF.txt");
	//PressureData = { 1,2,3,4,5,6 };

	SaveFile << PressureData[0];
	SaveFile.close(); 

}

/************************************************************************************
					Description:��ʱ�� 2 �ص����� �����Ӷ�
					Author: test
***************************************************************************************/
 
void CALLBACK onTimeFunc2(HWND hWnd, UINT uMsg, UINT idEvent, DWORD dwTime) {
	 
	 
	vector<vector<int>> laserData = URG04LX.getLaserDownData(true);

	 

	int leftCount = 0;
	for (int j = 214; j < 340; j++)
	{
		if (laserData[j][1] < 800) {
			leftCount++;
		}
	}

	int rightCount = 0;
	for (int j = 341; j < 467; j++)
	{
		if (laserData[j][1] < 600) {
			rightCount++;
		}
	}

	 
	if (leftCount > 80)
	{
		float speed[4] = { 0 };
	    unsigned char data_order[21] = { 0 };
	    Robot_Toilet.Move_Equation(0, 0, -10, speed);
	    Robot_Toilet.Data_Cov(speed, data_order);
	    Usart.WriteData(data_order, 21, hCOM5);
	} else if (rightCount > 80)
	{
		float speed[4] = { 0 };
		unsigned char data_order[21] = { 0 };
		Robot_Toilet.Move_Equation(0, 0, 10, speed);
		Robot_Toilet.Data_Cov(speed, data_order);
		Usart.WriteData(data_order, 21, hCOM5);
	} 
	else {
		Robot_Toilet.Key_Control_Robot(0.05);
	}
 
	 
	//float speed[4] = { 0 };
	//unsigned char data_order[21] = { 0 };
	//Robot_Toilet.Move_Equation(0.05, 90, 0, speed);
	//Robot_Toilet.Data_Cov(speed, data_order);
	//Usart.WriteData(data_order, 21, hCOM5);
}

/************************************************************************************
					Description:��ʱ�� 3 �ص����� ���⴫����ʶ����ȣ��Ҿ��ȱ���
					Author: test
***************************************************************************************/
extern float speedTemp[4] = { -20,-20,20,20 }; 
void CALLBACK onTimeFunc3(HWND hWnd, UINT uMsg, UINT idEvent, DWORD dwTime) {  

	vector<vector<int>> laserData=URG04LX.getLaserDownData(true);
	int pointCount = 0;
	for (int j = 300; j < 385; j++)
	{
		if (laserData[j][1] < 600) {
			pointCount++;
		}
	}
		 
	cout << pointCount << " "; 
	if (pointCount > 80) {
		unsigned char speed_Order[21] = { 0 };
		float speedStartTimer3[4] = { -30,-30,30,30 };
		if (speedTemp[0] != -30) {
			speedTemp[0] = speedTemp[0] - 2;
		}
		if (speedTemp[1] != -30) {
			speedTemp[1] = speedTemp[1] - 2;
		}
		if (speedTemp[2] != 30) {
			speedTemp[2] = speedTemp[2] + 2;
		}
		if (speedTemp[3] != 30) {
			speedTemp[3] = speedTemp[3] + 2;
		}
		Robot_Toilet.Data_Cov(speedTemp, speed_Order);
		Usart.WriteData(speed_Order, 21, hCOM5);
	}
	else {
		unsigned char speed_Order[21] = { 0 };
		float speedStartTimer3[4] = { -10,-10,10,10 };
		if (speedTemp[0] != -10) {
			speedTemp[0] = speedTemp[0] + 2;
		}
		if (speedTemp[1] != -10) {
			speedTemp[1] = speedTemp[1] + 2;
		}
		if (speedTemp[2] != 10) {
			speedTemp[2] = speedTemp[2] - 2;
		}
		if (speedTemp[3] != 10) {
			speedTemp[3] = speedTemp[3] - 2;
		}
		Robot_Toilet.Data_Cov(speedTemp, speed_Order);
		Usart.WriteData(speed_Order, 21, hCOM5);
	}

	//for (int i = 0; i < laserData.size(); i++)
	//{
	//	
	//	for (int j = 0; j < laserData[i].size(); j++)
	//	{
	//		cout << laserData[i][j] << " ";
	//	}
	//	cout << "\n";
	//}
	//�����ά��̬���� 
}
/************************************************************************************
					Description:��ʱ�� 4 PID���Ƽ�����
					Author: test
***************************************************************************************/ 
extern MatrixXd V3 = MatrixXd::Zero(3, 1);//���巴��������x�ᣬy�ᣬ��ת���ٶ�
void CALLBACK onTimeFunc4(HWND hWnd, UINT uMsg, UINT idEvent, DWORD dwTime)
{
 
	float theta = 0;
	float alafa = 0;
	////�´���ٶ�ָ��
	float speed[4];
	
	////Ŀ���ٶ�
	//float objVx =0;
	//float objVy= 10;
	//float objVtheta =0;

	// 
	//MatrixXd F(4, 1);
	//F(0, 0) = Robot_Toilet.PID_controler(objVx, objVy, objVtheta, V3)(0,0);
	//F(1, 0) = Robot_Toilet.PID_controler(objVx, objVy, objVtheta, V3)(1, 0);
	//F(2,0) = Robot_Toilet.PID_controler(objVx, objVy, objVtheta, V3)(2, 0);
	//F(3, 0) = Robot_Toilet.PID_controler(objVx, objVy, objVtheta, V3)(3, 0);

	//theta = Robot_Toilet.PID_controler(objVx, objVy, objVtheta, V3)(0, 1);
	//float deltaVx= Robot_Toilet.PID_controler(objVx, objVy, objVtheta, V3)(1, 1);
	//float deltaVy = Robot_Toilet.PID_controler(objVx, objVy, objVtheta, V3)(2, 1);
	//float deltaVtheta = Robot_Toilet.PID_controler(objVx, objVy, objVtheta, V3)(3, 1);

	//float power[4];
	//power[0] = F(0, 0);
	//power[1] = F(1, 0);
	//power[2] = F(2, 0);
	//power[3] = F(3, 0);
	//V3(0,0)= Robot_Toilet.Power_Cov_Speed(power, theta, alafa)(0,1);
	//V3(1, 0) = Robot_Toilet.Power_Cov_Speed(power, theta, alafa)(1, 1);
	//V3(2, 0) = Robot_Toilet.Power_Cov_Speed(power, theta, alafa)(2, 1);

	//-++-����� +--+��ǰ�� ���ĸ���������
	float power1[4];
	power1[0] = -0.1;  //f2
	power1[1] = 0;   //f1
	power1[2] =0;    //f4
	power1[3] = 0;  //f3
	
	speed[0]= Robot_Toilet.Power_Cov_Speed(power1, theta, alafa)(0, 0);
	speed[1] = Robot_Toilet.Power_Cov_Speed(power1, theta, alafa)(1, 0);
	speed[2] = Robot_Toilet.Power_Cov_Speed(power1, theta, alafa)(2, 0);
	speed[3] = Robot_Toilet.Power_Cov_Speed(power1, theta, alafa)(3, 0);

	//ֱ�Ӹ����ٶ�
	//speed[0] = 0;//FL 3
	//speed[1] = 0;//FR 6
	//
	//speed[2] = 0;//BR 12
	//speed[3] = 5;//BL 9
	
	cout.setf(ios::fixed, ios::floatfield);//ʮ���Ƽ����������ǿ�ѧ������
	cout.precision(3);//����2λС�� 
	cout << "theta :" << endl;
	cout << theta << endl;
	cout << "deltaVx :" << endl;

	 
	
	cout << "speed :" << endl;
	cout << "speed1 :" << speed[0] << "   " << "speed2 :" << speed[1] << "   " << "speed3 :" << "   " << speed[2] << "   " <<"speed4 :" << "   " << speed[3] << endl;

	unsigned char data_order[21] = { 0x01,0x30,0x30,0x30,0x31,0x30,0x35,0x30,0x30,0x30 ,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x31,0x0d,0x0a };
    Robot_Toilet.Data_Cov(speed, data_order); 
	Usart.WriteData(data_order, 21, hCOM5);
}
/************************************************************************************
					Description:�߳�һ �����Ӷ�
					Author: test
***************************************************************************************/
int ThreadProc1()
{ 

	MMRESULT timer_id2;
	CloseHandle(hCOM5);        
	hCOM5 = Usart.Open_Com("COM3", hCOM5);
	Usart.Get_Com_State(115200, hCOM5, DCB_COM5); 
	 
	return 0;
}

int ThreadProc2()
{ 
	MMRESULT timer_id3;
	URG04LX.Laser_Init(1);
	timer_id3 = timeSetEvent(100, 1, (LPTIMECALLBACK)onTimeFunc2, DWORD(1), TIME_PERIODIC);
	//�ϱ����ж�ȡ���⴫����

	 
	return 0;
}
/***************************************************************************************
									Description:������
									Author: Chunwei Yu
									Date:2017-07-16
***************************************************************************************/
int main(void)
{
	using namespace std;
	MMRESULT timer_id;  

	CloseHandle(hCOM5);
	hCOM5 = Usart.Open_Com("COM3", hCOM5);
	Usart.Get_Com_State(115200, hCOM5, DCB_COM5);
    //timer_id = timeSetEvent(100, 1, (LPTIMECALLBACK)onTimeFunc, DWORD(1), TIME_PERIODIC);
	// �������ж�ȡѹ�����������ݣ���ʱ��ע�ͷſ�
	
	
   /* thread t1(threadproc1);
    thread t2(threadproc2);
	t1.join();
	t2.join();*/

	timer_id = timeSetEvent(100, 1, (LPTIMECALLBACK)onTimeFunc4, DWORD(1), TIME_PERIODIC); 
	 
	system("PAUSE");
}




















