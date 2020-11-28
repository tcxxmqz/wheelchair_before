#ifndef _ROBOT_PROCESS_
#define _ROBOT_PROCESS_
/******************************** 
      ROBOT = 1  WheelChair
      ROBOT = 2  Toilet
**********************************/
#define ROBOT 2

#include <iostream>
#include<windows.h>
 

#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include <iostream>
#include <time.h>
#include <string>
#include <vector>

#define KEY_DOWN(VK_NONAME) ((GetAsyncKeyState(VK_NONAME) & 0x8000) ? 1 : 0) //����

#define PI 3.142
#define RAD_RATIO (PI/180.0)
#define ANGLE_RADIO (180/PI)
#define QUARTER_PI 0.25*PI
#define ONE_THIED_PI 0.333*PI
#define SPEED2ORDER_RADIO_TOILET 100/0.2119
#define SPEED2ORDER_RADIO_CHAIR SPEED2ORDER_RADIO_TOILET*0.46578
#define DOT_RADIO_TOILET 7.874    // ϵ��  ��/ s
#define DOT_RADIO_CHAIR 7.874*0.40486
//#define ROUTE_DATA_COUNT 266272
#define ROUTE_DATA_COUNT 1260
#define FILTER_COUNT 10 // �������˲�������
#define PID_COUNT 10 // �������˲�������
//��ȡ����������
//unsigned char abc[21] = { 0x01, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x38, 0x30, 0x30, 0x30, 0x30, 0x39, 0x0d, 0x0a };
/******************************************************************************************
                         ��������
*******************************************************************************************/
class Robot
{
public:
	//�������˶�ѧ����
	void  Move_Equation(float speed, float angle, float dot, float *v);
	// ָ���ʽת��
	void Data_Cov(float *speed, unsigned char *data_send);

	unsigned char Key_Control_Robot(float  target_speed);
	// ����������ı�
	void PressData2Txt(int *data);
	void FootData2File(float* preKFgoal, float* KFgoal, float* goal, char* str);
	void BackData2File(float* back, char* str);
	void ASData2File(float* goal, float speed, float angle, char* str);

	
	void SaveSitData(float hscore, float *gravity, float *savekfgoal, float *savebackgoal);

	
	
	void Robot_Movement(float d, float real_d, float delt_d, int v, unsigned char *pressStr);
	void Robot_MovementCircle(float v, unsigned char* pressStr);

	
	// �Ƕȷ�Χ�ı任
	float TanAngleConvert(float tempX, float tempY);	
};
struct KFParam
{
	float Q;//q: ��������Э����
	float R;//r : ��������Э����
	float P;//p : ��ʼֵЭ����
	float XkfX_1, PX_1;
	int flag;
};
struct PIDParam
{
	float P;//
	float I;//
	float D;//

	float Set_Value;
	float Input_Value;
	float Out_Valve;
	float err;
	float err_last;
	float integral;
	float out;
};
struct Pid
{
	float Set_Value;
	float Input_Value;
	float Out_Valve;
	float err;
	float err_last;
	float integral;
	float out;
};


struct gCoordinate
{
	float x;
	float y;
};
struct gAngle
{
	float x;
	float y;
	float z;
};
struct gGyro
{
	float x;
	float y;
	float z;
};
struct gRoute
{
	float x;
	float y;
	float angle;
	float gyro;
};

struct gTxt_Route_Buff   //���ڴ洢��txt �ĵ������·��
{
	float x[ROUTE_DATA_COUNT];
	float y[ROUTE_DATA_COUNT];
	float gyro[ROUTE_DATA_COUNT];
	float angle[ROUTE_DATA_COUNT];
};
extern vector<vector<float>> d_D;
extern Robot Robot_Toilet;
extern struct  KFParam KF[FILTER_COUNT];
extern struct  PIDParam PID[PID_COUNT];
#endif