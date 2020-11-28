#ifndef _KINECT_SENSOR_
#define _KINECT_SENSOR
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
#include "Robot_Process.h"
#include "Usart.h"


using namespace std;
//using namespace cv;
/******************************************************************************************
�궨��
*******************************************************************************************/
#define BODY_COUNT 6 
#define JOINT_COUNT 25 
#define CHAR_LEN 20     //����ļ��� ������ռ����
#define PRECISION 5     //���ݾ���
// �����ȡ�ĸ�����Դ
#define GET_DEPTH  1
#define GET_COLOR 1
#define GET_IR     1
#define GET_BODY   1
#define GET_INDEX  1
#define PHOTO_SAVE    1  //��������ͼƬ����ʽ���� ���ͼ��index��
#define BODY_DATA_OUT 1  //�����Ƿ񽫹����������������txt���ļ�
//����ͼƬ�ĳߴ�
#define DHEIGHT 424
#define DWIDTH 512
#define CHEIGHT 1080
#define CWIDTH 1920

/******************************************************************************************
�������ݴ�����
*******************************************************************************************/
class Body_Data
{
public:
	void DataOut_File(Joint* m_pjoint, char person_x);
	BOOL DataOut_Screen(Joint* m_pjoint);
	
	BOOL Body_init();
	void Body_Data_Zero();
	void GetAllData();
public:
	HRESULT hr;
	ofstream out_stream;
	IKinectSensor* m_pKinectSensor = NULL;
	IBodyFrameReader* m_pBodyFrameReader = NULL;
	IMultiSourceFrameReader* m_pMultiFrameReader = NULL; //ָ����� Դ ��ָ��
	IMultiSourceFrame* m_pMultiFrame = nullptr;
	// �ĸ�����֡������
	IDepthFrameReference* m_pDepthFrameReference = NULL;
	IColorFrameReference* m_pColorFrameReference = NULL;
	IInfraredFrameReference* m_pInfraredFrameReference = NULL;
	IBodyFrameReference* m_pBodyFrameReference = NULL;
	IBodyIndexFrameReference* m_pBodyIndexFrameReference = NULL;

	IInfraredFrame* m_pInfraredFrame = NULL;
	IDepthFrame* m_pDepthFrame = NULL;
	IColorFrame* m_pColorFrame = NULL;
	IBodyFrame* m_pIBodyFrame = NULL;
	IBodyIndexFrame* m_pIBodyIndexFrame = NULL;
};


/******************************************************************************************
ȫ�ֱ���
*******************************************************************************************/
struct Joint_Buff   //�������ݽṹ��  ( ȫ�ֱ��� )
{
	float x[JOINT_COUNT];
	float y[JOINT_COUNT];
	float z[JOINT_COUNT];
};


extern Body_Data My_Body_data;
extern Joint_Buff Joint_Data[BODY_COUNT];    //�����6���˵Ĺ�����Ϣ
#endif