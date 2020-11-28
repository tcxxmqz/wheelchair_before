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
宏定义
*******************************************************************************************/
#define BODY_COUNT 6 
#define JOINT_COUNT 25 
#define CHAR_LEN 20     //输出文件中 数字所占长度
#define PRECISION 5     //数据精度
// 定义获取哪个数据源
#define GET_DEPTH  1
#define GET_COLOR 1
#define GET_IR     1
#define GET_BODY   1
#define GET_INDEX  1
#define PHOTO_SAVE    1  //定义是以图片的形式保存 深度图像、index；
#define BODY_DATA_OUT 1  //定义是否将骨骼点坐标输出到‘txt’文件
//定义图片的尺寸
#define DHEIGHT 424
#define DWIDTH 512
#define CHEIGHT 1080
#define CWIDTH 1920

/******************************************************************************************
骨骼数据处理类
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
	IMultiSourceFrameReader* m_pMultiFrameReader = NULL; //指向多种 源 的指针
	IMultiSourceFrame* m_pMultiFrame = nullptr;
	// 四个数据帧及引用
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
全局变量
*******************************************************************************************/
struct Joint_Buff   //骨骼数据结构体  ( 全局变量 )
{
	float x[JOINT_COUNT];
	float y[JOINT_COUNT];
	float z[JOINT_COUNT];
};


extern Body_Data My_Body_data;
extern Joint_Buff Joint_Data[BODY_COUNT];    //最多有6和人的骨骼信息
#endif