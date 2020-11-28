#include "Kinect_Sensor.h"
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
#include <time.h>
#include <string>
//#include "videoGet\videoprocessor.h" 
using namespace std;
Body_Data My_Body_data;
Joint_Buff Joint_Data[BODY_COUNT];    //最多有6和人的骨骼信息
/************************************************************************************
							DataOut_File
							Input 1 : 人的骨骼节点指针
							Input 2 : 检测到第“person_x”个人
							Input 3 : 是否把骨骼点信息导出到txt文档
							Output: 将25个节点的xyz数据输出到txt文件
							并将处理后的骨骼点信息存于Joint_Data[]数组中
							Author: Chunwei Yu
							Date:2017-07-16
************************************************************************************/
void Body_Data::DataOut_File(Joint* m_pjoint, char person_x)
{
	
	int i = 0;	
		for (i = 0; i < 25; i++)
		{
			Joint_Data[person_x].x[i] = m_pjoint[i].Position.X;
			Joint_Data[person_x].y[i] = m_pjoint[i].Position.Y;
			Joint_Data[person_x].z[i] = m_pjoint[i].Position.Z;


			if ((abs(Joint_Data[person_x].x[i]) < 0.01) || (abs(Joint_Data[person_x].x[i]) > 10))
			{
				Joint_Data[person_x].x[i] = 0.0;
			}

			if ((abs(Joint_Data[person_x].y[i]) < 0.01) || (abs(Joint_Data[person_x].y[i]) > 10))
			{
				Joint_Data[person_x].y[i] = 0.0;
			}

			if ((abs(Joint_Data[person_x].z[i]) < 0.01) || (abs(Joint_Data[person_x].z[i]) > 10))
			{
				Joint_Data[person_x].z[i] = 0.0;
			}
		}
//	printf("***Head.X = %f      Head.Y = %f         Head.Z = %f \n\n", Joint_Data[1].x[4], Joint_Data[1].y[4], Joint_Data[1].z[4]);               // 屏幕打印 头部姿态信息  
		
#if BODY_DATA_OUT//是否把数据输出到文件
	ofstream Out_File("Body_Data.txt", ios::app);  // 文件
		switch (person_x)
		{
		case 1: {//第1个人骨骼信息
			for (i = 0; i < 25; i++)
			{
				Out_File << "1         "
					<< setw(2) << i + 1 << "  "
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].x[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].y[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].z[i] << "\n";
			}	}; break;
		case 2: { // 第2个人骨骼信息
			for (i = 0; i < 25; i++)
			{
				Out_File << "2         "
					<< setw(2) << i + 1 << "  "
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].x[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].y[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].z[i] << "\n";
			}	}; break;
		case 3: { // 第3个人骨骼信息
			for (i = 0; i < 25; i++)
			{
				Out_File << "3         "
					<< setw(2) << i + 1 << "  "
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].x[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].y[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].z[i] << "\n";
			}	}; break;
		case 4: { // 第4个人骨骼信息
			for (i = 0; i < 25; i++)
			{
				Out_File << "4         "
					<< setw(2) << i + 1 << "  "
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].x[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].y[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].z[i] << "\n";
			}	}; break;
		case 5: { // 第5个人骨骼信息
			for (i = 0; i < 25; i++)
			{
				Out_File << "5         "
					<< setw(2) << i + 1 << "  "
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].x[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].y[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].z[i] << "\n";
			}	}; break;
		case 6: { // 第6个人骨骼信息
			for (i = 0; i < 25; i++)
			{
				Out_File << "6         "
					<< setw(2) << i + 1 << "  "
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].x[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].y[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].z[i] << "\n";
			}	}; break;
		}
		Out_File.close();//关闭输出文件
	
#endif
}
/************************************************************************************
								ProcessBody
								Input 1 : 最大可检测到人的个数
								Input 2 : 最原始的骨骼点信息
								Output: 空
								Author: Chunwei Yu
								Date:2017-07-16
***************************************************************************************/
void ProcessBody(int nBodyCount, IBody** ppBodies)
{
	Body_Data bones_joint;
	HRESULT hr;
	int k = 0;//准确追踪到人，并成功获取的所追踪到人的骨骼节点
	IBody* pBody = NULL;
	for (int i = 0; i < nBodyCount; ++i)
	{
		pBody = ppBodies[i];//pBody中是一个指针数组，总共有6个指针，代表6个人的骨骼信息
		if (pBody)
		{			
			BOOLEAN bTracked = false;
			hr = pBody->get_IsTracked(&bTracked);
			if (SUCCEEDED(hr) && bTracked)
			{
				//printf("TRUE\n");
				
				Joint joints[JointType_Count];//定义一个25个关节点结构体
				hr = pBody->GetJoints(_countof(joints), joints);
				if (SUCCEEDED(hr))
				{
					//Trace_Flag = TRUE;
				 	++k;				
					bones_joint.DataOut_File(&joints[k], k);//把检测到人的数据输出到文件
				}
				else   //待定当检测不到人时把骨骼数据清零
				{
					//Trace_Flag = FALSE;
				}	
			}
			
		}
	}
	
////	if (Trace_Flag== FALSE)
//	{
//			for (int m = 0; m < 6; m++)
//				{
//					for (int j = 0; j < 25; j++)
//					{
//							Joint_Data[m].x[j] = 0;
//							Joint_Data[m].y[j] = 0;
//							Joint_Data[m].z[j] = 0;
//					}
//				}
//		}
}
/************************************************************************************
						GetAllData()  获取Kinect所有源
						
						Author: Chunwei Yu
						Date:2017-07-16
***************************************************************************************/
#define DHEIGHT 424
#define DWIDTH 512
#define CHEIGHT 1080
#define CWIDTH 1920
BOOL S_T = FALSE;
void Body_Data::GetAllData()
{
	HRESULT hr;

	static Mat i_rgb(CHEIGHT, CWIDTH, CV_8UC4);      //注意：这里必须为4通道的图，Kinect的数据只能以Bgra格式传出
	static Mat i_depth(DHEIGHT, DWIDTH, CV_16UC1);
	static Mat i_ir(DHEIGHT, DWIDTH, CV_16UC1);
	static Mat i_index(DHEIGHT, DWIDTH, CV_8UC3);
	UINT nColorBufferSize = 1920 * 1080 * 4;//这么多的像素点，排成一列
	UINT16 *depthData = new UINT16[424 * 512];

	hr = m_pMultiFrameReader->AcquireLatestFrame(&m_pMultiFrame);
	if (SUCCEEDED(hr) || m_pMultiFrame)
	{
		// 从多源数据帧中分离出彩色数据，深度数据和红外数据
		// color
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_ColorFrameReference(&m_pColorFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pColorFrameReference->AcquireFrame(&m_pColorFrame);
		// Depth
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_DepthFrameReference(&m_pDepthFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pDepthFrameReference->AcquireFrame(&m_pDepthFrame);
		 //Body Index
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_BodyIndexFrameReference(&m_pBodyIndexFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pBodyIndexFrameReference->AcquireFrame(&m_pIBodyIndexFrame);
		if (SUCCEEDED(hr))
		{		
				unsigned int buffersize = 0;
				unsigned char* buffer = nullptr;
				hr = m_pIBodyIndexFrame->AccessUnderlyingBuffer(&buffersize, &buffer);
				if (SUCCEEDED(hr))
				{
					for (int x = 0; x < DHEIGHT; x++)
					{
						for (int y = 0; y < DWIDTH; y++)

						{
							unsigned int index = x * 512 + y;
							//cout << buffer[index] << endl;
							if (buffer[index] != 255)//不等于255表示的为人体区域
							{
								i_index.at<Vec3b>(x, y) = Vec3b(0, 0, 255);
							}
							else
							{
								i_index.at<Vec3b>(x, y) = Vec3b(0, 0, 0);
							}
						}
					}
				}	
		}
		// Infrared
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_InfraredFrameReference(&m_pInfraredFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pInfraredFrameReference->AcquireFrame(&m_pInfraredFrame);
		
 	     //Body  直接把骨骼点信息 读取并输出到文件
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_BodyFrameReference(&m_pBodyFrameReference);
		if (SUCCEEDED(hr))
		{
            hr = m_pBodyFrameReference->AcquireFrame(&m_pIBodyFrame);
			if (SUCCEEDED(hr))
			{
				INT64 nTime = 0;

				hr = m_pIBodyFrame->get_RelativeTime(&nTime);

				IBody* ppBodies[BODY_COUNT] = { 0 };

				if (SUCCEEDED(hr))
				{
					hr = m_pIBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
				}
				if (SUCCEEDED(hr))
				{
					ProcessBody(BODY_COUNT, ppBodies);
				}
				for (int i = 0; i < _countof(ppBodies); ++i)
				{
					SafeRelease(ppBodies[i]);
				}
			}
		}
		// color拷贝到图片中
		if (SUCCEEDED(hr))
			hr = m_pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(i_rgb.data), ColorImageFormat::ColorImageFormat_Bgra);
		// depth拷贝到图片中
		if (SUCCEEDED(hr))
		{
			hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_depth.data));
		}
		// infrared拷贝到图片中
		if (SUCCEEDED(hr))
		{
			hr = m_pInfraredFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_ir.data));
		}

		//cv::VideoWriter write;
		//write.open("color.avi", CV_FOURCC('M', 'J', 'P', 'G'), 15.0, Size(1080, 1920));
		////write << i_rgb;
		//write << i_rgb;

		//释放资源
		SafeRelease(m_pColorFrame);
		SafeRelease(m_pDepthFrame);
		SafeRelease(m_pInfraredFrame);
		SafeRelease(m_pIBodyFrame);
		SafeRelease(m_pIBodyIndexFrame);

		SafeRelease(m_pColorFrameReference);
		SafeRelease(m_pDepthFrameReference);
		SafeRelease(m_pInfraredFrameReference);
		SafeRelease(m_pBodyFrameReference);
		SafeRelease(m_pBodyIndexFrameReference);

		SafeRelease(m_pMultiFrame);
		m_pMultiFrame = NULL;
#if PHOTO_SAVE
		// static BOOL S_T = FALSE;
		/****************************保存图片程序段************************************/
		if (KEY_DOWN('M'))//开始或停止
		{
			S_T = TRUE;
			cout << "Saving..." << endl;
		}
		if (KEY_DOWN('N'))
		{
			S_T = FALSE;
			cout << "Stoping..." << endl;
		}
			if (S_T)
			{
				static int i = 0;
				char time_str_dep[64];  //存放时间的字符串
				char time_str_index[64];  //存放时间的字符串
				char time_str_color[64];  //存放时间的字符串
				char str_i[5] = { 0 };//存放计数的字符数组
				++i;

				if (i >= 50)
				{
					i = 0;
					cout << "Geting Data ..." << endl << endl;
				}
					
				sprintf(str_i, "%d", i); //将计数值转换为字符串格式
				time_t t = time(0);
				strftime(time_str_dep, sizeof(time_str_dep), "DEP_%Y-%m-%d %H-%M-%S-", localtime(&t)); //年-月-日 时-分-秒 生成时间的字符串
				strcat(time_str_dep, str_i);
				strcat(time_str_dep, ".png");
				imwrite(time_str_dep, i_depth); //将字符串拼接成 ******.jpg 格式 并保存到文件

				strftime(time_str_index, sizeof(time_str_index), "INDEX_%Y-%m-%d %H-%M-%S-", localtime(&t)); //年-月-日 时-分-秒 生成时间的字符串
				strcat(time_str_index, str_i);
				strcat(time_str_index, ".png");
				imwrite(time_str_index, i_index); //将字符串拼接成 ******.jpg 格式 并保存到文件

				//strftime(time_str_color, sizeof(time_str_color), "COLOR_%Y-%m-%d %H-%M-%S-", localtime(&t)); //年-月-日 时-分-秒 生成时间的字符串
				//strcat(time_str_color, str_i);
				//strcat(time_str_color, ".png");
				//imwrite(time_str_color, i_rgb); //将字符串拼接成 ******.jpg 格式 并保存到文件
			}
			
			
#endif
		/*****************************************************************************/
	}
	/*imshow("rgb", i_rgb);
	imshow("depth", i_depth);
	imshow("BodyIndex", i_index);
	imshow("ir", i_ir);*/
	
}



/************************************************************************************
										Body_init
										Input  : 无
										Output: 无
										Author: Chunwei Yu
										Date:2017-07-16
***************************************************************************************/
BOOL Body_Data::Body_init()
{
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return FALSE;
	}
	m_pKinectSensor->Close();
	if (m_pKinectSensor)
	{
		hr = m_pKinectSensor->Open();
		IBodyFrameSource* pBodyFrameSource = NULL;//  
		if (SUCCEEDED(hr))
		{
			
			//hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);//
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Color |       //彩色
				FrameSourceTypes::FrameSourceTypes_Infrared |    //红外
				FrameSourceTypes::FrameSourceTypes_Depth |        //深度
				FrameSourceTypes::FrameSourceTypes_Body|
				FrameSourceTypes::FrameSourceTypes_BodyIndex,         //骨骼,    //人体索引
				&m_pMultiFrameReader);
		}
		if (!m_pKinectSensor || FAILED(hr))
		{
			return E_FAIL;
		}
		return TRUE;
	}
}


void  Body_Data::Body_Data_Zero()
{
	
}