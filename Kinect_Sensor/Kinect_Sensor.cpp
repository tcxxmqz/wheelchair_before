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
Joint_Buff Joint_Data[BODY_COUNT];    //�����6���˵Ĺ�����Ϣ
/************************************************************************************
							DataOut_File
							Input 1 : �˵Ĺ����ڵ�ָ��
							Input 2 : ��⵽�ڡ�person_x������
							Input 3 : �Ƿ�ѹ�������Ϣ������txt�ĵ�
							Output: ��25���ڵ��xyz���������txt�ļ�
							���������Ĺ�������Ϣ����Joint_Data[]������
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
//	printf("***Head.X = %f      Head.Y = %f         Head.Z = %f \n\n", Joint_Data[1].x[4], Joint_Data[1].y[4], Joint_Data[1].z[4]);               // ��Ļ��ӡ ͷ����̬��Ϣ  
		
#if BODY_DATA_OUT//�Ƿ������������ļ�
	ofstream Out_File("Body_Data.txt", ios::app);  // �ļ�
		switch (person_x)
		{
		case 1: {//��1���˹�����Ϣ
			for (i = 0; i < 25; i++)
			{
				Out_File << "1         "
					<< setw(2) << i + 1 << "  "
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].x[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].y[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].z[i] << "\n";
			}	}; break;
		case 2: { // ��2���˹�����Ϣ
			for (i = 0; i < 25; i++)
			{
				Out_File << "2         "
					<< setw(2) << i + 1 << "  "
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].x[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].y[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].z[i] << "\n";
			}	}; break;
		case 3: { // ��3���˹�����Ϣ
			for (i = 0; i < 25; i++)
			{
				Out_File << "3         "
					<< setw(2) << i + 1 << "  "
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].x[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].y[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].z[i] << "\n";
			}	}; break;
		case 4: { // ��4���˹�����Ϣ
			for (i = 0; i < 25; i++)
			{
				Out_File << "4         "
					<< setw(2) << i + 1 << "  "
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].x[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].y[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].z[i] << "\n";
			}	}; break;
		case 5: { // ��5���˹�����Ϣ
			for (i = 0; i < 25; i++)
			{
				Out_File << "5         "
					<< setw(2) << i + 1 << "  "
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].x[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].y[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].z[i] << "\n";
			}	}; break;
		case 6: { // ��6���˹�����Ϣ
			for (i = 0; i < 25; i++)
			{
				Out_File << "6         "
					<< setw(2) << i + 1 << "  "
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].x[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].y[i]
					<< setw(CHAR_LEN) << setprecision(PRECISION) << Joint_Data[person_x].z[i] << "\n";
			}	}; break;
		}
		Out_File.close();//�ر�����ļ�
	
#endif
}
/************************************************************************************
								ProcessBody
								Input 1 : ���ɼ�⵽�˵ĸ���
								Input 2 : ��ԭʼ�Ĺ�������Ϣ
								Output: ��
								Author: Chunwei Yu
								Date:2017-07-16
***************************************************************************************/
void ProcessBody(int nBodyCount, IBody** ppBodies)
{
	Body_Data bones_joint;
	HRESULT hr;
	int k = 0;//׼ȷ׷�ٵ��ˣ����ɹ���ȡ����׷�ٵ��˵Ĺ����ڵ�
	IBody* pBody = NULL;
	for (int i = 0; i < nBodyCount; ++i)
	{
		pBody = ppBodies[i];//pBody����һ��ָ�����飬�ܹ���6��ָ�룬����6���˵Ĺ�����Ϣ
		if (pBody)
		{			
			BOOLEAN bTracked = false;
			hr = pBody->get_IsTracked(&bTracked);
			if (SUCCEEDED(hr) && bTracked)
			{
				//printf("TRUE\n");
				
				Joint joints[JointType_Count];//����һ��25���ؽڵ�ṹ��
				hr = pBody->GetJoints(_countof(joints), joints);
				if (SUCCEEDED(hr))
				{
					//Trace_Flag = TRUE;
				 	++k;				
					bones_joint.DataOut_File(&joints[k], k);//�Ѽ�⵽�˵�����������ļ�
				}
				else   //��������ⲻ����ʱ�ѹ�����������
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
						GetAllData()  ��ȡKinect����Դ
						
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

	static Mat i_rgb(CHEIGHT, CWIDTH, CV_8UC4);      //ע�⣺�������Ϊ4ͨ����ͼ��Kinect������ֻ����Bgra��ʽ����
	static Mat i_depth(DHEIGHT, DWIDTH, CV_16UC1);
	static Mat i_ir(DHEIGHT, DWIDTH, CV_16UC1);
	static Mat i_index(DHEIGHT, DWIDTH, CV_8UC3);
	UINT nColorBufferSize = 1920 * 1080 * 4;//��ô������ص㣬�ų�һ��
	UINT16 *depthData = new UINT16[424 * 512];

	hr = m_pMultiFrameReader->AcquireLatestFrame(&m_pMultiFrame);
	if (SUCCEEDED(hr) || m_pMultiFrame)
	{
		// �Ӷ�Դ����֡�з������ɫ���ݣ�������ݺͺ�������
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
							if (buffer[index] != 255)//������255��ʾ��Ϊ��������
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
		
 	     //Body  ֱ�Ӱѹ�������Ϣ ��ȡ��������ļ�
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
		// color������ͼƬ��
		if (SUCCEEDED(hr))
			hr = m_pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(i_rgb.data), ColorImageFormat::ColorImageFormat_Bgra);
		// depth������ͼƬ��
		if (SUCCEEDED(hr))
		{
			hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_depth.data));
		}
		// infrared������ͼƬ��
		if (SUCCEEDED(hr))
		{
			hr = m_pInfraredFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_ir.data));
		}

		//cv::VideoWriter write;
		//write.open("color.avi", CV_FOURCC('M', 'J', 'P', 'G'), 15.0, Size(1080, 1920));
		////write << i_rgb;
		//write << i_rgb;

		//�ͷ���Դ
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
		/****************************����ͼƬ�����************************************/
		if (KEY_DOWN('M'))//��ʼ��ֹͣ
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
				char time_str_dep[64];  //���ʱ����ַ���
				char time_str_index[64];  //���ʱ����ַ���
				char time_str_color[64];  //���ʱ����ַ���
				char str_i[5] = { 0 };//��ż������ַ�����
				++i;

				if (i >= 50)
				{
					i = 0;
					cout << "Geting Data ..." << endl << endl;
				}
					
				sprintf(str_i, "%d", i); //������ֵת��Ϊ�ַ�����ʽ
				time_t t = time(0);
				strftime(time_str_dep, sizeof(time_str_dep), "DEP_%Y-%m-%d %H-%M-%S-", localtime(&t)); //��-��-�� ʱ-��-�� ����ʱ����ַ���
				strcat(time_str_dep, str_i);
				strcat(time_str_dep, ".png");
				imwrite(time_str_dep, i_depth); //���ַ���ƴ�ӳ� ******.jpg ��ʽ �����浽�ļ�

				strftime(time_str_index, sizeof(time_str_index), "INDEX_%Y-%m-%d %H-%M-%S-", localtime(&t)); //��-��-�� ʱ-��-�� ����ʱ����ַ���
				strcat(time_str_index, str_i);
				strcat(time_str_index, ".png");
				imwrite(time_str_index, i_index); //���ַ���ƴ�ӳ� ******.jpg ��ʽ �����浽�ļ�

				//strftime(time_str_color, sizeof(time_str_color), "COLOR_%Y-%m-%d %H-%M-%S-", localtime(&t)); //��-��-�� ʱ-��-�� ����ʱ����ַ���
				//strcat(time_str_color, str_i);
				//strcat(time_str_color, ".png");
				//imwrite(time_str_color, i_rgb); //���ַ���ƴ�ӳ� ******.jpg ��ʽ �����浽�ļ�
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
										Input  : ��
										Output: ��
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
				FrameSourceTypes::FrameSourceTypes_Color |       //��ɫ
				FrameSourceTypes::FrameSourceTypes_Infrared |    //����
				FrameSourceTypes::FrameSourceTypes_Depth |        //���
				FrameSourceTypes::FrameSourceTypes_Body|
				FrameSourceTypes::FrameSourceTypes_BodyIndex,         //����,    //��������
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