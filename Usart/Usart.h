#ifndef _USART_
#define _USART_

//#include "opencv2/core/core.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
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
#include <vector>
using namespace std;
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
								 		������
*******************************************************************************************/
class UsartCom
{
public:

	HANDLE UsartCom::Open_Com(char *com, HANDLE hCom);
	void Close_Com(HANDLE hCom);
	int Get_Com_State(unsigned long baud_rate, HANDLE hCom, DCB dcb);
	int Read_Data(unsigned char *abc, int count, HANDLE hCom);
	int WriteData(unsigned char *pBuffer, unsigned char uLen, HANDLE hCom);
	//void Str2Num(unsigned char* str, int* data);
	vector<int> Str2Num(unsigned char* str);
	void Data2File(float x, float y, float angle, float gyro,char *file);

	vector<int> getPressData(unsigned char* PressStr, bool FILE);
	vector<int> getUltrasonicData(unsigned char* UltraStr, bool FILE); 

	int postData(unsigned char *pBuffer, unsigned char uLen, HANDLE hCom);//by myself
};


extern UsartCom Usart;
extern HANDLE hCOM9;  //�Ϸ����� 9
extern HANDLE hCOM5;  //����ͨ�� 6
extern HANDLE hCOM3;  //�·����� 3

extern DCB DCB_COM9;  
extern DCB DCB_COM5;  
extern DCB DCB_COM3;


#endif