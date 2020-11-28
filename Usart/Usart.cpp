#include "Usart.h"
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

HANDLE hCOM9;  //上方激光 9
HANDLE hCOM5;  //无线通信 6
HANDLE hCOM3;  //下方激光 3

DCB DCB_COM9;
DCB DCB_COM5;
DCB DCB_COM3;

UsartCom Usart;
unsigned char gReceive_Buff[1024] = { 0 }; //接收数据缓存区
/******************************************************************************************
								打开串口
*******************************************************************************************/
HANDLE UsartCom::Open_Com(char *com, HANDLE hCom)
{
	using namespace std;
	//CloseHandle(hCOM6);

	hCom = CreateFileA(com,
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		0,
		NULL);
	if (hCom == (HANDLE)-1)
	{
		cout << "\n 串口打开错误！\n";
		return 0;
	}
	else
	{
		cout << "\n 串口打开成功！\n";
		return hCom;
	}

}

void UsartCom::Close_Com(HANDLE hCom)
{
	CloseHandle(hCom);
	hCom = NULL;
}

/******************************************************************************************
									设置串口的各项参数
*******************************************************************************************/
int UsartCom::Get_Com_State(unsigned long baud_rate, HANDLE hCom, DCB dcb)
{
	SetupComm(hCom, 2048, 2048);//设置输入输出缓存区  分别为2048
	GetCommState(hCom, &dcb);//获得串行端口当前配置信息
	dcb.BaudRate = CBR_115200;
	dcb.BaudRate = baud_rate;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;//无奇偶校验位
	dcb.fParity = FALSE;
	dcb.StopBits = ONESTOPBIT;//1位停止位
	dcb.fBinary = TRUE;
	dcb.XonLim = 2048;
	dcb.XoffLim = 512;
	dcb.EofChar = 0;
	SetCommState(hCom, &dcb);//利用修改后的dcb结构重新配置串行端口信息
	// 设置串口超时参数
	COMMTIMEOUTS to =                   // 串口超时控制参数
	{
		0,                       // 读字符间隔超时时间
		10,                              // 读操作时每字符的时间
		0,                              // 基本的（额外的）读超时时间
		MAXDWORD,                       // 写操作时每字符的时间
		10                               // 基本的（额外的）写超时时间
	};
	SetCommTimeouts(hCom, &to);
	// 设置串口缓冲队列
	SetupComm(hCom, 1024, 1024);
	PurgeComm(hCom, PURGE_TXCLEAR | PURGE_RXCLEAR);
	return TRUE;
}
/******************************************************************************************
								串口数据读取
*******************************************************************************************/

int UsartCom::Read_Data(unsigned char *abc, int count, HANDLE hCom)
{
	DWORD wCount;//读取的字节数
	BOOL  bReadState;//读取标志位
	PurgeComm(hCom, PURGE_RXCLEAR);
	bReadState = ReadFile(hCom, abc, count, &wCount, NULL);
	if (!bReadState)
	{
		using namespace std;
		cout << "\n 串口读取数据失败\n";
		return FALSE;
	}
	else
	{
		return TRUE;
	}
}
/******************************************************************************************
										串口数据发送
*******************************************************************************************/
int UsartCom::WriteData(unsigned char *pBuffer, unsigned char uLen, HANDLE hCom)
{
	// 写入数据到串口
	DWORD dwWritten;

	if (uLen > 0)
	{
		dwWritten = 0;
		if (!WriteFile(hCom, pBuffer, uLen, &dwWritten, NULL))
		{
			DWORD dwErr = GetLastError();
			printf("write error %ld\n", dwErr);
			return dwErr;
		}
		else {
			//printf("write done %d\n");
		}
	}
	PurgeComm(hCom, PURGE_TXCLEAR | PURGE_RXCLEAR);
	return 0;
}

/******************************************************************************************
								 获取压力数据（提取后）
*******************************************************************************************/
vector<int>UsartCom::getPressData(unsigned char* PressStr, bool FILE)
{
	//unsigned char PressStr[50] = { 0 };
	vector<int> tempdata(0);
	unsigned char pressStr[21] = { 0x01, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x32, 0x30, 0x30, 0x30, 0x33, 0x0d, 0x0a };
	Usart.WriteData(pressStr, 21, hCOM5);
	while (!Usart.Read_Data(PressStr, 40, hCOM5));
	//cout << PressStr << endl;
	tempdata = Usart.Str2Num(PressStr);
	if (FILE == true)
	{
		cout << "FILECREATE0" << endl;
		ofstream outfileP("12Pressure.txt", ios::app);  // 文件
		cout << "Pressure sensor data is Outing..." << endl;
		outfileP << tempdata[0] << setw(5) << tempdata[1] << setw(5) << tempdata[2] << setw(5) << tempdata[3] << setw(5) << tempdata[4] << setw(5) << tempdata[5] << endl;
		outfileP.close();
	}
	return tempdata;
}
/******************************************************************************************
							获取压力数据（提取后）
*******************************************************************************************/
vector<int>UsartCom::getUltrasonicData(unsigned char* UltraStr, bool FILE)
{
	//unsigned char UltraStr[50] = { 0 };
	vector<int> tempdata(0);
	/*unsigned char chaoshengStr[21] = { 0x01, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x38, 0x30, 0x30, 0x30, 0x30, 0x39, 0x0d, 0x0a };
	Usart.WriteData(chaoshengStr, 21, hCOM6);
	while (!Usart.Read_Data(UltraStr, 36, hCOM6));*/

	//cout << UltraStr << endl;
	tempdata = Usart.Str2Num(UltraStr);
	if (FILE == true)
	{
		ofstream outfileU("Ultrasonic.txt", ios::app);  // 文件
		cout << "Ultrasonic sensor data is Outing..." << endl;
		outfileU << tempdata[0] << setw(5) << tempdata[1] << setw(5) << tempdata[2] << setw(5) << tempdata[3] << setw(5) << tempdata[4] << setw(5) << tempdata[5] << endl;
		outfileU.close();
	}
	return tempdata;
}

/******************************************************************************************
			   解析压力传感器数据
*******************************************************************************************/
vector<int>UsartCom::Str2Num(unsigned char* str)
{
	char tem_str[6][6] = { { 0 }, { 0 } };//二维数组存放六个数字字符串
	vector<int> data(7);
	//unsigned char i = 0;
	// 提取排泄支援机器人的  超声波  传感器数据
	/*if (str[0] == '2' &&str[1] == '_' && str[2] == 'S'&& str[3] == 'N'&& str[4] == ':' && str[34] == '\r' && str[35] == '\n') // 判断帧头
	{
		int i = 0, num = 0;
		while (str[i] != '\0')
		{
			i++;
			if (i == 35)break;
			if (!(i % 5) && str[i] != '\0')  // 5根据字符串规律得出
			{
				int count = 0;
				for (int j = i; j < i + 4; j++) // 字符串的分割
				{
					tem_str[num][count++] = str[j];
				}
				//printf("%s\n", tem_str[num]);
				//data.push_back(atoi(tem_str[num]));
				data[num] = atoi(tem_str[num]);
				num++;
			}
		}

		return data;
	}*/
	// 提取排泄支援机器人的  压力  传感器数据
	if (str[0] == '4' &&str[1] == '_' && str[2] == 'L'&& str[3] == 'S'&& str[4] == ':')// 判断帧头
	{
		int i = 0, num = 0;
		while (str[i] != '\0')
		{
			i++;
			if (!(i % 6) && str[i] != '\0')  // 5根据字符串规律得出
			{
				int count = 0;
				for (int j = i; j < i + 4; j++) // 字符串的分割
				{
					tem_str[num][count++] = str[j];
				}
				//printf("%s\n", tem_str[num]);
				data[num] = atoi(tem_str[num]);
				//data.push_back(atoi(tem_str[num]));
				num++;
			}
		}
		return data;
	}
	return data;
}


void UsartCom::Data2File(float x, float y, float angle, float gyro, char *file)
{
	ofstream Out_File(file, ios::app);  // 文件
	/*static BOOL file_flag = FALSE;
	if (file_flag == FALSE)
	{
		file_flag = TRUE;
	}*/
	Out_File << setw(CHAR_LEN) << setprecision(PRECISION) << x
		<< setw(CHAR_LEN) << setprecision(PRECISION) << y
		<< setw(CHAR_LEN) << setprecision(PRECISION) << angle
		<< setw(CHAR_LEN) << setprecision(PRECISION) << gyro << "\n";


}


