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

HANDLE hCOM9;  //�Ϸ����� 9
HANDLE hCOM5;  //����ͨ�� 6
HANDLE hCOM3;  //�·����� 3

DCB DCB_COM9;
DCB DCB_COM5;
DCB DCB_COM3;

UsartCom Usart;
unsigned char gReceive_Buff[1024] = { 0 }; //�������ݻ�����
/******************************************************************************************
								�򿪴���
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
		cout << "\n ���ڴ򿪴���\n";
		return 0;
	}
	else
	{
		cout << "\n ���ڴ򿪳ɹ���\n";
		return hCom;
	}

}

void UsartCom::Close_Com(HANDLE hCom)
{
	CloseHandle(hCom);
	hCom = NULL;
}

/******************************************************************************************
									���ô��ڵĸ������
*******************************************************************************************/
int UsartCom::Get_Com_State(unsigned long baud_rate, HANDLE hCom, DCB dcb)
{
	SetupComm(hCom, 2048, 2048);//�����������������  �ֱ�Ϊ2048
	GetCommState(hCom, &dcb);//��ô��ж˿ڵ�ǰ������Ϣ
	dcb.BaudRate = CBR_115200;
	dcb.BaudRate = baud_rate;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;//����żУ��λ
	dcb.fParity = FALSE;
	dcb.StopBits = ONESTOPBIT;//1λֹͣλ
	dcb.fBinary = TRUE;
	dcb.XonLim = 2048;
	dcb.XoffLim = 512;
	dcb.EofChar = 0;
	SetCommState(hCom, &dcb);//�����޸ĺ��dcb�ṹ�������ô��ж˿���Ϣ
	// ���ô��ڳ�ʱ����
	COMMTIMEOUTS to =                   // ���ڳ�ʱ���Ʋ���
	{
		0,                       // ���ַ������ʱʱ��
		10,                              // ������ʱÿ�ַ���ʱ��
		0,                              // �����ģ�����ģ�����ʱʱ��
		MAXDWORD,                       // д����ʱÿ�ַ���ʱ��
		10                               // �����ģ�����ģ�д��ʱʱ��
	};
	SetCommTimeouts(hCom, &to);
	// ���ô��ڻ������
	SetupComm(hCom, 1024, 1024);
	PurgeComm(hCom, PURGE_TXCLEAR | PURGE_RXCLEAR);
	return TRUE;
}
/******************************************************************************************
								�������ݶ�ȡ
*******************************************************************************************/

int UsartCom::Read_Data(unsigned char *abc, int count, HANDLE hCom)
{
	DWORD wCount;//��ȡ���ֽ���
	BOOL  bReadState;//��ȡ��־λ
	PurgeComm(hCom, PURGE_RXCLEAR);
	bReadState = ReadFile(hCom, abc, count, &wCount, NULL);
	if (!bReadState)
	{
		using namespace std;
		cout << "\n ���ڶ�ȡ����ʧ��\n";
		return FALSE;
	}
	else
	{
		return TRUE;
	}
}
/******************************************************************************************
										�������ݷ���
*******************************************************************************************/
int UsartCom::WriteData(unsigned char *pBuffer, unsigned char uLen, HANDLE hCom)
{
	// д�����ݵ�����
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
								 ��ȡѹ�����ݣ���ȡ��
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
		ofstream outfileP("12Pressure.txt", ios::app);  // �ļ�
		cout << "Pressure sensor data is Outing..." << endl;
		outfileP << tempdata[0] << setw(5) << tempdata[1] << setw(5) << tempdata[2] << setw(5) << tempdata[3] << setw(5) << tempdata[4] << setw(5) << tempdata[5] << endl;
		outfileP.close();
	}
	return tempdata;
}
/******************************************************************************************
							��ȡѹ�����ݣ���ȡ��
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
		ofstream outfileU("Ultrasonic.txt", ios::app);  // �ļ�
		cout << "Ultrasonic sensor data is Outing..." << endl;
		outfileU << tempdata[0] << setw(5) << tempdata[1] << setw(5) << tempdata[2] << setw(5) << tempdata[3] << setw(5) << tempdata[4] << setw(5) << tempdata[5] << endl;
		outfileU.close();
	}
	return tempdata;
}

/******************************************************************************************
			   ����ѹ������������
*******************************************************************************************/
vector<int>UsartCom::Str2Num(unsigned char* str)
{
	char tem_str[6][6] = { { 0 }, { 0 } };//��ά���������������ַ���
	vector<int> data(7);
	//unsigned char i = 0;
	// ��ȡ��й֧Ԯ�����˵�  ������  ����������
	/*if (str[0] == '2' &&str[1] == '_' && str[2] == 'S'&& str[3] == 'N'&& str[4] == ':' && str[34] == '\r' && str[35] == '\n') // �ж�֡ͷ
	{
		int i = 0, num = 0;
		while (str[i] != '\0')
		{
			i++;
			if (i == 35)break;
			if (!(i % 5) && str[i] != '\0')  // 5�����ַ������ɵó�
			{
				int count = 0;
				for (int j = i; j < i + 4; j++) // �ַ����ķָ�
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
	// ��ȡ��й֧Ԯ�����˵�  ѹ��  ����������
	if (str[0] == '4' &&str[1] == '_' && str[2] == 'L'&& str[3] == 'S'&& str[4] == ':')// �ж�֡ͷ
	{
		int i = 0, num = 0;
		while (str[i] != '\0')
		{
			i++;
			if (!(i % 6) && str[i] != '\0')  // 5�����ַ������ɵó�
			{
				int count = 0;
				for (int j = i; j < i + 4; j++) // �ַ����ķָ�
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
	ofstream Out_File(file, ios::app);  // �ļ�
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


