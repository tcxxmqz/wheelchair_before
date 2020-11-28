#include "Lasor.h"
#include <iostream>
#include <string.h>
#include <windows.h>
#include "Usart.h"
#include <vector>
#include <math.h>
using namespace std;
vector<vector<int>> gOriginal_data1;//���ڴ�ż����״�����������  �д�Ŷ�Ӧ�ĵ�  �д�ŵ�����Ӧ�ľ��루mm��    C1:�� n ��  C2:��->����
vector<vector<int>> gOriginal_data2;//���ڴ�ż����״�����������  �д�Ŷ�Ӧ�ĵ�  �д�ŵ�����Ӧ�ľ��루mm��    C1:�� n ��  C2:��->����
float gDis_Angle[2] = { 0 };
unsigned char gLaser_Str_Buff1[LASER_DATA_BUFF_SIZE] = { 0 }; // �·������״ﻺ����
unsigned char gLaser_Str_Buff2[LASER_DATA_BUFF_SIZE] = { 0 }; // �Ϸ������״ﻺ����
Laser_Sensor URG04LX;

/******************************************************************************************
��ȡ�·����⴫��������
Input 1���Ƿ�������ļ�
Return ������ԭʼ����  n * 2 ������
*******************************************************************************************/
vector<vector<int>> Laser_Sensor::getLaserDownData(bool OUTFILE)
{
	vector<vector<int>>  tempdata(0);
	while (!URG04LX.Obtain_Data(START_STEP, ENDING_STEP, gLaser_Str_Buff1, 1));//��ȡ�·����⴫��������
	tempdata = URG04LX.Data_Extract(gLaser_Str_Buff1);
	if (OUTFILE == true)URG04LX.LasorDataOut(tempdata, 1);
	if (!tempdata.empty())
	{
		return tempdata;
	}
}

/******************************************************************************************
						��ȡ�Ϸ����⴫��������
						Input 1���Ƿ�������ļ�
						Return ������ԭʼ����  n * 2 ������
*******************************************************************************************/
vector<vector<int>> Laser_Sensor::getLaserUpData(bool OUTFILE)
{
	vector<vector<int>>  tempdata(0);
	while (!URG04LX.Obtain_Data(START_STEP, ENDING_STEP, gLaser_Str_Buff2, 2));//��ȡ�·����⴫��������
	tempdata = URG04LX.Data_Extract(gLaser_Str_Buff2);
	if (OUTFILE == true)URG04LX.LasorDataOut(tempdata, 2);
	if (!tempdata.empty())
	{
		return tempdata;
	}
}
/******************************************************************************************
						�Ϸ��������ݴ�����������
						Input 1������ֵ���ļ���ԭʼ����
						Return ��������λ����Ϣ ��X Y �ȣ� �����뱳��ƫ��ĽǶ�
*******************************************************************************************/

#define S_O_SEP_THRESHOLD 500  //�Ϸ���������ָ���ֵ
#define S_START_ANGLE -30      // �Ϸ�����ļ�⿪ʼ�Ƕ�  
#define S_END_ANGLE 30         //�Ϸ�����ļ����ֹ�Ƕ�

void Laser_Sensor::UpLaserDataProc(vector<vector<int>>up_laser, vector<float> &back)
{
	float tem_back[3] = { 0 };
	int start_step = (int)(341 + S_START_ANGLE / 0.353);
	int end_step = (int)(341 + S_END_ANGLE / 0.353);
	int tem_step = 0, tem_dis = 0, ip = start_step;

	//for (int i = 0; i < STEP - 1; i++)
	for (int i = start_step; i < end_step; i++)
	{
		if ((abs(up_laser[i + 1][1] - up_laser[i][1]) >= S_O_SEP_THRESHOLD) || i == 424)
		{

			int ocount = 0;  //��������ļ���
			vector < vector<int> > tem_laser;
			int N = i - ip;
			tem_laser.resize(N);//ipΪ��һ�ε�iֵ
			for (int m = 0; m < N; ++m)
			{
				tem_laser[m].resize(2);
			}

			for (int j = ip + 1; j < i + 1; j++) // �����������֮������� ���������ݴ洢��һ����ʱbuff ֱ���͵��Ȳ���⺯��
			{
				if (up_laser[j][1] >= 0)
				{
					tem_laser[ocount][0] = up_laser[j][0]; //step   // �˴���ֵ�д���
					tem_laser[ocount][1] = up_laser[j][1]; //dis
					ocount++;
					if (j == 424)break;
				}
				else
				{
					tem_laser.clear();
				}

			}
			BackFind(tem_laser, tem_back);// �����Ȳ����
			if (tem_back[0] != 0 && tem_back[1] != 0 && tem_back[2] != 0)
			{
				back.push_back(tem_back[0]);    // x����
				back.push_back(tem_back[1]);    // y����
				back.push_back(tem_back[2]);    // R�뾶
				tem_back[0] = 0;
				tem_back[1] = 0;
				tem_back[2] = 0;
			}
			ip = i;        //��¼��һ��iֵ
		}
	}
}
/******************************************************************************************
			�Ϸ���������λ����ȡ
			Input 1���ѷ������������
			Return ������λ����Ϣ ��X Y �ȣ�
*******************************************************************************************/
// �ָ�������������̫���̫С����ȥ
// ���������ռ�ĽǶ�
#define MAX_DIS_AVE 1000
#define MIN_DIS_AVE 100
#define ANGLE_THROLD 10
void Laser_Sensor::BackFind(vector<vector<int>>sub_object, float* back)
{
	int N = sub_object.size(), sum_dis = 0, ave_dis = 0;
	vector<float> tem_x(N), tem_y(N);
	float theta = 0;
	if (N <= 10)
	{
		return;
	}
	else
	{
		float sumx = 0, sumy = 0, ave_x = 0, ave_y = 0;
		// ���������ȫת��Ϊֱ������ĵ�
		for (int i = 0; i < N; i++)
		{
			sum_dis += sub_object[i][1];
			tem_x[i] = (float)(sub_object[i][1] * cos((90 + (sub_object[i][0] - 385) * LASER_KA) *0.0175));
			tem_y[i] = (float)(sub_object[i][1] * sin((90 + (sub_object[i][0] - 385) * LASER_KA) *0.0175));
			sumx += tem_x[i];
			sumy += tem_y[i];
		}
		// ����ƽ��ֵ  ���Ƕȼ�Ϊ N �ļ��������ǵĿ��
		ave_dis = sum_dis / N;

		if ((ave_dis >= MIN_DIS_AVE) && (ave_dis <= MAX_DIS_AVE) && N > ANGLE_THROLD)
		{
			theta = (90 + (sub_object[(int)(N / 2)][0] - 385) * 0.35);
			ave_x = sumx / N;
			ave_y = sumy / N;
			back[0] = ave_x;
			back[1] = ave_y;
			back[2] = theta;
		}
		else
		{
			back[0] = 0;
			back[1] = 0;
			back[2] = 0;
		}

		return;
	}

}
/******************************************************************************************
							�·��������ݴ���Ѱ���Ȳ�
							Input 1������ֵ���ļ���ԭʼ����
							Return ���Ȳ���λ�ü���С��Ϣ
*******************************************************************************************/
#define X_O_SEP_THRESHOLD 40  //�·���������ָ���ֵ
void Laser_Sensor::DownLaserDataProc(vector<vector<int>>down_laser, vector<float> &foot)
{
	float tem_foot[3] = { 0 };
	int tem_step = 0, tem_dis = 0, ip = 0;
	for (int i = 0; i < STEP - 1; i++)
	{
		if (abs(down_laser[i + 1][1] - down_laser[i][1]) >= X_O_SEP_THRESHOLD)
		{

			int ocount = 0;  //��������ļ���
			vector < vector<int> > tem_laser;
			int N = i - ip;
			tem_laser.resize(N);//ipΪ��һ�ε�iֵ
			for (int m = 0; m < N; ++m)
			{
				tem_laser[m].resize(2);
			}

			for (int j = ip + 1; j < i + 1; j++) // �����������֮������� ���������ݴ洢��һ����ʱbuff ֱ���͵��Ȳ���⺯��
			{
				if (down_laser[j][1] > 20)
				{
					tem_laser[ocount][0] = down_laser[j][0]; //step   // �˴���ֵ�д���
					tem_laser[ocount][1] = down_laser[j][1]; //dis
					ocount++;
				}
				else
				{
					tem_laser.clear();
				}

			}
			FootFind(tem_laser, tem_foot);// �����Ȳ����
			if (tem_foot[0] != 0 && tem_foot[1] != 0 && tem_foot[2] != 0)
			{
				foot.push_back(tem_foot[0]);    // x����
				foot.push_back(tem_foot[1]);    // y����
				foot.push_back(tem_foot[2]);    // R�뾶
				tem_foot[0] = 0;
				tem_foot[1] = 0;
				tem_foot[2] = 0;
			}

			ip = i;        //��¼��һ��iֵ
		}
	}
}
/******************************************************************************************
					�Էָ�����������Ȳ���Բ���
					Input 1���ָ�õ�����
					Return ����Բ�� �뾶 �� ���ģ�x, y��
*******************************************************************************************/
//char NN = 0;
void Laser_Sensor::FootFind(vector<vector<int>>sub_object, float* foot)
{
	int N = sub_object.size();
	vector<float> tem_x(N), tem_y(N);
	/*NN++;
	if (NN == 8)
	{
		cout << NN << endl;
	}*/
	// �ָ���������С��5���� ��ֱ�ӷ���
	if (N <= 10)
	{

		return;
	}
	else
	{
		//ֻȡ��������ϵ�����������Ȳ���⡣
		float x1 = 0, x2 = 0, x4 = 0, Xm = 0, Xn = 0, X = 0;
		float y1 = 0, y2 = 0, y4 = 0, Ym = 0, Yn = 0, Y = 0;
		float k1 = 0, k2 = 0, k14 = 0, d14 = 0, D = 0, R = 0;
		// ȡ���������е�
		int mid = N / 2;
		float sum_x = 0, sum_y = 0;
		// ���������ȫת��Ϊֱ������ĵ�
		for (int i = 0; i < N; i++)
		{
			tem_x[i] = (float)(sub_object[i][1] * cos((90 + (sub_object[i][0] - 385) * LASER_KA) *0.0175));
			tem_y[i] = (float)(sub_object[i][1] * sin((90 + (sub_object[i][0] - 385) * LASER_KA) *0.0175));
			sum_x = sum_x + tem_x[i];
			sum_y = sum_y + tem_y[i];

		}
		x1 = (tem_x[0] + tem_x[1] + tem_x[2]) / 3;
		y1 = (tem_y[0] + tem_y[1] + tem_y[2]) / 3;
		x2 = (tem_x[mid - 1] + tem_x[mid - 2] + tem_x[mid - 3] + tem_x[mid] + tem_x[mid + 1] + tem_x[mid + 2] + tem_x[mid + 3]) / 7;
		y2 = (tem_y[mid - 1] + tem_y[mid - 2] + tem_y[mid - 3] + tem_y[mid] + tem_y[mid + 1] + tem_y[mid + 2] + tem_y[mid + 3]) / 7;
		x4 = (tem_x[N - 1] + tem_x[N - 2] + tem_x[N - 3]) / 3;
		y4 = (tem_y[N - 1] + tem_y[N - 2] + tem_y[N - 3]) / 3;
		// ���������Ҵ��ߵ�б��
		k1 = -(x2 - x1) / (y2 - y1);
		k2 = -(x4 - x2) / (y4 - y2);
		// k1 �����ҵ��е�
		Xm = (x1 + x2) / 2; Ym = (y1 + y2) / 2;
		// k2 �����ҵ��е�
		Xn = (x4 + x2) / 2; Yn = (y4 + y2) / 2;
		// P1��P4��ľ���
		d14 = sqrt(pow(y4 - y1, 2) + pow((x4 - x1), 2));
		// P1 P4 ����ֱ�ߵ�б��
		k14 = (y4 - y1) / (x4 - x1);
		//P1 P4 ���е����߶�P1P4 �ľ���
		D = abs((-k14 * x2 + y2 + k14 * x1 - y1) / (sqrt(k14 * k14 + 1)));
		X = (k1*Xm - k2 * Xn + Yn - Ym) / (k1 - k2);
		Y = k1 * (X - Xm) + Ym;
		R = (float)(sqrt(pow((X - x1), 2) + pow((Y - y1), 2)) + sqrt(pow((X - x2), 2) + pow((Y - y2), 2)) + sqrt(pow((X - x4), 2) + pow((Y - y4), 2))) / 3.0;
		//����˵����ɵ�ֱ��, ԭ�����꣨x, y������ Ax + By + C>0
		float	A = -(y4 - y1), B = x4 - x1, C = x1 * (y4 - y1) - y1 * (x4 - x1);
		bool tempTF = false;
		if (A*X + B * Y + C <= 0)tempTF = true;
		// �ж��Ƿ���ϽŴ�С��Բ  D < ��ֵԽ��Խ�ܼ������ʽ�С������
		if (D > 0.005*d14 && D < 1.1*d14 && R < 200 && R>10 && tempTF)
		{

			//foot[0] = X;
			//foot[1] = Y;
			foot[2] = R;
			foot[0] = sum_x / N;
			foot[1] = sum_y / N;

		}
		else
		{
			return;
		}
	}
}
/******************************************************************************************
					���⴫�����ַ����ݽ���
					Input 1���������ַ���
					Return ������ԭʼ����  n * 2 ������
*******************************************************************************************/
vector<vector<int>> Laser_Sensor::Data_Extract(unsigned char *data_str)
{
	vector<vector<int>>data_array;

	data_array.resize(STEP);
	for (int i = 0; i < STEP; ++i)
	{
		data_array[i].resize(DIS);
	}

	int length = 0, starting_step = 0, ending_step = 0;
	int step_temp[8] = { 0 }, data_byte = 0, data_block = 0, data_remain = 0, data_remain_start = 0;
	int k = 1000, count = 0;//count�����������洢���ݵ����յ�����
	int data_length = 0, temp = 0, p = 0, q = 0;
	int temp_sum = 0, data_temp_high = 0, data_temp_low = 0, data;
	//length = strlen(data_str);//������26 bytes �̶����ȵ�����  ���� Ϊ���⴫��������
	while (data_str[count])  //�����յ����ݵĳ���
	{
		count++;
	}
	length = count;
	count = 0;
	//cout << "���ݵ���Ϊ��" << length << " byte" << endl;
	data_str[length] = 0x00;
	if (length <= 1450)  //�����ݸ��� >= 64bytes ʱ�������  
	{
		if (data_str[0] == 'G' && data_str[1] == 'S' && data_str[length - 1] == 0x0A && data_str[length - 2] == 0x0A)
		{
			//************�� for ѭ�� ������ȡ ��ʼ�� �� ������***************//
			for (int i = 0; i < 8; i++)
			{
				if (i >= 0 && i <= 3)
				{
					step_temp[i] = (data_str[i + 2] - 0x30) * k;
					k = (int)(k / 10);
					starting_step += step_temp[i];//��ȡ�ڿ�ʼ��
				}
				else
				{
					if (i == 4) k = 1000;
					step_temp[i] = (data_str[i + 2] - 0x30) * k;
					k = (int)(k / 10);
					ending_step += step_temp[i];//��ȡ�ڽ�����
				}
			}
			/* cout << "���ݳ���Ϊ��" << ending_step - starting_step + 1 << " ��,"
				  << starting_step << " ~ " << ending_step << endl << endl;*/
			if (data_str[22] == 0x0A)//���ĺ����������
			{
				int j = 0, i = 0, m = 0;
				data_length = ending_step - starting_step + 1;// 66�� 
				data_byte = data_length * 2;
				data_block = data_byte / 64;  //���ݿ���  64�ֽ�Ϊһ��
				data_remain = data_byte % 64; //ʣ��������ռ�ֽ�
	//******************** ���ݿ� ����********************************//
				for (i = 0; i < data_block; i++)  //���ݿ���
				{
					temp_sum = 0;
					for (j = 0; j < 64; j++)      //ÿһ�����ݸ��� 64 
					{
						temp_sum += data_str[23 + i * 66 + j];
					}
					temp_sum = temp_sum & 0x3F;
					temp_sum = temp_sum + 0x30; //ȡ�ܺ͵ĵ���λ ��У���
					p = 22 + (i + 1) * 66 - 1;  //Ϊ sum ������
					temp = data_str[p];
					/*if (i == 0){ p = 23 + i * 64 + j; temp = data_str[p]; }
					else { q = 23 + i * 64 + j + 2; temp = data_str[q]; }*/
					if (temp_sum == temp)//����У��� �� �����ܵ�У����Ƿ����
					{                                             //��ʼ��ȡ����
						for (m = 0; m < 64; m = m + 2)      //ÿһ�����ݸ��� 64 
						{
							data_temp_high = data_str[23 + i * 66 + m] - 0x30;
							data_temp_high = data_temp_high & 0x3F;//ȡ���ݵĸ���λ
							data_temp_high = data_temp_high << 6;
							data_temp_low = data_str[23 + i * 66 + m + 1] - 0x30;//ȡ���ݵĵ���λ
							data_temp_low = data_temp_low & 0x3F;
							data = data_temp_high + data_temp_low;//���ݺϳ�	
							data_array[count][0] = starting_step + count;  //�����ݴ洢�����յ�������
							data_array[count][1] = data;
							//cout << data_array[count][0] << endl;
							count++;
						}
					}
				}
				//********************ʣ�����ݴ���********************************//
				data_remain_start = data_block * 66 + 23;
				temp_sum = 0;
				for (j = data_remain_start; j < data_remain_start + data_remain; j++)      //ÿһ�����ݸ��� 64 
				{
					temp_sum += data_str[j];
				}
				temp_sum = temp_sum & 0x3F;
				temp_sum = temp_sum + 0x30; //ȡ�ܺ͵ĵ���λ ��У���
				if (temp_sum == data_str[data_remain_start + data_remain])//����У��� �� �����ܵ�У����Ƿ����
				{
					for (int j = data_remain_start; j < data_remain_start + data_remain; j += 2)
					{
						data_temp_high = data_str[j] - 0x30;
						data_temp_high = data_temp_high & 0x3F;//ȡ���ݵĸ���λ
						data_temp_high = data_temp_high << 6;
						data_temp_low = data_str[j + 1] - 0x30;//ȡ���ݵĵ���λ
						data_temp_low = data_temp_low & 0x3F;
						data = data_temp_high + data_temp_low;//���ݺϳ�	
						data_array[count][0] = starting_step + count;  //�����ݴ洢�����յ�������
						data_array[count][1] = data;
						//cout << data_array[count][0] << endl;
						count++;
					}
				}
			}
		}
		/* for (int i = 0; data_array[i][0] != 0; i++)
		  {
			  cout << data_array[i][0] << "  " << data_array[i][1] << endl;
		  }*/
		  //memset(data_array, 0, sizeof(data_array));
	}
	return data_array;
}
/******************************************************************************************
							  ���⴫�����ĳ�ʼ������
*******************************************************************************************/
void Laser_Sensor::Laser_Init(int LASERx)
{
	unsigned char gBit_Rate[] = { 0x53, 0x53, 0x31, 0x31, 0x35, 0x32, 0x30, 0x30, 0x0A };
	unsigned char gOpen_Laser[] = { 0x42, 0x4D, 0x0A };
	unsigned char data_buff[] = { 0 };
	HANDLE COMx = NULL;
	DCB DCBx = { };
	char com3[] = { "COM5" };
	char com9[] = { "COM4" };
	char comStr[5] = { 0 };
	char flag = 0;
	char i = 0;
	if (LASERx == 1)
	{
		strcpy(comStr, com3);
	}
	if (LASERx == 2)
	{
		strcpy(comStr, com9);
	}
	COMx = Usart.Open_Com(comStr, COMx); //�򿪴���
	Usart.Get_Com_State(19200, COMx, DCBx); //���ò�����Ϊ 115200
	Sleep(500);
	Usart.WriteData(gBit_Rate, 9, COMx); //�Ѳ������޸ĳ�115200
	Sleep(100);

	while (!Usart.Read_Data(data_buff, 14, COMx))////�ȴ����ڷ��ص�����  ��ʱ��ǿ���˳�
	{
		i++;
		Sleep(100);
		if (i >= 10)
		{
			i = 0;
			break;
		}
	}
	if (data_buff[9] == 0x30 && data_buff[10] == 0x30)
	{
		//flag = 0x01;  //�޸Ĳ����ʳɹ�
	}
	Usart.Close_Com(COMx);
	Sleep(500);
	COMx = Usart.Open_Com(comStr, COMx);
	memset(data_buff, 0, sizeof(data_buff));
	Usart.Get_Com_State(115200, COMx, DCBx);
	Sleep(500);
	Usart.WriteData(gOpen_Laser, 3, COMx); //�򿪼���
	Sleep(100);
	while (!Usart.Read_Data(data_buff, 8, COMx))////�ȴ����ڷ��ص�����  ��ʱ��ǿ���˳�
	{
		i++;
		Sleep(100);
		if (i >= 10)
		{
			i = 0; break;
		}
	}
	if (data_buff[3] == 0x30 && data_buff[4] == 0x30)
	{
		flag |= 0x02;  //�޸Ĳ����ʳɹ�
		cout << "   URG04LX �����״��ʼ���ɹ�����   " << endl;
	}

	if (LASERx == 1) { hCOM3 = COMx; DCB_COM3 = DCBx; }
	if (LASERx == 2) { hCOM9 = COMx; DCB_COM9 = DCBx; }

}

/******************************************************************************************
									���⴫������ȡ���ݺ���
							Input 1�� start_step ��ʼ��
							Input 2�� end_step   ��ֹ��
							Input 3�� data_buff  Ҫ�洢���ݵ����黺����
*******************************************************************************************/
bool Laser_Sensor::Obtain_Data(int start_step, int end_step, unsigned char *data_buff, int LASERx)
{
	unsigned char send_order[13] = { 0 };
	int data_length = 0, recrive_data_count = 0, data_block = 0, data_remain = 0;
	int count = 0;
	data_length = (end_step - start_step + 1) * 2;  //bytes
	data_block = data_length / 64;
	data_remain = data_length % 64;

	if (data_block == 0)  //����<64��
	{
		data_length += 26;
	}
	else  //����>=64
	{
		if (data_remain == 0)  //������64�ı���
		{
			data_length += 23 + data_block * 2 + 1;
		}
		else //���ݶ��� 64 �ı���������ʣ��
		{
			data_length += (data_block + 1) * 2 + 24;
		}
	}
	send_order[0] = 0x47;
	send_order[1] = 0x53;
	send_order[2] = start_step / 1000 + 0x30; start_step %= 1000;
	send_order[3] = start_step / 100 + 0x30;  start_step %= 100;
	send_order[4] = start_step / 10 + 0x30;   start_step %= 10;
	send_order[5] = start_step + 0x30;

	send_order[6] = end_step / 1000 + 0x30; end_step %= 1000;
	send_order[7] = end_step / 100 + 0x30;  end_step %= 100;
	send_order[8] = end_step / 10 + 0x30;   end_step %= 10;
	send_order[9] = end_step + 0x30;

	send_order[10] = 0x30;
	send_order[11] = 0x31;
	send_order[12] = 0x0A;
	//Usart.WriteData(send_order, 13);
	if (LASERx == 1)  //  ǰ������
	{
		Usart.WriteData(send_order, 13, hCOM3);
		while (!Usart.Read_Data(data_buff, data_length, hCOM3));
	}
	if (LASERx == 2)  //  �󷽼���
	{
		Usart.WriteData(send_order, 13, hCOM9);
		while (!Usart.Read_Data(data_buff, data_length, hCOM9));
	}

	while (data_buff[count])  //�����յ����ݵĳ���
	{
		count++;
	}
	if (count == data_length) //����������//�����������ȡ��ȷ
	{
		return TRUE;
	}
	else
	{
		cout << "   ��ȡ����ʧ�ܣ����ݳ��ȳ��ִ��󣡣�   " << endl;
		return FALSE;
	}
}
///******************************************************************************************
//                           ���⴫������������ֵ����ȡȥ��
//             Input 1�� start_step ��ȡԭʼ���⴫�������ݶ�ά���ݵ�����
//*******************************************************************************************/
//void Laser_Sensor::Dist_Angle_Calculate(int(*data_array)[2], float *dis_angle)
//{
//	static float lsAngle_L = 0, lsAngle_R = 0;
//	static float lsDis_L = 0, lsDis_R = 0;
//	unsigned char finish_flag = 0;
//	for (int m = 0; m < DATA_TOTLE; m++)
//	{
//		if (data_array[m][1]>LASER_SCAN_RANGE || data_array[m][1]<100)   //100 ΪżȻ���ֵ�����  ��ifѭ�����ڶԼ��⴫����������ȥ��
//		{
//			data_array[m][1] = 0;
//		}
//		//��ȡ�ҽŵĽǶ���Ϣ
//		if (!(finish_flag & 0x01))  // ����ҽŵ���Ϣû��ȡ��� ������ȡ
//		{
//	        if (data_array[m][1] != 0 && data_array[m + 1][1] != 0 && data_array[m + 2][1] != 0 && data_array[m + 3][1] != 0 && data_array[m + 4][1] != 0 && data_array[m + 5][1] != 0 && data_array[m + 6][1] != 0)
//			{	
//				lsAngle_R = -120 + (data_array[m][0] + 3 - 44)*LASER_KA;//��m��ȡ���м��ֵ
//				lsDis_R = (data_array[m][1] + data_array[m + 1][1] + data_array[m + 2][1] + data_array[m + 3][1] + data_array[m + 4][1] + data_array[m + 5][1] + data_array[m + 6][1]) / 7.0;
//				finish_flag |= 0x01;	//��ʾ�ҽŵ���Ϣ��ȡ���
//			}	
//		}
//		
//		//����ŵĽǶ���Ϣ
//		if (!(finish_flag & 0x02))  // �����ŵ���Ϣû��ȡ��� ������ȡ
//		{
//			if (data_array[DATA_TOTLE - m][1] != 0 && data_array[DATA_TOTLE - m - 1][1] != 0 && data_array[DATA_TOTLE - m - 2][1] != 0 && data_array[DATA_TOTLE - m - 3][1] != 0 && data_array[DATA_TOTLE - m - 4][1] != 0 && data_array[DATA_TOTLE - m - 5][1] != 0 && data_array[DATA_TOTLE - m - 6][1] != 0)
//			{
//				
//				lsAngle_L = -120 + ( data_array[DATA_TOTLE - m][0]- 3 - 44)*LASER_KA;//��m��ȡ���м��ֵ
//				lsDis_L = (data_array[DATA_TOTLE - m][1] + data_array[DATA_TOTLE - m - 1][1] + data_array[DATA_TOTLE - m - 2][1] + data_array[DATA_TOTLE - m - 3][1] + data_array[DATA_TOTLE - m - 4][1] + data_array[DATA_TOTLE - m - 5][1] + data_array[DATA_TOTLE - m - 6][1]) / 7.0;			
//				finish_flag |= 0x02;   //��ʾ��ŵ���Ϣ��ȡ���
//			}
//		}
//		if (finish_flag == 0x03)  //�ǶȺ;��� ������ȡ���
//		{
//			if (lsAngle_R >= -90 && lsAngle_R <= 90)  //��ȡ�ҽŵľ���
//			{
//				lsDis_R = lsDis_R * cos(lsAngle_R*3.14 / 180);
//			}
//			/*else
//			{
//				lsDis_R = lsDis_R * sin(abs((lsAngle_R - 90)*3.14 / 180));
//			}*/
//			
//			if (lsAngle_L >= -90 && lsAngle_L <= 90)  //��ȡ��ŵľ���
//			{
//				lsDis_L = lsDis_L * cos(lsAngle_L*3.14 / 180);
//			}
//			/*else
//			{
//				lsDis_L = lsDis_L * sin(abs((lsAngle_L + 90)*3.14 / 180));
//			}*/
//		      break; //���ҽ���ȡ������˳�
//		}
//	}
//
//	cout << "Distance_L: " << lsDis_L << "     " << "Distance_R: " << lsDis_R << endl;
//	cout << "Angle_L: " << lsAngle_L << "     " << "Angle_R: " << lsAngle_R << endl<<endl;
//
//	dis_angle[0] = (lsDis_L + lsDis_R) / 2;        //[0] Ϊ ����
//	dis_angle[1] = (lsAngle_L + lsAngle_R) / 2;   //[1] Ϊ �Ƕ�
//}


/******************************************************************************************
���⴫������������ֵ����ȡȥ��
Input 1�� start_step ��ȡԭʼ���⴫�������ݶ�ά���ݵ�����
*******************************************************************************************/
void Laser_Sensor::Dist_Angle_Calculate(int(*data_array)[2], float *dis_angle)
{
	static float lsAngle_L = 0, lsAngle_R = 0;
	static float lsDis_L = 0, lsDis_R = 0;
	unsigned char finish_flag = 0;
	for (int m = 0; m < DATA_TOTLE; m++)
	{
		if (data_array[m][1] > LASER_SCAN_RANGE || data_array[m][1] < 100)   //100 ΪżȻ���ֵ�����  ��ifѭ�����ڶԼ��⴫����������ȥ��
		{
			data_array[m][1] = 0;
		}
	}

	for (int m = 0; m < DATA_TOTLE; m++)//��ȡ�ҽŵĽǶ���Ϣ
	{

		if (data_array[m][1] != 0 && data_array[m + 1][1] != 0 && data_array[m + 2][1] != 0 && data_array[m + 3][1] != 0 && data_array[m + 4][1] != 0 && data_array[m + 5][1] != 0 && data_array[m + 6][1] != 0)
		{
			lsAngle_R = -120 + (data_array[m][0] + 3 - 44)*LASER_KA;//��m��ȡ���м��ֵ
			lsDis_R = (data_array[m][1] + data_array[m + 1][1] + data_array[m + 2][1] + data_array[m + 3][1] + data_array[m + 4][1] + data_array[m + 5][1] + data_array[m + 6][1]) / 7.0;

			break;
		}
	}

	for (int m = DATA_TOTLE; m > 15; m--)////����ŵĽǶ���Ϣ
	{
		if (data_array[m][1] != 0 && data_array[m - 1][1] != 0 && data_array[m - 2][1] != 0 && data_array[m - 3][1] != 0 && data_array[m - 4][1] != 0 && data_array[m - 5][1] != 0 && data_array[m - 6][1] != 0)
		{

			lsAngle_L = -120 + (data_array[m][0] - 3 - 44)*LASER_KA;//��m��ȡ���м��ֵ
			lsDis_L = (data_array[m][1] + data_array[m - 1][1] + data_array[m - 2][1] + data_array[m - 3][1] + data_array[m - 4][1] + data_array[m - 5][1] + data_array[m - 6][1]) / 7.0;
			break;
		}

	}
	///////////////////// get vertical distance ////////////////////////////////
	if (lsAngle_R >= -90 && lsAngle_R <= 90)  //��ȡ�ҽŵľ���
	{
		lsDis_R = lsDis_R * cos(lsAngle_R*3.14 / 180);
	}

	if (lsAngle_L >= -90 && lsAngle_L <= 90)  //��ȡ��ŵľ���
	{
		lsDis_L = lsDis_L * cos(lsAngle_L*3.14 / 180);
	}
	//////////////////////////////////////////////////////////////////////////////

	dis_angle[0] = (lsDis_L + lsDis_R) / 2000;        //[0] Ϊ ���� ��λ ��
	dis_angle[1] = ((lsAngle_L + lsAngle_R) / 2);     //[1] ��λΪ�Ƕ� ��
	//dis_angle[1] = RAD_RATIO * ((lsAngle_L + lsAngle_R) / 2);   //[1] ��λΪ rad/s
}

/******************************************************************************************
				   ���⴫�������������TXT�ĵ�
	   Input 1�� �������Ϊԭʼ���⴫�������ݶ�ά����
*******************************************************************************************/
void Laser_Sensor::LasorDataOut(vector<vector<int>> data, int LASERx)
{
	if (LASERx == 1)
	{
		ofstream outfile("10Laser1.txt", ios::app);  // �ļ�
		//cout << "Lasor 2 sensor data is Geting..." << endl;
		for (int i = 0; i < DATA_TOTLE; i++)
		{
			outfile << data[i][0] << setw(5) << data[i][1] << endl;
		}
		outfile.close();
	}
	if (LASERx == 2)
	{
		ofstream outfile("10Laser2.txt", ios::app);  // �ļ�
		//cout << "Lasor 1 sensor data is Geting..." << endl;
		for (int i = 0; i < DATA_TOTLE; i++)
		{
			outfile << data[i][0] << setw(5) << data[i][1] << endl;
		}
		outfile.close();
	}
}
