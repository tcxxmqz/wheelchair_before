#include "Lasor.h"
#include <iostream>
#include <string.h>
#include <windows.h>
#include "Usart.h"
#include <vector>
#include <math.h>
using namespace std;
vector<vector<int>> gOriginal_data1;//用于存放激光雷达解析后的数据  行存放对应的点  列存放点所对应的距离（mm）    C1:第 n 点  C2:点->距离
vector<vector<int>> gOriginal_data2;//用于存放激光雷达解析后的数据  行存放对应的点  列存放点所对应的距离（mm）    C1:第 n 点  C2:点->距离
float gDis_Angle[2] = { 0 };
unsigned char gLaser_Str_Buff1[LASER_DATA_BUFF_SIZE] = { 0 }; // 下方激光雷达缓存区
unsigned char gLaser_Str_Buff2[LASER_DATA_BUFF_SIZE] = { 0 }; // 上方激光雷达缓存区
Laser_Sensor URG04LX;

/******************************************************************************************
获取下方激光传感器数据
Input 1：是否输出到文件
Return ：激光原始数据  n * 2 的容器
*******************************************************************************************/
vector<vector<int>> Laser_Sensor::getLaserDownData(bool OUTFILE)
{
	vector<vector<int>>  tempdata(0);
	while (!URG04LX.Obtain_Data(START_STEP, ENDING_STEP, gLaser_Str_Buff1, 1));//获取下方激光传感器数据
	tempdata = URG04LX.Data_Extract(gLaser_Str_Buff1);
	if (OUTFILE == true)URG04LX.LasorDataOut(tempdata, 1);
	if (!tempdata.empty())
	{
		return tempdata;
	}
}

/******************************************************************************************
						获取上方激光传感器数据
						Input 1：是否输出到文件
						Return ：激光原始数据  n * 2 的容器
*******************************************************************************************/
vector<vector<int>> Laser_Sensor::getLaserUpData(bool OUTFILE)
{
	vector<vector<int>>  tempdata(0);
	while (!URG04LX.Obtain_Data(START_STEP, ENDING_STEP, gLaser_Str_Buff2, 2));//获取下方激光传感器数据
	tempdata = URG04LX.Data_Extract(gLaser_Str_Buff2);
	if (OUTFILE == true)URG04LX.LasorDataOut(tempdata, 2);
	if (!tempdata.empty())
	{
		return tempdata;
	}
}
/******************************************************************************************
						上方激光数据处理，背部中心
						Input 1：已数值化的激光原始数据
						Return ：背部的位置信息 （X Y θ） 坐标与背部偏离的角度
*******************************************************************************************/

#define S_O_SEP_THRESHOLD 500  //上方激光物体分割阈值
#define S_START_ANGLE -30      // 上方激光的检测开始角度  
#define S_END_ANGLE 30         //上方激光的检测终止角度

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

			int ocount = 0;  //这个物体点的计数
			vector < vector<int> > tem_laser;
			int N = i - ip;
			tem_laser.resize(N);//ip为上一次的i值
			for (int m = 0; m < N; ++m)
			{
				tem_laser[m].resize(2);
			}

			for (int j = ip + 1; j < i + 1; j++) // 如果遇到两步之间的跳变 则把这段数据存储到一个临时buff 直接送到腿部检测函数
			{
				if (up_laser[j][1] >= 0)
				{
					tem_laser[ocount][0] = up_laser[j][0]; //step   // 此处赋值有错误
					tem_laser[ocount][1] = up_laser[j][1]; //dis
					ocount++;
					if (j == 424)break;
				}
				else
				{
					tem_laser.clear();
				}

			}
			BackFind(tem_laser, tem_back);// 送入腿部检测
			if (tem_back[0] != 0 && tem_back[1] != 0 && tem_back[2] != 0)
			{
				back.push_back(tem_back[0]);    // x坐标
				back.push_back(tem_back[1]);    // y坐标
				back.push_back(tem_back[2]);    // R半径
				tem_back[0] = 0;
				tem_back[1] = 0;
				tem_back[2] = 0;
			}
			ip = i;        //记录上一次i值
		}
	}
}
/******************************************************************************************
			上方物体中心位置提取
			Input 1：已分离的物体数据
			Return ：返回位置信息 （X Y θ）
*******************************************************************************************/
// 分割物体后这个物体太大或太小都舍去
// 这个物体所占的角度
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
		// 把这个物体全转化为直角坐标的点
		for (int i = 0; i < N; i++)
		{
			sum_dis += sub_object[i][1];
			tem_x[i] = (float)(sub_object[i][1] * cos((90 + (sub_object[i][0] - 385) * LASER_KA) *0.0175));
			tem_y[i] = (float)(sub_object[i][1] * sin((90 + (sub_object[i][0] - 385) * LASER_KA) *0.0175));
			sumx += tem_x[i];
			sumy += tem_y[i];
		}
		// 距离平均值  最大角度即为 N 的计数，即角的宽度
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
							下方激光数据处理，寻找腿部
							Input 1：已数值化的激光原始数据
							Return ：腿部的位置及大小信息
*******************************************************************************************/
#define X_O_SEP_THRESHOLD 40  //下方激光物体分割阈值
void Laser_Sensor::DownLaserDataProc(vector<vector<int>>down_laser, vector<float> &foot)
{
	float tem_foot[3] = { 0 };
	int tem_step = 0, tem_dis = 0, ip = 0;
	for (int i = 0; i < STEP - 1; i++)
	{
		if (abs(down_laser[i + 1][1] - down_laser[i][1]) >= X_O_SEP_THRESHOLD)
		{

			int ocount = 0;  //这个物体点的计数
			vector < vector<int> > tem_laser;
			int N = i - ip;
			tem_laser.resize(N);//ip为上一次的i值
			for (int m = 0; m < N; ++m)
			{
				tem_laser[m].resize(2);
			}

			for (int j = ip + 1; j < i + 1; j++) // 如果遇到两步之间的跳变 则把这段数据存储到一个临时buff 直接送到腿部检测函数
			{
				if (down_laser[j][1] > 20)
				{
					tem_laser[ocount][0] = down_laser[j][0]; //step   // 此处赋值有错误
					tem_laser[ocount][1] = down_laser[j][1]; //dis
					ocount++;
				}
				else
				{
					tem_laser.clear();
				}

			}
			FootFind(tem_laser, tem_foot);// 送入腿部检测
			if (tem_foot[0] != 0 && tem_foot[1] != 0 && tem_foot[2] != 0)
			{
				foot.push_back(tem_foot[0]);    // x坐标
				foot.push_back(tem_foot[1]);    // y坐标
				foot.push_back(tem_foot[2]);    // R半径
				tem_foot[0] = 0;
				tem_foot[1] = 0;
				tem_foot[2] = 0;
			}

			ip = i;        //记录上一次i值
		}
	}
}
/******************************************************************************************
					对分割后的物体进行腿部类圆检测
					Input 1：分割好的物体
					Return ：类圆的 半径 和 中心（x, y）
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
	// 分割后的物体若小于5个点 则直接返回
	if (N <= 10)
	{

		return;
	}
	else
	{
		//只取这个物体上的三个点进行腿部检测。
		float x1 = 0, x2 = 0, x4 = 0, Xm = 0, Xn = 0, X = 0;
		float y1 = 0, y2 = 0, y4 = 0, Ym = 0, Yn = 0, Y = 0;
		float k1 = 0, k2 = 0, k14 = 0, d14 = 0, D = 0, R = 0;
		// 取这个物体的中点
		int mid = N / 2;
		float sum_x = 0, sum_y = 0;
		// 把这个物体全转化为直角坐标的点
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
		// 两条内切弦垂线的斜率
		k1 = -(x2 - x1) / (y2 - y1);
		k2 = -(x4 - x2) / (y4 - y2);
		// k1 所在弦的中点
		Xm = (x1 + x2) / 2; Ym = (y1 + y2) / 2;
		// k2 所在弦的中点
		Xn = (x4 + x2) / 2; Yn = (y4 + y2) / 2;
		// P1与P4点的距离
		d14 = sqrt(pow(y4 - y1, 2) + pow((x4 - x1), 2));
		// P1 P4 所在直线的斜率
		k14 = (y4 - y1) / (x4 - x1);
		//P1 P4 弧中点与线段P1P4 的距离
		D = abs((-k14 * x2 + y2 + k14 * x1 - y1) / (sqrt(k14 * k14 + 1)));
		X = (k1*Xm - k2 * Xn + Yn - Ym) / (k1 - k2);
		Y = k1 * (X - Xm) + Ym;
		R = (float)(sqrt(pow((X - x1), 2) + pow((Y - y1), 2)) + sqrt(pow((X - x2), 2) + pow((Y - y2), 2)) + sqrt(pow((X - x4), 2) + pow((Y - y4), 2))) / 3.0;
		//计算端点连成的直线, 原点坐标（x, y）满足 Ax + By + C>0
		float	A = -(y4 - y1), B = x4 - x1, C = x1 * (y4 - y1) - y1 * (x4 - x1);
		bool tempTF = false;
		if (A*X + B * Y + C <= 0)tempTF = true;
		// 判断是否符合脚大小的圆  D < 的值越大，越能检测出曲率较小的物体
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
					激光传感器字符数据解析
					Input 1：待解析字符串
					Return ：激光原始数据  n * 2 的容器
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
	int k = 1000, count = 0;//count用来计数，存储数据到最终的数组
	int data_length = 0, temp = 0, p = 0, q = 0;
	int temp_sum = 0, data_temp_high = 0, data_temp_low = 0, data;
	//length = strlen(data_str);//其中有26 bytes 固定长度的数据  其余 为激光传感器数据
	while (data_str[count])  //计算收到数据的长度
	{
		count++;
	}
	length = count;
	count = 0;
	//cout << "数据点数为：" << length << " byte" << endl;
	data_str[length] = 0x00;
	if (length <= 1450)  //当数据个数 >= 64bytes 时处理程序  
	{
		if (data_str[0] == 'G' && data_str[1] == 'S' && data_str[length - 1] == 0x0A && data_str[length - 2] == 0x0A)
		{
			//************此 for 循环 用于提取 起始步 与 结束步***************//
			for (int i = 0; i < 8; i++)
			{
				if (i >= 0 && i <= 3)
				{
					step_temp[i] = (data_str[i + 2] - 0x30) * k;
					k = (int)(k / 10);
					starting_step += step_temp[i];//获取第开始步
				}
				else
				{
					if (i == 4) k = 1000;
					step_temp[i] = (data_str[i + 2] - 0x30) * k;
					k = (int)(k / 10);
					ending_step += step_temp[i];//获取第结束步
				}
			}
			/* cout << "数据长度为：" << ending_step - starting_step + 1 << " 个,"
				  << starting_step << " ~ " << ending_step << endl << endl;*/
			if (data_str[22] == 0x0A)//它的后面就是数据
			{
				int j = 0, i = 0, m = 0;
				data_length = ending_step - starting_step + 1;// 66点 
				data_byte = data_length * 2;
				data_block = data_byte / 64;  //数据块数  64字节为一块
				data_remain = data_byte % 64; //剩余数据所占字节
	//******************** 数据块 处理********************************//
				for (i = 0; i < data_block; i++)  //数据块数
				{
					temp_sum = 0;
					for (j = 0; j < 64; j++)      //每一块数据个数 64 
					{
						temp_sum += data_str[23 + i * 66 + j];
					}
					temp_sum = temp_sum & 0x3F;
					temp_sum = temp_sum + 0x30; //取总和的低六位 即校验和
					p = 22 + (i + 1) * 66 - 1;  //为 sum 的坐标
					temp = data_str[p];
					/*if (i == 0){ p = 23 + i * 64 + j; temp = data_str[p]; }
					else { q = 23 + i * 64 + j + 2; temp = data_str[q]; }*/
					if (temp_sum == temp)//计算校验和 与 所接受的校验和是否相等
					{                                             //开始提取数据
						for (m = 0; m < 64; m = m + 2)      //每一块数据个数 64 
						{
							data_temp_high = data_str[23 + i * 66 + m] - 0x30;
							data_temp_high = data_temp_high & 0x3F;//取数据的高六位
							data_temp_high = data_temp_high << 6;
							data_temp_low = data_str[23 + i * 66 + m + 1] - 0x30;//取数据的低六位
							data_temp_low = data_temp_low & 0x3F;
							data = data_temp_high + data_temp_low;//数据合成	
							data_array[count][0] = starting_step + count;  //将数据存储到最终的数组中
							data_array[count][1] = data;
							//cout << data_array[count][0] << endl;
							count++;
						}
					}
				}
				//********************剩余数据处理********************************//
				data_remain_start = data_block * 66 + 23;
				temp_sum = 0;
				for (j = data_remain_start; j < data_remain_start + data_remain; j++)      //每一块数据个数 64 
				{
					temp_sum += data_str[j];
				}
				temp_sum = temp_sum & 0x3F;
				temp_sum = temp_sum + 0x30; //取总和的低六位 即校验和
				if (temp_sum == data_str[data_remain_start + data_remain])//计算校验和 与 所接受的校验和是否相等
				{
					for (int j = data_remain_start; j < data_remain_start + data_remain; j += 2)
					{
						data_temp_high = data_str[j] - 0x30;
						data_temp_high = data_temp_high & 0x3F;//取数据的高六位
						data_temp_high = data_temp_high << 6;
						data_temp_low = data_str[j + 1] - 0x30;//取数据的低六位
						data_temp_low = data_temp_low & 0x3F;
						data = data_temp_high + data_temp_low;//数据合成	
						data_array[count][0] = starting_step + count;  //将数据存储到最终的数组中
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
							  激光传感器的初始化函数
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
	COMx = Usart.Open_Com(comStr, COMx); //打开串口
	Usart.Get_Com_State(19200, COMx, DCBx); //设置波特率为 115200
	Sleep(500);
	Usart.WriteData(gBit_Rate, 9, COMx); //把波特率修改成115200
	Sleep(100);

	while (!Usart.Read_Data(data_buff, 14, COMx))////等待串口返回的数据  超时则强制退出
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
		//flag = 0x01;  //修改波特率成功
	}
	Usart.Close_Com(COMx);
	Sleep(500);
	COMx = Usart.Open_Com(comStr, COMx);
	memset(data_buff, 0, sizeof(data_buff));
	Usart.Get_Com_State(115200, COMx, DCBx);
	Sleep(500);
	Usart.WriteData(gOpen_Laser, 3, COMx); //打开激光
	Sleep(100);
	while (!Usart.Read_Data(data_buff, 8, COMx))////等待串口返回的数据  超时则强制退出
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
		flag |= 0x02;  //修改波特率成功
		cout << "   URG04LX 激光雷达初始化成功！！   " << endl;
	}

	if (LASERx == 1) { hCOM3 = COMx; DCB_COM3 = DCBx; }
	if (LASERx == 2) { hCOM9 = COMx; DCB_COM9 = DCBx; }

}

/******************************************************************************************
									激光传感器获取数据函数
							Input 1： start_step 开始步
							Input 2： end_step   终止步
							Input 3： data_buff  要存储数据的数组缓存区
*******************************************************************************************/
bool Laser_Sensor::Obtain_Data(int start_step, int end_step, unsigned char *data_buff, int LASERx)
{
	unsigned char send_order[13] = { 0 };
	int data_length = 0, recrive_data_count = 0, data_block = 0, data_remain = 0;
	int count = 0;
	data_length = (end_step - start_step + 1) * 2;  //bytes
	data_block = data_length / 64;
	data_remain = data_length % 64;

	if (data_block == 0)  //数据<64个
	{
		data_length += 26;
	}
	else  //数据>=64
	{
		if (data_remain == 0)  //数据是64的倍数
		{
			data_length += 23 + data_block * 2 + 1;
		}
		else //数据多于 64 的倍数并且有剩余
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
	if (LASERx == 1)  //  前方激光
	{
		Usart.WriteData(send_order, 13, hCOM3);
		while (!Usart.Read_Data(data_buff, data_length, hCOM3));
	}
	if (LASERx == 2)  //  后方激光
	{
		Usart.WriteData(send_order, 13, hCOM9);
		while (!Usart.Read_Data(data_buff, data_length, hCOM9));
	}

	while (data_buff[count])  //计算收到数据的长度
	{
		count++;
	}
	if (count == data_length) //如果长度相等//则表明数据收取正确
	{
		return TRUE;
	}
	else
	{
		cout << "   获取数据失败，数据长度出现错误！！   " << endl;
		return FALSE;
	}
}
///******************************************************************************************
//                           激光传感器数据特征值的提取去噪
//             Input 1： start_step 提取原始激光传感器数据二维数据的数组
//*******************************************************************************************/
//void Laser_Sensor::Dist_Angle_Calculate(int(*data_array)[2], float *dis_angle)
//{
//	static float lsAngle_L = 0, lsAngle_R = 0;
//	static float lsDis_L = 0, lsDis_R = 0;
//	unsigned char finish_flag = 0;
//	for (int m = 0; m < DATA_TOTLE; m++)
//	{
//		if (data_array[m][1]>LASER_SCAN_RANGE || data_array[m][1]<100)   //100 为偶然出现的噪声  此if循环用于对激光传感器初步的去噪
//		{
//			data_array[m][1] = 0;
//		}
//		//提取右脚的角度信息
//		if (!(finish_flag & 0x01))  // 如果右脚的信息没提取完成 继续提取
//		{
//	        if (data_array[m][1] != 0 && data_array[m + 1][1] != 0 && data_array[m + 2][1] != 0 && data_array[m + 3][1] != 0 && data_array[m + 4][1] != 0 && data_array[m + 5][1] != 0 && data_array[m + 6][1] != 0)
//			{	
//				lsAngle_R = -120 + (data_array[m][0] + 3 - 44)*LASER_KA;//在m中取最中间的值
//				lsDis_R = (data_array[m][1] + data_array[m + 1][1] + data_array[m + 2][1] + data_array[m + 3][1] + data_array[m + 4][1] + data_array[m + 5][1] + data_array[m + 6][1]) / 7.0;
//				finish_flag |= 0x01;	//表示右脚的信息提取完毕
//			}	
//		}
//		
//		//提左脚的角度信息
//		if (!(finish_flag & 0x02))  // 如果左脚的信息没提取完成 继续提取
//		{
//			if (data_array[DATA_TOTLE - m][1] != 0 && data_array[DATA_TOTLE - m - 1][1] != 0 && data_array[DATA_TOTLE - m - 2][1] != 0 && data_array[DATA_TOTLE - m - 3][1] != 0 && data_array[DATA_TOTLE - m - 4][1] != 0 && data_array[DATA_TOTLE - m - 5][1] != 0 && data_array[DATA_TOTLE - m - 6][1] != 0)
//			{
//				
//				lsAngle_L = -120 + ( data_array[DATA_TOTLE - m][0]- 3 - 44)*LASER_KA;//在m中取最中间的值
//				lsDis_L = (data_array[DATA_TOTLE - m][1] + data_array[DATA_TOTLE - m - 1][1] + data_array[DATA_TOTLE - m - 2][1] + data_array[DATA_TOTLE - m - 3][1] + data_array[DATA_TOTLE - m - 4][1] + data_array[DATA_TOTLE - m - 5][1] + data_array[DATA_TOTLE - m - 6][1]) / 7.0;			
//				finish_flag |= 0x02;   //表示左脚的信息提取完毕
//			}
//		}
//		if (finish_flag == 0x03)  //角度和距离 数据提取完成
//		{
//			if (lsAngle_R >= -90 && lsAngle_R <= 90)  //获取右脚的距离
//			{
//				lsDis_R = lsDis_R * cos(lsAngle_R*3.14 / 180);
//			}
//			/*else
//			{
//				lsDis_R = lsDis_R * sin(abs((lsAngle_R - 90)*3.14 / 180));
//			}*/
//			
//			if (lsAngle_L >= -90 && lsAngle_L <= 90)  //获取左脚的距离
//			{
//				lsDis_L = lsDis_L * cos(lsAngle_L*3.14 / 180);
//			}
//			/*else
//			{
//				lsDis_L = lsDis_L * sin(abs((lsAngle_L + 90)*3.14 / 180));
//			}*/
//		      break; //左右脚提取完成则退出
//		}
//	}
//
//	cout << "Distance_L: " << lsDis_L << "     " << "Distance_R: " << lsDis_R << endl;
//	cout << "Angle_L: " << lsAngle_L << "     " << "Angle_R: " << lsAngle_R << endl<<endl;
//
//	dis_angle[0] = (lsDis_L + lsDis_R) / 2;        //[0] 为 距离
//	dis_angle[1] = (lsAngle_L + lsAngle_R) / 2;   //[1] 为 角度
//}


/******************************************************************************************
激光传感器数据特征值的提取去噪
Input 1： start_step 提取原始激光传感器数据二维数据的数组
*******************************************************************************************/
void Laser_Sensor::Dist_Angle_Calculate(int(*data_array)[2], float *dis_angle)
{
	static float lsAngle_L = 0, lsAngle_R = 0;
	static float lsDis_L = 0, lsDis_R = 0;
	unsigned char finish_flag = 0;
	for (int m = 0; m < DATA_TOTLE; m++)
	{
		if (data_array[m][1] > LASER_SCAN_RANGE || data_array[m][1] < 100)   //100 为偶然出现的噪声  此if循环用于对激光传感器初步的去噪
		{
			data_array[m][1] = 0;
		}
	}

	for (int m = 0; m < DATA_TOTLE; m++)//提取右脚的角度信息
	{

		if (data_array[m][1] != 0 && data_array[m + 1][1] != 0 && data_array[m + 2][1] != 0 && data_array[m + 3][1] != 0 && data_array[m + 4][1] != 0 && data_array[m + 5][1] != 0 && data_array[m + 6][1] != 0)
		{
			lsAngle_R = -120 + (data_array[m][0] + 3 - 44)*LASER_KA;//在m中取最中间的值
			lsDis_R = (data_array[m][1] + data_array[m + 1][1] + data_array[m + 2][1] + data_array[m + 3][1] + data_array[m + 4][1] + data_array[m + 5][1] + data_array[m + 6][1]) / 7.0;

			break;
		}
	}

	for (int m = DATA_TOTLE; m > 15; m--)////提左脚的角度信息
	{
		if (data_array[m][1] != 0 && data_array[m - 1][1] != 0 && data_array[m - 2][1] != 0 && data_array[m - 3][1] != 0 && data_array[m - 4][1] != 0 && data_array[m - 5][1] != 0 && data_array[m - 6][1] != 0)
		{

			lsAngle_L = -120 + (data_array[m][0] - 3 - 44)*LASER_KA;//在m中取最中间的值
			lsDis_L = (data_array[m][1] + data_array[m - 1][1] + data_array[m - 2][1] + data_array[m - 3][1] + data_array[m - 4][1] + data_array[m - 5][1] + data_array[m - 6][1]) / 7.0;
			break;
		}

	}
	///////////////////// get vertical distance ////////////////////////////////
	if (lsAngle_R >= -90 && lsAngle_R <= 90)  //获取右脚的距离
	{
		lsDis_R = lsDis_R * cos(lsAngle_R*3.14 / 180);
	}

	if (lsAngle_L >= -90 && lsAngle_L <= 90)  //获取左脚的距离
	{
		lsDis_L = lsDis_L * cos(lsAngle_L*3.14 / 180);
	}
	//////////////////////////////////////////////////////////////////////////////

	dis_angle[0] = (lsDis_L + lsDis_R) / 2000;        //[0] 为 距离 单位 米
	dis_angle[1] = ((lsAngle_L + lsAngle_R) / 2);     //[1] 单位为角度 °
	//dis_angle[1] = RAD_RATIO * ((lsAngle_L + lsAngle_R) / 2);   //[1] 单位为 rad/s
}

/******************************************************************************************
				   激光传感器数据输出到TXT文档
	   Input 1： 传入参数为原始激光传感器数据二维数组
*******************************************************************************************/
void Laser_Sensor::LasorDataOut(vector<vector<int>> data, int LASERx)
{
	if (LASERx == 1)
	{
		ofstream outfile("10Laser1.txt", ios::app);  // 文件
		//cout << "Lasor 2 sensor data is Geting..." << endl;
		for (int i = 0; i < DATA_TOTLE; i++)
		{
			outfile << data[i][0] << setw(5) << data[i][1] << endl;
		}
		outfile.close();
	}
	if (LASERx == 2)
	{
		ofstream outfile("10Laser2.txt", ios::app);  // 文件
		//cout << "Lasor 1 sensor data is Geting..." << endl;
		for (int i = 0; i < DATA_TOTLE; i++)
		{
			outfile << data[i][0] << setw(5) << data[i][1] << endl;
		}
		outfile.close();
	}
}
