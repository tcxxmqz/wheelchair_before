#include "Robot_Process.h"
#include <math.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <windows.h>

#include "Usart.h"
#include <string.h>
#include <iomanip>
#include <omp.h>
#include "stdlib.h"
#pragma comment(lib, "Winmm.lib") 
#include <mmsystem.h>
#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include <vector>
using namespace std;

Robot Robot_Toilet;
using namespace std;
/******************************************************************************************
						排泄支援机器人压力数据输出到TXT文档
Input 1： 传入参数为压力传感器数据数组 void  PressData2Txt(int *data);
*******************************************************************************************/
void Robot::PressData2Txt(int *data)
{
	ofstream outfile("Pressure.txt", ios::app);  // 文件
	cout << "Pressure sensor data is Outing..." << endl;
	outfile << data[0] << setw(5) << data[1] << setw(5) << data[2] << setw(5) << data[3] << setw(5) << data[4] << setw(5) << data[5] << endl;
	outfile.close();
}

/******************************************************************************************
                                          运动学方程
										 speed： 整体运动速度 （m/s）
										 angle： 运动方向      (°)
										 dot  ： 自旋转角速度  (°/s)
										         dot > 0  CW
												 dot < 0  CCW
										 v：     四个轮子的速度(无量纲)
*******************************************************************************************/
void Robot::Move_Equation(float speed, float angle, float dot, float *v)
{
	float  rad;
	static float  angle_1 = 0;  //用于存储角度的历史值
	rad = angle * RAD_RATIO;  //角度转弧度
	//dot = ( angle_1 - angle )/2;
	//printf("角度为：%f,  角速度为：%f \n", angle, dot);
	angle_1 = angle;
	
	if (ROBOT == 2)
	{
        speed = speed* SPEED2ORDER_RADIO_TOILET;
	    dot = dot * DOT_RADIO_TOILET;
		//dot = dot * 0.1;
	}
	if (ROBOT == 1)
	{
        speed = speed* SPEED2ORDER_RADIO_CHAIR;
		dot = dot * DOT_RADIO_CHAIR;
	}
	
	
	
#if (ROBOT==1)

	v[0] =  speed * cos(rad - QUARTER_PI) + 0.425*dot;;
	v[1] = -speed * sin(rad - QUARTER_PI) + 0.425*dot;;
	v[2] = -speed * sin(rad + QUARTER_PI) + 0.425*dot;;
	v[3] =  speed * sin(rad - QUARTER_PI) + 0.425*dot;;

#endif
	
#if (ROBOT==2)

	v[0] = speed * cos( rad ) + 0.15*dot;
	v[1] = -speed * cos(rad - ONE_THIED_PI) + 0.05*dot;
	v[2] = -speed * cos(rad + ONE_THIED_PI) + 0.05*dot;

#endif

}

/******************************************************************************************
                              速度信息转换成指令
							speed： 四个轮子速度
							data_send： 转换完成的指令
*******************************************************************************************/
void Robot::Data_Cov(float *speed, unsigned char *data_send)
{
	unsigned char Speed_Up_4bit = 0, Speed_Low_4bit = 0, i = 0, FSC = 0, en = 0, dir = 0;

	int temp_speed[4] = { 0 };

	data_send[0] = 0x01;//start code

	for (i = 0; i < 4; i++)
	{
		//temp_speed[i] = (int)(speed[i] * SPEED2ORDER_RADIO);
		temp_speed[i] = (int)(speed[i]);
		if (temp_speed[i] != 0)
			en = 1;
		else
			en = 0;
		if (temp_speed[i] >= 0)
			dir = 0;
		else
		{
            dir = 1;
			temp_speed[i] = - temp_speed[i];
		}
			
		Speed_Up_4bit = (0xf0 & temp_speed[i]) >> 4;
		Speed_Low_4bit = 0x0f & temp_speed[i];
		
		switch (i)
		{
		case 0:{   //FL
			if (ROBOT == 1)
			{
               data_send[1] = 0x30 | en | (dir << 1);//µç»ú·½ÏòÓëÊ¹ÄÜ
			   data_send[2] = 0x30 | Speed_Up_4bit;//µç»úµÄ¸ßËÄÎ»Êý¾Ý
			   data_send[3] = 0x30 | Speed_Low_4bit;//µç»úµÄµÍËÄÎ»Êý¾Ý
			   Speed_Up_4bit = 0;
			   Speed_Low_4bit = 0;
			}
			if (ROBOT == 2)
			{
				data_send[1] = 0x30;
				data_send[2] = 0x30;
				data_send[3] = 0x30;

				data_send[4] = 0x30 | en | (dir << 1);//µç»ú·½ÏòÓëÊ¹ÄÜ     FF
				data_send[5] = 0x30 | Speed_Up_4bit;//µç»úµÄ¸ßËÄÎ»Êý¾Ý
				data_send[6] = 0x30 | Speed_Low_4bit;//µç»úµÄµÍËÄÎ»Êý¾Ý
				Speed_Up_4bit = 0;
				Speed_Low_4bit = 0;			
			}
				}; break;
		case 1:{  //FR
			if (ROBOT == 1)
			{
				data_send[4] = 0x30 | en | (dir << 1);//µç»ú·½ÏòÓëÊ¹ÄÜ
				data_send[5] = 0x30 | Speed_Up_4bit;//µç»úµÄ¸ßËÄÎ»Êý¾Ý
				data_send[6] = 0x30 | Speed_Low_4bit;//µç»úµÄµÍËÄÎ»Êý¾Ý
				Speed_Up_4bit = 0;
				Speed_Low_4bit = 0;
			}
			if (ROBOT == 2)
			{
				data_send[10] = 0x30 | en | (dir << 1);//µç»ú·½ÏòÓëÊ¹ÄÜ
				data_send[11] = 0x30 | Speed_Up_4bit;//µç»úµÄ¸ßËÄÎ»Êý¾Ý
				data_send[12] = 0x30 | Speed_Low_4bit;//µç»úµÄµÍËÄÎ»Êý¾Ý
				Speed_Up_4bit = 0;
				Speed_Low_4bit = 0;
			}
			
				}; break;
		case 2:{   //BR
			if (ROBOT == 1)
			{
                data_send[10] = 0x30 | en | (dir << 1);//µç»ú·½ÏòÓëÊ¹ÄÜ
			    data_send[11] = 0x30 | Speed_Up_4bit;//µç»úµÄ¸ßËÄÎ»Êý¾Ý
			    data_send[12] = 0x30 | Speed_Low_4bit;//µç»úµÄµÍËÄÎ»Êý¾Ý
		    	Speed_Up_4bit = 0;
		    	Speed_Low_4bit = 0;
			}
			if (ROBOT == 2)
			{
				data_send[7] = 0x30 | en | (dir << 1);//µç»ú·½ÏòÓëÊ¹ÄÜ
				data_send[8] = 0x30 | Speed_Up_4bit;//µç»úµÄ¸ßËÄÎ»Êý¾Ý
				data_send[9] = 0x30 | Speed_Low_4bit;//µç»úµÄµÍËÄÎ»Êý¾Ý
				Speed_Up_4bit = 0;
				Speed_Low_4bit = 0;
			}
			
				}; break;
		case 3:{   //BL
			if (ROBOT == 1)
			{
                data_send[7] = 0x30 | en | (dir << 1);//µç»ú·½ÏòÓëÊ¹ÄÜ
				data_send[8] = 0x30 | Speed_Up_4bit;//µç»úµÄ¸ßËÄÎ»Êý¾Ý
				data_send[9] = 0x30 | Speed_Low_4bit;//µç»úµÄµÍËÄÎ»Êý¾Ý
				Speed_Up_4bit = 0;
				Speed_Low_4bit = 0;
			}		
			   }; break;
		}
     }

	data_send[13] = 0x30;   // 若 0x38 获取超声波数据
	if (ROBOT == 2)  // 获取马桶机器人的压力传感器数据
	{
		data_send[14] = 0x3A;  // 0x32: 压力   0x3A : 50ms 返回压力
	}
	else
	{
		data_send[14] = 0x30;  // 轮椅机器人
	}
	

	data_send[15] = 0x30;
	data_send[16] = 0x30;

	for (i = 0; i<17; i++)//校验
	{
		FSC ^= data_send[i];
	}
	data_send[17] = 0x30 | ((FSC & 0xf0) >> 4);
	data_send[18] = 0x30 | (FSC & 0x0f);

	data_send[19] = 0x0d;
	data_send[20] = 0x0a;
}



/******************************************************************************************
							机器人前后运动寻找最优点
							Input1 d:     理想距离（d_D 表中所得）
							Input2 real_d: 机器人距腿实际距离
							Input3 delta_d: 根据奖励得来的机器人运动量
							Input4 v	  : 机器人的运动速度
							Output pressureStr： 为机器人压力数据数组
*******************************************************************************************/
void Robot::Robot_Movement(float d, float real_d, float delt_d, int v, unsigned char *pressStr)
{ 
	unsigned char speed_Order[21] = { 0 };
	float d_min = 0,d_max =  0;
	d_min = real_d - delt_d ; 
	d_max = real_d + delt_d ;
	static char FBFlag = 0; // 前 0 ； 后 1；
	 float speed[4] = { 0 };
	
	if ((d < d_min) || FBFlag == 1)
	{
		FBFlag = 1;
		Robot_Toilet.Move_Equation(0.01, 270, 0, speed);  // Back Movement

	}
	if ((d > d_max) || FBFlag == 0)
	{
		FBFlag = 0;
		Robot_Toilet.Move_Equation(0.01, 90, 0, speed);  // Forward Movement
		
	}
	Robot_Toilet.Data_Cov(speed, speed_Order);
	Usart.WriteData(speed_Order, 21, hCOM6);
	while (!Usart.Read_Data(pressStr, 42, hCOM6));//读取到压力+超声波字符串则为78  压力为：42
}



/******************************************************************************************
									（背）数据输出到文件
*******************************************************************************************/
void Robot::BackData2File(float* goal, char* str)
{
	fstream outfile(str, ios::app);
	outfile << setw(2) << "  "
		<< setw(CHAR_LEN) << setprecision(PRECISION) << goal[0]
		<< setw(CHAR_LEN) << setprecision(PRECISION) << goal[1]
		<< setw(CHAR_LEN) << setprecision(PRECISION) << goal[2] << "\n";
	outfile.close();
}
/******************************************************************************************
						（滤波前后 Speed  Angle ）数据输出到文件
*******************************************************************************************/
void Robot::ASData2File(float* goal, float speed, float angle, char* str)
{
	fstream outfile(str, ios::app);
	outfile << setw(2) << "  "
		<< setw(CHAR_LEN) << setprecision(PRECISION) << goal[0] * 0.000005   // speed
		<< setw(CHAR_LEN) << setprecision(PRECISION) << goal[1]              // angle
		<< setw(CHAR_LEN) << setprecision(PRECISION) << goal[2]              // foot angle
		<< setw(CHAR_LEN) << setprecision(PRECISION) << speed 
		<< setw(CHAR_LEN) << setprecision(PRECISION) << angle << "\n";       // kf angle
	outfile.close();
}

/******************************************************************************************
						两个点的角度转换 0~360° ，取x轴正方向为0°开始
						input1 ：x2-x1        
						input2 : y2-y1    
						return : 角度（弧度制）
*******************************************************************************************/
float Robot::TanAngleConvert(float tempX, float tempY)
{
	float theta = 0;
	if (tempX < 0) // 二三象限
	{
		theta = atan(tempY / tempX) + PI;
	}
		
	if (tempX > 0 && tempY < 0) // 四象限
	{
		theta = atan(tempY / tempX) + 2 * PI;       // 引力方向
	}
    if (tempX > 0 && tempY > 0)
	{
		theta = atan(tempY / tempX);
	}
	return theta;
}

/******************************************************************************************
							电脑键盘控制机器人移动
							input ：运动 speed
*******************************************************************************************/
unsigned char Robot::Key_Control_Robot(float  target_speed)
{
	float speed[4] = { 0 };
	unsigned char data_order[21] = {0};
	unsigned char key_flag = 0;
	if (KEY_DOWN('W'))//前
	{
		Move_Equation(target_speed, 90, 0, speed);
		Data_Cov(speed, data_order);
		key_flag = 1;
	}
	if (KEY_DOWN('S'))//后
	{
		Move_Equation(target_speed, 270, 0, speed);
		Data_Cov(speed, data_order);
		key_flag = 2;
	}
	if (KEY_DOWN('A'))//左
	{
		Move_Equation(target_speed, 180, 0, speed);
		Data_Cov(speed, data_order);
		key_flag = 3;
	}
	if (KEY_DOWN('D'))//右
	{
		Move_Equation(target_speed, 0, 0, speed);
		Data_Cov(speed, data_order);
		key_flag = 4;
	}
	if (KEY_DOWN('Z'))//CCW
	{
		Move_Equation(0, 0, -10, speed);
		Data_Cov(speed, data_order);
		key_flag = 5;	
	}
	if (KEY_DOWN('X'))//CW
	{
		Move_Equation(0, 0, 10, speed);
		Data_Cov(speed, data_order);
		key_flag = 6;
	}
	if (KEY_DOWN('P'))//开始或停止
	{
		key_flag = 7;
	}
	if (key_flag != 0)
	{
		Usart.WriteData(data_order, 21, hCOM6);
	}
	Sleep(10);
	return key_flag;
}


