#include "Robot_Process.h"
#include <math.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <windows.h>
#include <cmath>  
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
#include <functional> 
#include <Eigen/Dense> 
#include <map>
using namespace Eigen;
using namespace std;

Robot Robot_Toilet;
using namespace std;

HANDLE hCOM6;  //test
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
		speed = speed * SPEED2ORDER_RADIO_TOILET;
		dot = dot * DOT_RADIO_TOILET;
		//dot = dot * 0.1;
	}
	if (ROBOT == 1)
	{
		speed = speed * SPEED2ORDER_RADIO_CHAIR;
		dot = dot * DOT_RADIO_CHAIR;
	}



#if (ROBOT==1)

	v[0] = speed * cos(rad - QUARTER_PI) + 0.425*dot;;
	v[1] = -speed * sin(rad - QUARTER_PI) + 0.425*dot;;
	v[2] = -speed * sin(rad + QUARTER_PI) + 0.425*dot;;
	v[3] = speed * sin(rad - QUARTER_PI) + 0.425*dot;;

#endif

#if (ROBOT==2)

	v[0] = speed * cos(rad) + 0.15*dot;
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
			temp_speed[i] = -temp_speed[i];
		}

		Speed_Up_4bit = (0xf0 & temp_speed[i]) >> 4;
		Speed_Low_4bit = 0x0f & temp_speed[i];

		switch (i)
		{
		case 0: {   //FL
			if (ROBOT == 1)
			{
				data_send[1] = 0x30 | en | (dir << 1);
				data_send[2] = 0x30 | Speed_Up_4bit;
				data_send[3] = 0x30 | Speed_Low_4bit;
				Speed_Up_4bit = 0;
				Speed_Low_4bit = 0;
			}
			if (ROBOT == 2)
			{ 

				data_send[1] = 0x30;
				data_send[2] = 0x30; 
				data_send[3] = 0x30; 

				data_send[4] = 0x30 | en | (dir << 1);//    FF
				data_send[5] = 0x30 | Speed_Up_4bit;
				data_send[6] = 0x30 | Speed_Low_4bit;
				Speed_Up_4bit = 0;
				Speed_Low_4bit = 0;
			}
		}; break;
		case 1: {  //FR
			if (ROBOT == 1)
			{
				data_send[4] = 0x30 | en | (dir << 1);
				data_send[5] = 0x30 | Speed_Up_4bit;
				data_send[6] = 0x30 | Speed_Low_4bit;
				Speed_Up_4bit = 0;
				Speed_Low_4bit = 0;
			}
			if (ROBOT == 2)
			{
				data_send[10] = 0x30 | en | (dir << 1);
				data_send[11] = 0x30 | Speed_Up_4bit;
				data_send[12] = 0x30 | Speed_Low_4bit;
				Speed_Up_4bit = 0;
				Speed_Low_4bit = 0;
			}

		}; break;
		case 2: {   //BR
			if (ROBOT == 1)
			{
				data_send[10] = 0x30 | en | (dir << 1);
				data_send[11] = 0x30 | Speed_Up_4bit;
				data_send[12] = 0x30 | Speed_Low_4bit;
				Speed_Up_4bit = 0;
				Speed_Low_4bit = 0;
			}
			if (ROBOT == 2)
			{
				data_send[7] = 0x30 | en | (dir << 1);
				data_send[8] = 0x30 | Speed_Up_4bit;
				data_send[9] = 0x30 | Speed_Low_4bit;
				Speed_Up_4bit = 0;
				Speed_Low_4bit = 0;
			}

		}; break;
		case 3: {   //BL
			if (ROBOT == 1)
			{
				data_send[7] = 0x30 | en | (dir << 1);
				data_send[8] = 0x30 | Speed_Up_4bit;
				data_send[9] = 0x30 | Speed_Low_4bit;
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

	for (i = 0; i < 17; i++)//校验
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
	float d_min = 0, d_max = 0;
	d_min = real_d - delt_d;
	d_max = real_d + delt_d;
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
	unsigned char data_order[21] = { 0 };
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
	if (KEY_DOWN('Z'))//CCW逆时针
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
		Usart.WriteData(data_order, 21, hCOM5);
	}
	Sleep(10);
	return key_flag;
}

/******************************************************************************************
							机器人动力学子函数，PID控制器，反馈系统
							input ：x轴上速度的分量，y轴上速度的分量，theta角速度
							output ：四个轮的驱动力
*******************************************************************************************/

extern float beforeEdeltaVx = 0;
extern float beforeEdeltaVy = 0;
extern float beforeEdeltaVtheta = 0;
extern int timeCount = 0;
extern float fx = 0;
extern float fy = 0;
extern float ftheta = 0;
MatrixXd Robot::PID_controler(float vx, float vy, float vtheta, MatrixXd A) {

	timeCount++;//记录定时器运行次数，五次一秒
	cout << timeCount << endl;

	float deltaVx;//与反馈x轴方向的差
	float deltaVy;//与反馈y轴方向的差
	float deltaVtheta;//与反馈角速度的差

	deltaVx = vx - A(0, 0);
	deltaVy = vy - A(1, 0);
	deltaVtheta = vtheta - A(2, 0);


	float kp = 0.015;//比例系数
	float kd = 0.02;//微分系数
	float kp2 = 0.015;//比例系数
	float kd2 = 0.02;//微分系数
	float kp3 = 0.015;//比例系数
	float kd3 = 0.02;//微分系数

	 //控制器输出方程，三个输出量
	fx = kp * deltaVx + kd * (deltaVx - beforeEdeltaVx);
	fy = kp2 * deltaVy + kd2 * (deltaVy - beforeEdeltaVy);
	ftheta = kp3 * deltaVtheta + kd3 * (deltaVtheta - beforeEdeltaVtheta);


	beforeEdeltaVx = deltaVx;
	beforeEdeltaVy = deltaVy;
	deltaVtheta = deltaVtheta;

	//计算theta角
	float theta = vtheta * timeCount;
	//运动学转换矩阵
	MatrixXd P(4, 3);
	P(0, 0) = -sin(theta);
	P(0, 1) = cos(theta);
	P(0, 2) = 0.7;
	P(1, 0) = cos(theta);
	P(1, 1) = sin(theta);
	P(1, 2) = -0.7;
	P(2, 0) = -sin(theta);
	P(2, 1) = cos(theta);
	P(2, 2) = -0.7;
	P(3, 0) = cos(theta);
	P(3, 1) = sin(theta);
	P(3, 2) = 0.7;
	//动力学力量分量矩阵
	MatrixXd Fxytheta(3, 1);
	Fxytheta(0, 0) = fx;
	Fxytheta(1, 0) = fy;
	Fxytheta(2, 0) = fy;
	//动力学力量分量矩阵
	MatrixXd F(4, 1);
	F = P * Fxytheta;

	//建立结果矩阵
	MatrixXd RES(4, 2);
	//下面四行装入正向结果的四个力
	RES(0, 0) = F(0, 0);
	RES(1, 0) = F(1, 0);
	RES(2, 0) = F(2, 0);
	RES(3, 0) = F(3, 0);
	//下面在结果集矩阵的第一行二列装入theta角
	RES(0, 1) = theta;
	//下面装入vx的delta
	RES(1, 1) = deltaVx;
	//下面装入vy的delta
	RES(2, 1) = deltaVy;
	//下面装入theta的delta
	RES(3, 1) = deltaVtheta;
	return RES;
}

/******************************************************************************************
							机器人动力学子函数，根据四个轮驱动力得到实际机器人的目标速度与角速度
							input ：四个轮驱动力
							output ：四个轮速度
*******************************************************************************************/
extern int timeTemp = 5;
extern int secondTime = 1; 
extern float vxSpeed = 0;
extern float vySpeed = 0;
extern float thetaSpeed = 0;
MatrixXd  Robot::Power_Cov_Speed(float power[4], float theta, float alafa) {
	float Fg1 = 0;
	float Fg2 = 0;
	float Fg3 = 0;
	float Fg4 = 0;
	float fg1234 = 0;

	//动力学公式当中的大F矩阵
	MatrixXd F(5, 1);
	F(0, 0) = power[0] + Fg1;
	F(1, 0) = power[1] + Fg2;
	F(2, 0) = power[2] + Fg3;
	F(3, 0) = power[3] + Fg4;
	F(4, 0) = fg1234;

	//动力学当中的大B矩阵
	MatrixXd B(3, 5);
	B(0, 0) = -sin(theta);
	B(0, 1) = cos(theta);
	B(0, 2) = -sin(theta);
	B(0, 3) = cos(theta);
	B(0, 4) = -cos(alafa);

	B(1, 0) = cos(theta);
	B(1, 1) = sin(theta);
	B(1, 2) = cos(theta);
	B(1, 3) = sin(theta);
	B(1, 4) = -sin(alafa);

	B(2, 0) = 0.7;
	B(2, 1) = -0.7;
	B(2, 2) = -0.7;
	B(2, 3) = 0.7;
	B(2, 4) = 0;

	//动力学当中的大G矩阵
	MatrixXd G(3,3);
	G(0, 0) = 100;//kg为单位
	G(0, 1) = 0;
	G(0, 2) = 0;

	G(1, 0) = 0;
	G(1, 1) = 100;//同上
	G(1, 2) = 0;

	G(2, 0) = 0;
	G(2, 1) = 0;
	G(2, 2) = 10;//转动惯量暂时设为10

	MatrixXd G_inverse = G.inverse();//求G矩阵的逆矩阵

	//描述矩阵公式Gq=BF  q=G.inverse()BF
	MatrixXd q(3,1);
	q = G.inverse()*B*F; 

	//运动学方程中速度矩阵
	timeTemp++;
	MatrixXd V3(3, 1);
	
	vxSpeed = vxSpeed + q(0, 0);
	vySpeed= vySpeed + q(1, 0);
	thetaSpeed = thetaSpeed + q(2, 0);
	V3(0, 0) = vxSpeed;
	V3(1, 0) = vySpeed;
	V3(2, 0) = thetaSpeed;

	//运动学方程V=PA
	MatrixXd P(4, 3);
	P(0, 0) = -sin(theta);
	P(0, 1) = cos(theta);
	P(0, 2) = 0.7;
	P(1, 0) = cos(theta);
	P(1, 1) = sin(theta);
	P(1, 2) = -0.7;
	P(2, 0) = -sin(theta);
	P(2, 1) = cos(theta);
	P(2, 2) = -0.7;
	P(3, 0) = cos(theta);
	P(3, 1) = sin(theta);
	P(3, 2) = 0.7;

	MatrixXd V4(4, 1);
	V4 = P * V3;

	//建立结果矩阵
	MatrixXd RES(4, 2);
	RES(0, 0) = V4(0,0);
	RES(1, 0) = V4(1, 0);
	RES(2, 0) = V4(2, 0);
	RES(3, 0) = V4(3, 0);

	RES(0,1) = V3(0, 0);
	RES(1, 1) = V3(1, 0);
	RES(2, 1) = V3(2, 0);

	return RES;
}
 
