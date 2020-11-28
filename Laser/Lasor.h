#ifndef _LASER_H_
#define _LASER_H_
#include <iostream>
#include <vector>
using namespace std;
#define STEP 682
#define DIS  2
#define START_STEP 45//132
#define ENDING_STEP 725//640
#define DATA_TOTLE ENDING_STEP-START_STEP+1
#define LASER_KA 0.35
#define LASER_KD 1.07
#define LASER_DATA_BUFF_SIZE 1450 

#define LASER_SCAN_RANGE 1500     //去除范围外的噪声噪
extern vector<vector<int>> gOriginal_data1;//用于存放激光雷达解析后的数据  行存放对应的点  列存放点所对应的距离（mm）    C1:第 n 点  C2:点->距离
extern vector<vector<int>> gOriginal_data2;//用于存放激光雷达解析后的数据  行存放对应的点  列存放点所对应的距离（mm）
extern unsigned char gLaser_Str_Buff1[LASER_DATA_BUFF_SIZE];
extern unsigned char gLaser_Str_Buff2[LASER_DATA_BUFF_SIZE];
extern float gDis_Angle[2];
class Laser_Sensor
{
  public :
	  vector<vector<int>> Data_Extract(unsigned char *data_str);
	  void Laser_Init(int LASERx);
	  bool Obtain_Data(int start_step, int end_step, unsigned char *data_buff, int LASERx);
	  void Dist_Angle_Calculate(int (*data_array)[2], float *dis_angle );          //传入的参数为激光传感器 待去噪 的数据 
	  void LasorDataOut(vector<vector<int>> data, int LASERx);        // 传入参数为激光的原始数据，导出成txt文件。//返回参数为提取的角度和距离的值   
	  void UpLaserDataProc(vector<vector<int>>up_laser, vector<float> &back);              // 上方数据处理，患者后背中心
	  void DownLaserDataProc(vector<vector<int>>down_laser, vector<float> &foot);           // 下方激光处理，返回患者腿部
	  void FootFind(vector<vector<int>>sub_object, float* foot);				  // 寻找腿部 没有找到，则返回0，否则返回半径及中心
	  void BackFind(vector<vector<int>>sub_object, float* foot);
	  vector<vector<int>> getLaserDownData(bool OUTFILE); //获取上方激光数据
	  vector<vector<int>> getLaserUpData(bool OUTFILE);  //获取下方激光数据

private :
	  
};
extern Laser_Sensor URG04LX;
#endif