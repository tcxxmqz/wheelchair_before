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

#define LASER_SCAN_RANGE 1500     //ȥ����Χ���������
extern vector<vector<int>> gOriginal_data1;//���ڴ�ż����״�����������  �д�Ŷ�Ӧ�ĵ�  �д�ŵ�����Ӧ�ľ��루mm��    C1:�� n ��  C2:��->����
extern vector<vector<int>> gOriginal_data2;//���ڴ�ż����״�����������  �д�Ŷ�Ӧ�ĵ�  �д�ŵ�����Ӧ�ľ��루mm��
extern unsigned char gLaser_Str_Buff1[LASER_DATA_BUFF_SIZE];
extern unsigned char gLaser_Str_Buff2[LASER_DATA_BUFF_SIZE];
extern float gDis_Angle[2];
class Laser_Sensor
{
  public :
	  vector<vector<int>> Data_Extract(unsigned char *data_str);
	  void Laser_Init(int LASERx);
	  bool Obtain_Data(int start_step, int end_step, unsigned char *data_buff, int LASERx);
	  void Dist_Angle_Calculate(int (*data_array)[2], float *dis_angle );          //����Ĳ���Ϊ���⴫���� ��ȥ�� ������ 
	  void LasorDataOut(vector<vector<int>> data, int LASERx);        // �������Ϊ�����ԭʼ���ݣ�������txt�ļ���//���ز���Ϊ��ȡ�ĽǶȺ;����ֵ   
	  void UpLaserDataProc(vector<vector<int>>up_laser, vector<float> &back);              // �Ϸ����ݴ������ߺ�����
	  void DownLaserDataProc(vector<vector<int>>down_laser, vector<float> &foot);           // �·����⴦�����ػ����Ȳ�
	  void FootFind(vector<vector<int>>sub_object, float* foot);				  // Ѱ���Ȳ� û���ҵ����򷵻�0�����򷵻ذ뾶������
	  void BackFind(vector<vector<int>>sub_object, float* foot);
	  vector<vector<int>> getLaserDownData(bool OUTFILE); //��ȡ�Ϸ���������
	  vector<vector<int>> getLaserUpData(bool OUTFILE);  //��ȡ�·���������

private :
	  
};
extern Laser_Sensor URG04LX;
#endif