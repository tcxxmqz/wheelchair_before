#include "Lasor.h"
#include <iostream>
#include <string.h>
using namespace std;

int gOriginal_data[682][2] = { 0 };

void Laser_Sensor::Data_Extract(char *data_str, int *data_array)
{
	int length = 0, starting_step= 0,ending_step=0;
	int step_temp[8] = { 0 }, data_byte = 0, data_block = 0,data_remain=0,data_remain_start=0;
	int k = 1000, count = 0;//count�����������洢���ݵ����յ�����
	int data_length = 0,temp=0;
	int temp_sum = 0, data_temp_high = 0, data_temp_low = 0,data;
	length = strlen(data_str);//������26 bytes �̶����ȵ�����  ���� Ϊ���⴫��������
	cout << length << endl << endl;
	
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
	//**************************************************************//
    //************ �����ݸ��� <= 64bytes ʱ������� ***************//
		//if (length <= 90 && length >= 26)  
	 //   {
		//	if (data_str[22] == 0x0A)//���ĺ����������
		//	{
		//		for (int i = 23; i < length - 3; i++)  //���ݵ���������Ϊ���ݵ�У���
		//		{
		//			temp_sum += data_str[i];
		//		}
		//		temp_sum = temp_sum & 0x3F;
		//		temp_sum = temp_sum + 0x30; //ȡ�ܺ͵ĵ���λ ��У���
		//		if (temp_sum == data_str[length - 3])//����У��� �� �����ܵ�У����Ƿ����
		//		{                                    //��������ʼ ������ȡ����
		//			data_length = ending_step - starting_step + 1;  //���� step ����
		//			for (int i = 23; (data_str[i+1] != 0x0A) || (data_str[i+2] != 0x0A); i=i+2)  //�������ݵ���ȡ
		//			{
		//			
		//				data_temp_high = data_str[i] - 0x30;
		//				data_temp_high = data_temp_high & 0x3F;//ȡ���ݵĸ���λ
		//				data_temp_high = data_temp_high << 6;
		//				data_temp_low = data_str[i+1] - 0x30;//ȡ���ݵĵ���λ
		//				data_temp_low = data_temp_low & 0x3F;
		//				data = data_temp_high + data_temp_low;//���ݺϳ�
		//				cout << data<< endl;

		//				gOriginal_data[count][0] = starting_step + count;  //�����ݴ洢�����յ�������
		//				gOriginal_data[count][1] = data;
		//				count++;
		//			} 
		//		}
		//		temp_sum = 0;
		//	}
	 //    }
	//**************************************************************//
	//************ �����ݸ��� >= 64bytes ʱ������� ***************//
		//if (length > 90 && length <= 1432 )  //�����ݸ��� >= 64bytes ʱ�������  
		//{
			if (data_str[22] == 0x0A)//���ĺ����������
			{
				int j = 0,i=0;
				data_length = ending_step - starting_step + 1;// 66�� 
				data_byte = data_length * 2;
				data_block = data_byte / 64;  //���ݿ���  64�ֽ�Ϊһ��
				data_remain = data_byte % 64; //ʣ��������ռ�ֽ�
				for ( i = 0; i < data_block; i++)  //���ݿ���
				{
					temp_sum = 0;
					for (j = 0; j < 64; j++)      //ÿһ�����ݸ��� 64 
					{
						temp_sum += data_str[23+i * 66 + j];
					}
					temp_sum = temp_sum & 0x3F;
					temp_sum = temp_sum + 0x30; //ȡ�ܺ͵ĵ���λ ��У���

					if (i == 0){ temp = data_str[23 + i * 64 + j]; }
					else { temp = data_str[23 + i * 64 + j + 2]; }

					if (temp_sum == temp )//����У��� �� �����ܵ�У����Ƿ����
					{                                             //��ʼ��ȡ����
						for (j = 0; j < 64; j = j + 2)      //ÿһ�����ݸ��� 64 
						{
							data_temp_high = data_str[23 + i * 66 + j] - 0x30;
							data_temp_high = data_temp_high & 0x3F;//ȡ���ݵĸ���λ
							data_temp_high = data_temp_high << 6;
							data_temp_low = data_str[23 + i * 66 + j +1 ] - 0x30;//ȡ���ݵĵ���λ
							data_temp_low = data_temp_low & 0x3F;
							data = data_temp_high + data_temp_low;//���ݺϳ�	
							gOriginal_data[count][0] = starting_step + count;  //�����ݴ洢�����յ�������
							gOriginal_data[count][1] = data;
							count++;
						}
					}
				}

			//********************ʣ�����ݴ���********************************//
				data_remain_start = data_block * 66+23;
				temp_sum = 0;
				for (j = data_remain_start ; j < data_remain_start + data_remain; j++)      //ÿһ�����ݸ��� 64 
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
						gOriginal_data[count][0] = starting_step + count;  //�����ݴ洢�����յ�������
						gOriginal_data[count][1] = data;
						count++;
					}
			    }
				
			}
		//}
		for (int i = 0; gOriginal_data[i][0] != 0; i++)
		{
			cout << gOriginal_data[i][0] << "  " << gOriginal_data[i][1] << endl;
		}
	}
}