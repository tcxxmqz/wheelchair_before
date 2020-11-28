#include "Lasor.h"
#include <iostream>
#include <string.h>
using namespace std;

int gOriginal_data[682][2] = { 0 };

void Laser_Sensor::Data_Extract(char *data_str, int *data_array)
{
	int length = 0, starting_step= 0,ending_step=0;
	int step_temp[8] = { 0 }, data_byte = 0, data_block = 0,data_remain=0,data_remain_start=0;
	int k = 1000, count = 0;//count用来计数，存储数据到最终的数组
	int data_length = 0,temp=0;
	int temp_sum = 0, data_temp_high = 0, data_temp_low = 0,data;
	length = strlen(data_str);//其中有26 bytes 固定长度的数据  其余 为激光传感器数据
	cout << length << endl << endl;
	
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
	//**************************************************************//
    //************ 当数据个数 <= 64bytes 时处理程序 ***************//
		//if (length <= 90 && length >= 26)  
	 //   {
		//	if (data_str[22] == 0x0A)//它的后面就是数据
		//	{
		//		for (int i = 23; i < length - 3; i++)  //数据倒数第三个为数据的校验和
		//		{
		//			temp_sum += data_str[i];
		//		}
		//		temp_sum = temp_sum & 0x3F;
		//		temp_sum = temp_sum + 0x30; //取总和的低六位 即校验和
		//		if (temp_sum == data_str[length - 3])//计算校验和 与 所接受的校验和是否相等
		//		{                                    //如果相等则开始 解析提取数据
		//			data_length = ending_step - starting_step + 1;  //计算 step 个数
		//			for (int i = 23; (data_str[i+1] != 0x0A) || (data_str[i+2] != 0x0A); i=i+2)  //进行数据的提取
		//			{
		//			
		//				data_temp_high = data_str[i] - 0x30;
		//				data_temp_high = data_temp_high & 0x3F;//取数据的高六位
		//				data_temp_high = data_temp_high << 6;
		//				data_temp_low = data_str[i+1] - 0x30;//取数据的低六位
		//				data_temp_low = data_temp_low & 0x3F;
		//				data = data_temp_high + data_temp_low;//数据合成
		//				cout << data<< endl;

		//				gOriginal_data[count][0] = starting_step + count;  //将数据存储到最终的数组中
		//				gOriginal_data[count][1] = data;
		//				count++;
		//			} 
		//		}
		//		temp_sum = 0;
		//	}
	 //    }
	//**************************************************************//
	//************ 当数据个数 >= 64bytes 时处理程序 ***************//
		//if (length > 90 && length <= 1432 )  //当数据个数 >= 64bytes 时处理程序  
		//{
			if (data_str[22] == 0x0A)//它的后面就是数据
			{
				int j = 0,i=0;
				data_length = ending_step - starting_step + 1;// 66点 
				data_byte = data_length * 2;
				data_block = data_byte / 64;  //数据块数  64字节为一块
				data_remain = data_byte % 64; //剩余数据所占字节
				for ( i = 0; i < data_block; i++)  //数据块数
				{
					temp_sum = 0;
					for (j = 0; j < 64; j++)      //每一块数据个数 64 
					{
						temp_sum += data_str[23+i * 66 + j];
					}
					temp_sum = temp_sum & 0x3F;
					temp_sum = temp_sum + 0x30; //取总和的低六位 即校验和

					if (i == 0){ temp = data_str[23 + i * 64 + j]; }
					else { temp = data_str[23 + i * 64 + j + 2]; }

					if (temp_sum == temp )//计算校验和 与 所接受的校验和是否相等
					{                                             //开始提取数据
						for (j = 0; j < 64; j = j + 2)      //每一块数据个数 64 
						{
							data_temp_high = data_str[23 + i * 66 + j] - 0x30;
							data_temp_high = data_temp_high & 0x3F;//取数据的高六位
							data_temp_high = data_temp_high << 6;
							data_temp_low = data_str[23 + i * 66 + j +1 ] - 0x30;//取数据的低六位
							data_temp_low = data_temp_low & 0x3F;
							data = data_temp_high + data_temp_low;//数据合成	
							gOriginal_data[count][0] = starting_step + count;  //将数据存储到最终的数组中
							gOriginal_data[count][1] = data;
							count++;
						}
					}
				}

			//********************剩余数据处理********************************//
				data_remain_start = data_block * 66+23;
				temp_sum = 0;
				for (j = data_remain_start ; j < data_remain_start + data_remain; j++)      //每一块数据个数 64 
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
						gOriginal_data[count][0] = starting_step + count;  //将数据存储到最终的数组中
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