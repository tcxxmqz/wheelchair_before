#ifndef _SHOW_H_
#define _SHOW_H_
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>  
#include <Windows.h>
using namespace std;
//using namespace cv;

#define WINDOW_NAME1 "【绘制图1】"        //为窗口标题定义的宏 
#define WINDOW_NAME2 "【绘制图2】"        //为窗口标题定义的宏 
#define WINDIV   150      // 窗口边缘预留   
#define WINDOW_WIDTH 600 //定义窗口大小的宏
#define DETECT_MAX_DiS 2000 //激光最大检测距离
#define X_SCALE_RATIO (WINDOW_WIDTH)/(2*DETECT_MAX_DiS)
#define Y_SCALE_RATIO (WINDOW_WIDTH - 150 )/2500   // 500是激光后500mm距离
#define MAX_SPEED 0.3 // 机器人最大速度 （m/s）
class Show
{
	public:
		void DrawEllipse(Mat img, double angle, float x, float y, int a, int b);//绘制椭圆
		void DrawFilledCircle(Mat img, Point center);//绘制圆
		void DrawPolygon(Mat img);//绘制多边形
		void DrawLine(Mat img, Point start, Point end);//绘制线段
		void DisplayFoot(vector<float> foot, float *footgoal, float *newfootgoal, int goalangle, int footangle, float speed);

};

#endif