#include "show.h"
#include "Robot_Process.h"

void Show::DisplayFoot(vector<float> foot, float *footgoal, float *newfootgoal, int goalangle, int footangle, float speed)
{

	char code = (char)-1;
	float temGoal[2] = { 0, 0 };
	float temGoal2[2] = { 0, 0 };
	// �����հ׵�Matͼ��
	Mat atomImage = Mat::zeros(WINDOW_WIDTH, WINDOW_WIDTH, CV_8UC3);
	int tempx = 0, tempy = 0;

	// �������м�⵽���㲿 �� Ŀ��
	int f_size = foot.size();
	if (f_size == 0)return;
	int footcount = (f_size / 3);
	for (int i = 0; i < f_size; i = i + 3)
	{
		tempx = (foot[i] + DETECT_MAX_DiS) * X_SCALE_RATIO;
		tempy = (foot[i+1] + 500) * Y_SCALE_RATIO;
		ellipse(atomImage, Point(tempx, tempy), Size(9, 9), 0, 0, 360, Scalar(255, 129, 0), 1, 8);
		//cout <<'[' <<tempx << "   " << tempy <<']'<< endl;
	}

	// Ŀ��
		tempx = (footgoal[0] + DETECT_MAX_DiS) * X_SCALE_RATIO;
		tempy = (footgoal[1] + 500) * Y_SCALE_RATIO;
		circle(atomImage, Point(tempx, tempy), WINDOW_WIDTH / 128, Scalar(0, 0, 255), -1, 8);

		tempx = (newfootgoal[0] + DETECT_MAX_DiS) * X_SCALE_RATIO;
		tempy = (newfootgoal[1] + 500) * Y_SCALE_RATIO;

		circle(atomImage, Point(tempx, tempy), WINDOW_WIDTH / 128, Scalar(0, 0, 200), -1, 8);
		
		int line_s_x = 0, line_s_y = 0;
		int line_e_x = 0, line_e_y = 0;
		line_s_x = tempx - 20 * cos(footangle / 57.29);
		line_s_y = tempy - 20 * sin(footangle / 57.29);
		line_e_x = tempx + 20 * cos(footangle / 57.29);
		line_e_y = tempy + 20 * sin(footangle / 57.29);
		line(atomImage, Point(line_s_x, line_s_y), Point(line_e_x, line_e_y), Scalar(255, 0, 255), 1, 8);
	//��1.2���ٻ���Բ�ģ�������ԭ�㣩
		circle(atomImage, Point(300, 30), WINDOW_WIDTH / 128, Scalar(0, 255, 127), -1, 8);
		ellipse(atomImage, Point(300, 30), Size(9, 18), goalangle - 90 , 0, 360, Scalar(255, 255, 0), 4, 8);//�� �� ��
		ellipse(atomImage, Point(300, 30), Size(9, 18), goalangle -90 , 88, 92, Scalar(0, 0, 255), 7, 8); // �� �� ��  
		line(atomImage, Point(300, 30), Point(300, 60), Scalar(255, 0, 255), 1, 8);
	// �����ٶȽ�����
		line(atomImage, Point(200, 0), Point(200, 50), Scalar(255, 144, 30), 1, 8); // |
		line(atomImage, Point(210, 0), Point(210, 50), Scalar(255, 144, 30), 1, 8); // |
		line(atomImage, Point(200, 50), Point(210, 50), Scalar(255, 144, 30), 1, 8);// -
		line(atomImage, Point(205, 0), Point(205, (speed / MAX_SPEED) * 50), Scalar(255, 0, 255), 3, 8);// ����
		putText(atomImage, "Speed", Point(210, 10), cv::FONT_HERSHEY_DUPLEX, 0.3, cv::Scalar(255, 255, 255), 1);
		//DrawFilledCircle(atomImage, Point(footgoal[0] * SCALE_RATIO - WINDOW_WIDTH / 2, footgoal[1] * SCALE_RATIO));

	// ---------------------------<3>��ʾ���Ƴ���ͼ��------------------------

		imshow(WINDOW_NAME1, atomImage);
		moveWindow(WINDOW_NAME1, 0, 200);	
		code = (char)waitKey(1);
		atomImage = Scalar::all(200);
}



//-------------------------------��DrawEllipse( )������--------------------------------
//		�������Զ���Ļ��ƺ�����ʵ���˻��Ʋ�ͬ�Ƕȡ���ͬ�ߴ����Բ
//-----------------------------------------------------------------------------------------
void Show::DrawEllipse(Mat img, double angle, float x, float y , int a, int b)
{
	int thickness = 1;
	int lineType = 8;

	ellipse(img,
		Point(x, y),// Բ��
		Size(WINDOW_WIDTH / a, WINDOW_WIDTH / b),// ��Сλ�ھ��� ��
		angle,									  // �Ƕ�
		0,
		360,
		Scalar(255, 129, 0),                      // ��ɫ
		thickness,							      // �߿�
		lineType);								  // ����
}


//-----------------------------------��DrawFilledCircle( )������---------------------------
//		�������Զ���Ļ��ƺ�����ʵ����ʵ��Բ�Ļ���
//-----------------------------------------------------------------------------------------
void Show::DrawFilledCircle(Mat img, Point center)
{
	int thickness = -1;
	int lineType = 8;

	circle(img,
		center,
		WINDOW_WIDTH / 128,
		Scalar(0, 0, 255),
		thickness,
		lineType);
}


//-----------------------------------��DrawPolygon( )������--------------------------
//		�������Զ���Ļ��ƺ�����ʵ���˰�����εĻ���
//--------------------------------------------------------------------------------------
void Show::DrawPolygon(Mat img)
{
	int lineType = 8;

	//����һЩ��
	Point rookPoints[1][20];
	rookPoints[0][0] = Point(WINDOW_WIDTH / 4, 7 * WINDOW_WIDTH / 8);
	rookPoints[0][1] = Point(3 * WINDOW_WIDTH / 4, 7 * WINDOW_WIDTH / 8);
	rookPoints[0][2] = Point(3 * WINDOW_WIDTH / 4, 13 * WINDOW_WIDTH / 16);
	rookPoints[0][3] = Point(11 * WINDOW_WIDTH / 16, 13 * WINDOW_WIDTH / 16);
	rookPoints[0][4] = Point(19 * WINDOW_WIDTH / 32, 3 * WINDOW_WIDTH / 8);
	rookPoints[0][5] = Point(3 * WINDOW_WIDTH / 4, 3 * WINDOW_WIDTH / 8);
	rookPoints[0][6] = Point(3 * WINDOW_WIDTH / 4, WINDOW_WIDTH / 8);
	rookPoints[0][7] = Point(26 * WINDOW_WIDTH / 40, WINDOW_WIDTH / 8);
	rookPoints[0][8] = Point(26 * WINDOW_WIDTH / 40, WINDOW_WIDTH / 4);
	rookPoints[0][9] = Point(22 * WINDOW_WIDTH / 40, WINDOW_WIDTH / 4);
	rookPoints[0][10] = Point(22 * WINDOW_WIDTH / 40, WINDOW_WIDTH / 8);
	rookPoints[0][11] = Point(18 * WINDOW_WIDTH / 40, WINDOW_WIDTH / 8);
	rookPoints[0][12] = Point(18 * WINDOW_WIDTH / 40, WINDOW_WIDTH / 4);
	rookPoints[0][13] = Point(14 * WINDOW_WIDTH / 40, WINDOW_WIDTH / 4);
	rookPoints[0][14] = Point(14 * WINDOW_WIDTH / 40, WINDOW_WIDTH / 8);
	rookPoints[0][15] = Point(WINDOW_WIDTH / 4, WINDOW_WIDTH / 8);
	rookPoints[0][16] = Point(WINDOW_WIDTH / 4, 3 * WINDOW_WIDTH / 8);
	rookPoints[0][17] = Point(13 * WINDOW_WIDTH / 32, 3 * WINDOW_WIDTH / 8);
	rookPoints[0][18] = Point(5 * WINDOW_WIDTH / 16, 13 * WINDOW_WIDTH / 16);
	rookPoints[0][19] = Point(WINDOW_WIDTH / 4, 13 * WINDOW_WIDTH / 16);

	const Point* ppt[1] = { rookPoints[0] };
	int npt[] = { 20 };

	fillPoly(img,
		ppt,
		npt,
		1,
		Scalar(255, 255, 255),
		lineType);
}


//-----------------------------------��DrawLine( )������--------------------------
//		�������Զ���Ļ��ƺ�����ʵ�����ߵĻ���
//---------------------------------------------------------------------------------
void Show::DrawLine(Mat img, Point start, Point end)
{
	int thickness = 2;
	int lineType = 8;
	line(img,
		start,
		end,
		Scalar(255, 0, 0),
		thickness,
		lineType);
}