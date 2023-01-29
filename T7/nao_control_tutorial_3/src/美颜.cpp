#include <opencv2/opencv.hpp>
#include <iostream>
#include"myApi.h"
 
using namespace std;
using namespace cv;
 
Mat g_srcImg; //原图
Mat g_beautyFgImg;  //美颜后的前景图
Mat g_vagueBeImg;  //模糊后背景图
Mat g_roiImg;  //前景和后景组合后图片
Mat g_rectImg;  //绘制矩形边框
Mat g_brightImg;  //调整亮度
 
 
// 调整模糊程度
int g_iVagueValue = 3;
int g_iVagueMaxValue = 10;
 
// 调整美颜程度
int g_iBeautyValue = 9;
int g_iBeautyMaxValue = 20;
 
// 绘制矩形，设置相框
int g_iRectValue = 5;
int g_iRectMaxValue = 30;
 
//调整亮度范围
int g_iBrightValue = 6;
int g_iBrightMaxValue = 20;
 
//调整模糊程度
void TrackbarVague(int, void*);
 
//调整美颜程度
void TrackbarBeauty(int, void*);
 
//绘制矩形调整相框
void TrackbarRect(int, void*);
 
//调整亮度程度
void TrackbarBright(int, void*);
 
//将美颜前景图调到模糊背景图层上
void SetRoi(void);
 
int main()
{
	g_srcImg = imread("F:\\testImage\\test2.png");
	if (!g_srcImg.data)
		return -1;
	namedWindow("img", WINDOW_FREERATIO);
	imshow("img", g_srcImg);
 
	/*
	功能：创建滚动条并将其附加到指定的窗口
	参数：trackbarname:创建的滚动条名称
		  winname：滚动条父窗口的名称
		  value：滚动条所在位置的值
		  count：滚动条的最大值，最小值总是0
		  onChange：指向滚动条改变位置时要调用的函数的指针（回调函数），函数的原型应该是void Foo（int， void*）;
					其中第一个参数是滚动条所在的位置，第二个参数是用户数据（见下一个参数）；如果回调函数传空，则不调用
					回调，但只更新滚动条值
		  userdata:按原样传递给回调的用户数据，它可以用来处理滚动条不使用全局变量事件
	返回：成功返回0
	*/
 
	createTrackbar("背景模糊", "img", &g_iVagueValue, g_iVagueMaxValue, TrackbarVague);
	createTrackbar("前景美颜", "img", &g_iBeautyValue, g_iBeautyMaxValue, TrackbarBeauty);
	TrackbarBeauty(0, 0);
 
	createTrackbar("相框宽度", "img", &g_iRectValue, g_iRectMaxValue, TrackbarRect);
	TrackbarRect(0, 0);
 
	createTrackbar("图片亮度", "img", &g_iBrightValue, g_iBrightMaxValue, TrackbarBright);
	TrackbarBright(0, 0);
	TrackbarVague(0, 0);
	waitKey(0);
	return 0;
}
 
void TrackbarVague(int, void*)
{
	//SignaX和SigmaY必须是奇数
	int iSigma = 2 * g_iVagueValue + 1;
	//利用高斯模糊做底图
	GaussianBlur(g_srcImg, g_vagueBeImg, Size(iSigma, iSigma), iSigma, iSigma);
	//全部初始化完才显示
	if (g_beautyFgImg.data && g_vagueBeImg.data)
	{
		SetRoi();
	}
}
 
void TrackbarBeauty(int, void*)
{
	// 利用双边模糊，实现磨皮的效果
	int iSigmaSpace = 50 + 10 * g_iBeautyValue;
	Mat tmpImg;
	bilateralFilter(g_srcImg, tmpImg, 15, iSigmaSpace, 0);
 
	// 再利用掩膜计算，增加图像对比度，最终实现美颜相机的效果
	Mat kernal = (Mat_<int>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
	filter2D(tmpImg, g_beautyFgImg, -1, kernal, Point(-1, -1), 0);
 
	// 功能：调整图像大小
	// 参数：src 源图像
	//		 dst 目标图像
	//		 dsize 输出图像大小，如果为(0, 0)，则大小为(img.cols * fx, img.rows * fy)
	//		 fx x方向的缩小比例
	//		 fy y方向的缩小比例
	resize(g_beautyFgImg, g_beautyFgImg, Size(0, 0), 0.8, 0.8);
 
	// 全部初始化完才显示
	if (g_beautyFgImg.data && g_vagueBeImg.data)
	{
		SetRoi();
	}
}
 
void TrackbarRect(int, void*)
{
	if (!g_roiImg.data)
	{
		return;
	}
 
	// 绘制一个矩形，当做相框
	int iPosX = (g_vagueBeImg.cols - g_beautyFgImg.cols) / 2 - g_iRectValue / 2;
	int iPosY = (g_vagueBeImg.rows - g_beautyFgImg.rows) / 2 - g_iRectValue / 2;
	int iWidth = g_beautyFgImg.cols + g_iRectValue;
	int iHeight = g_beautyFgImg.rows + g_iRectValue;
	Rect rect(iPosX, iPosY, iWidth, iHeight);
	Scalar color(255, 255, 255);
	g_rectImg = g_roiImg.clone();
	rectangle(g_rectImg, rect, color, g_iRectValue);
 
	// 调整颜色
	TrackbarBright(g_iBrightValue, 0);
}
 
void TrackbarBright(int, void*)
{
	if (!g_rectImg.data)
	{
		cout << "g_rectImg is empty" << endl;
		return;
	}
	int iRows = g_rectImg.rows;
	int iCols = g_rectImg.cols;
	Mat tmpImg = Mat::zeros(g_rectImg.size(), g_rectImg.type());
	float fAlpha = 0.1 + (float)g_iBrightValue / 10.0;
	float fBeta = 30; // 增益变量
 
	for (int i = 0; i < iRows; i++)
	{
		for (int j = 0; j < iCols; j++)
		{
			if (g_rectImg.channels() == 1)
			{
				float f = g_rectImg.at<uchar>(i, j);
				tmpImg.at<uchar>(i, j) = saturate_cast<uchar>(f * fAlpha + fBeta);
			}
			else if (g_rectImg.channels() == 3)
			{
				float fGreen = g_rectImg.at<Vec3b>(i, j)[0];
				float fBlue = g_rectImg.at<Vec3b>(i, j)[1];
				float fRed = g_rectImg.at<Vec3b>(i, j)[2];
 
				tmpImg.at<Vec3b>(i, j)[0] = saturate_cast<uchar>(fGreen * fAlpha + fBeta);
				tmpImg.at<Vec3b>(i, j)[1] = saturate_cast<uchar>(fBlue * fAlpha + fBeta);
				tmpImg.at<Vec3b>(i, j)[2] = saturate_cast<uchar>(fRed * fAlpha + fBeta);
			}
		}
	}
 
	// 保存调整亮度后的背景图
	g_brightImg = tmpImg.clone();
 
	imshow("img", g_brightImg);
}
 
 
void SetRoi(void)
{
	// 设定ROI区域
	int iPosX = (g_vagueBeImg.cols - g_beautyFgImg.cols) / 2;
	int iPosY = (g_vagueBeImg.rows - g_beautyFgImg.rows) / 2;
	g_roiImg = g_vagueBeImg.clone();
	Mat imageROI = g_roiImg(Rect(iPosX, iPosY, g_beautyFgImg.cols, g_beautyFgImg.rows));
	//加载掩摸
	Mat mask;
	cvtColor(g_beautyFgImg, mask, COLOR_BGR2GRAY);
	g_beautyFgImg.copyTo(imageROI, mask);
	TrackbarRect(g_iRectValue, 0);
}


