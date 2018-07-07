#pragma once
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

class CEyeDetector
{
private:
	CascadeClassifier eyeDetector;	//眼睛检测器
    CascadeClassifier leftEyeDetector;//左眼检测器
    CascadeClassifier rightEyeDetector;//右眼检测器

public:

    CEyeDetector(void);
	~CEyeDetector(void);


    /**
    从输入的人脸图像中检测出眼睛的位置，如果分类器文件是用来检测单只眼睛的，返回的vector可能只有一个元素
    @param	face	人脸图像
    @return	把所有检测到的眼睛返回到vector容器中，如果没有检测到，vector的大小为0
	*/	
	vector<Rect> detectEyes(Mat face);

    /**
    检测左眼位置
    @param	face	人脸图像
    @return	左眼位置，如果没检测到返回Rect(0)
    */
    Rect detectLeftEye(Mat face);

    /**
    检测右眼位置
    @param	face	人脸图像
    @return	右眼位置，如果没检测到返回Rect(0)
    */
    Rect detectRightEye(Mat face);

private:
    /**
     * @brief 预处理图像
     * @param img   输入图像，在原图上操作
     */
    void preprocessImage(Mat img);
};

