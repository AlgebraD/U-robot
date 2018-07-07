#pragma once
#include "opencv2/core.hpp"
#include "opencv2/objdetect.hpp"
#include "EyeDetector.h"
#include "global.h"


using namespace std;
using namespace cv;

class CFaceDetector
{

private:
	CascadeClassifier	faceDetector;	//人脸检测器
	CEyeDetector		eyeDetector;	//眼睛检测器
//	vector<Mat>			gatheredFaces;	//存储采集的人脸图像

public:

	/**
	人脸检测构造器
    @param	faceClassiferFilePath 人脸分类器文件路径,
	*/
    CFaceDetector(char* faceClassiferFilePath);
	~CFaceDetector(void);

	/**
	从输入的图像中检测出所有的人脸
	@param	srcImg	待检测的图像
    @return	把所有检测到的人脸位置返回到vector容器中，如果没有检测到人脸，vector的大小为空
	*/	
	vector<Rect> detectFaces(Mat srcImg);

    /**
    从输入的图像中检测出所有的人脸，并对人脸进行预处理，处理后的图片可以直接传给识别模块识别
    @param	srcImg              待检测的图像
    @param  preprocessedFaces   存储预处理后的人脸图像
    @param  desireFaceSize      期望的人脸大小
    @return	把所有检测到的人脸位置返回到vector容器中，如果没有检测到人脸，vector的大小为空
    */
    vector<Rect> detectFacesAndPreprocess(Mat srcImg,vector<Mat>& preprocessedFaces,
                                          Size desireFaceSize=Size(92,112));

    /**
     * @brief 从图像文件中检测人脸
     * @param path      文件目录，该目录下所有文件必须为同一人
     * @param desireFaceSize    输出的人脸尺寸
     * @return
     */
    vector<Mat> detectFacesAndPreprocessFromFiles(String path,Size desireFaceSize=Size(92,112));

private:

    /**
    预处理待识别人脸图像（图像的几何变换、加权直方图均衡化、消噪、去除非人脸像素）
    @param	face	输入的人脸图像，在原图操作
    @param  desireFaceSize    期望的人脸大小
    @return 返回预处理结果，true of false
    */
    bool preprocessFace(Mat& face,Size desireFaceSize);

    /**
    人脸几何变换（缩放、旋转、平移），使用传入的图像最接近标准的正向人脸
    @param	face        输入的人脸图像，在原图操作，必须为单通道灰度图
    @param  leftEye     左眼位置
    @param  rightEye    右眼位置
    @param  desireFaceSize    期望的人脸大小
    @return	变换成功返回true，失败返回false
    */
    bool faceGeometricTransfer(Mat& face,Rect leftEye,Rect rightEye,Size desireFaceSize);

    /**
    直方图均衡化，先对左右两半边图像各均衡化，再对整幅图像均衡化，最后，把三幅图按不同的权重混合在一起，
        这样可以最大程度降低光照对图片的影响
    @param	face	输入的人脸图像，在原图操作，必须为单通道灰度图
    */
    void equalizeFace(Mat& face);

    /**
    给输入的人脸图像添加一个椭圆形蒙版，屏蔽掉角落一些无用的像素
    @param	face	输入的人脸图像，在原图操作
    @param  desireFaceSize    期望的人脸大小
    */
    void maskFace(Mat& face,Size desireFaceSize);

	/**
	按比例缩放图片
	@param	srcImg		源图片
	@param	destWidth	目标图片宽度，缩放比例为，srcImg.size().width/destImg.size().wdith,
							如果该值大于0，忽略destHeight参数
	@param	destHeight	目标图片高度，缩放比例为，srcImg.size().height/destImg.size().height,
							如果destWidth小于等于0，使用该参数
	@return	返回大于等于0的数值为缩放比例，否则缩放失败		
	*/
	float scaleImg(Mat& srcImg,int destWidth,int destHeight=0);

	
	
};

