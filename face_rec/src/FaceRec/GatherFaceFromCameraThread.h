#pragma once

//#define __cplusplus 201103L
//#define _GLIBCXX_HAS_GTHREADS
//#define _GLIBCXX_USE_C99_STDINT_TR1
#include "opencv2/opencv.hpp"
#include <thread>
#include "FaceDetector.h"
#include "RecognizeResult.h"

using namespace std;
using namespace cv;

/**
从摄像头采集人脸图片的线程，该类不是线程安全的，建议做为全局惟一实例
*/
class CFaceWorkThread
{
private:
	int				gatherTimeInterval;	//采集时间间隔，单位为毫秒
	thread			*gatherThread;		//采集线程对象
	bool			b_exitThread;		//是否退出线程的标志
	vector<Mat>		gatheredFaces;		//采集的人脸图像
	vector<Rect>	faceRectsInImg;		//人脸在源图中的位置和尺寸
	Size			faceSize;			//采集输出的人脸大小
    char*           faceDataPath;       //人脸图像数据路径
    CRecognizeResult recResult;         //人脸识别结果

public:
	CFaceDetector	faceDetector;	//人脸识别器
	bool			isGathering;	//是否正在采集人脸
	

public:
	/**
	构造器
	@param	faceClassiferFilePath 人脸分类器文件路径
    @param  faceDataPath          人脸数据路径
	*/
    CFaceWorkThread(char* faceClassiferFilePath,char *faceDataPath);
    ~CFaceWorkThread(void);

	void operator()();//线程函数

	/**
	开始从摄像头采集人脸图像，采集的图像保存在内存中。
	采集策略有两个，一、每隔一定时间才采集一次；二、采集的人脸图像和上次有明显区别才采集
	@param	timeInterval	采集人脸图像的时间间隔，单位为毫秒，默认每秒采集一次
	@param	faceSize		采集输出的脸部图片大小，必要时会对识别出的人脸进行转换
	*	
	*/	
    void beginGatherFaceFromCamera(Size faceSize = Size(92,112), int timeInterval=1000);

    /**
    停止从摄像头采集人脸图像
    @return	返回采集到的人脸图像的数量
    */
    int endGatherFaceFromCamera();

	/**
    保存采集到的人脸图像到文件
    @param	name		人名
	@param	id			人脸图像的id标识
	*/
    void saveGatheredFacesToFile(char *name,int id);
	
    // /**
    // 开始从摄像头采集人脸图像，采集的图像保存在内存中。
    // 采集策略有两个，一、每隔一定时间才采集一次；二、采集的人脸图像和上次有明显区别才采集
    // @param	timeInterval	采集人脸图像的时间间隔，单位为毫秒，默认每秒采集一次
    // @param	faceSize		采集输出的脸部图片大小，必要时会对识别出的人脸进行转换
    // *
    // */
    // void beginGatherFaceFromCamera(Size faceSize = Size(92,112), int timeInterval=1000);

	/**
	计算两幅图片的相似度
	@param	img1	图片1
	@param	img2	图片2
	@return	返回相似度，越接近0越相似,0.2图像没有移动，0.4图像有移动，0.3做阈值		
	*/
	double calSimilarity(Mat img1,Mat img2);

	vector<Mat> getGatheredFaces();

private:
    /**
    获取新的人脸图像标签
    */
    int getNewFaceLabel();
};
