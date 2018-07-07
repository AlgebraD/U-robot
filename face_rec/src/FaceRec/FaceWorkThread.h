#pragma once

//#define __cplusplus 201103L
//#define _GLIBCXX_HAS_GTHREADS
//#define _GLIBCXX_USE_C99_STDINT_TR1
#include "opencv2/opencv.hpp"
#include <thread>
#include "FaceDetector.h"
#include "RecognizeResult.h"
#include "FaceRecognizer.h"
#include <map>

using namespace std;
using namespace cv;

enum EN_WORK_THREAD_STATUS{
    IDLE,           //空闲
    GATHER_FROM_CAMERA,      //正在从摄像头采集人脸
    GATHER_FROM_FILES,      //正在从文件采集人脸
    RECOGNIZING_ONCE,		//识别人脸，只识别一次
	RECOGNIZING_PERIOD,    //持续识别人脸一段时间，最识别出次数最多的最为识别结果
};

struct ST_GATHER_FACE_INFO{
    vector<Mat>		gatheredFaces;		//采集的人脸图像
    vector<Rect>	faceRectsInImg;		//人脸在源图中的位置和尺寸
    int     label;
    string  name;
    void clearInfo(){
        gatheredFaces.clear();
        faceRectsInImg.clear();
    }
};

//人脸识别标签信息
struct ST_FACE_DATA_INFO{
    vector<string>  names;      //人名
    vector<int>     labels;     //人脸图像标签，与人名一一对应
    void clearInfo(){
        names.clear();
        labels.clear();
    }
};

/**
人脸识别工作线程，该类不是线程安全的，建议做为全局惟一实例
*/
class CFaceWorkThread
{
private:
    EN_WORK_THREAD_STATUS   workStatus; //线程的工作状态
	int				gatherTimeInterval;	//采集时间间隔，单位为毫秒
	thread			*gatherThread;		//采集线程对象
	bool			b_exitThread;		//是否退出线程的标志
    ST_GATHER_FACE_INFO gatheredFaceInfos;    //采集的人脸数据信息
	Size			faceSize;			//采集输出的人脸大小
    char*           faceDataPath;       //人脸图像数据路径
    CRecognizeResult recResult;         //人脸识别结果
    int             faceRecTimeout;     //人脸识别超时时间，毫秒为单位
    ST_FACE_DATA_INFO faceDataInfo;   //人脸识别标签对应信息
    String          gatherFilePath;     //从文件采集人像的目录路径
	vector<vector<int>>	recognizeInfoPeriod;//持续识别的结果，内嵌的vector有两个元素，第一个为label，第二个为该label被识别出的次数
	int				recognizePeriodTime;	//持续识别的时间长度，以毫秒为单位

public:
    CFaceDetector	faceDetector;       //人脸检测器
    CFaceRecognizer faceRecognize;      //人脸识别器

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
     * @brief 初始化
     */
    void initialize();

    /**
     * @brief getWorkThreadStatus
     * @return 返回线程的工作状态
     */
    EN_WORK_THREAD_STATUS getWorkThreadStatus();

	/**
	开始从摄像头采集人脸图像，采集的图像保存在内存中。
	采集策略有两个，一、每隔一定时间才采集一次；二、采集的人脸图像和上次有明显区别才采集
	@param	timeInterval	采集人脸图像的时间间隔，单位为毫秒，默认每秒采集一次
	@param	faceSize		采集输出的脸部图片大小，必要时会对识别出的人脸进行转换
	*	
	*/	
    void beginGatherFaceFromCamera(Size size, int timeInterval=800);

    /**
    停止从摄像头采集人脸图像
    @return	返回采集到的人脸图像的数量
    */
    int endGatherFaceFromCamera();

    /**
     * @brief 从图像文件中读取人脸
     * @param path  文件目录，该目录下文件都为同一人
     */
    void gatherFaceFromFiles(String path,Size size);

    /**
     * @brief 设置采集人像的人名
     * @param name  人名
     */
    void setGatherName(string name);

	/**
    保存采集到的人脸图像到文件
	*/
    void saveGatheredFacesToFile();
	
    /**
    从摄像头识别人脸
    @param	timeOut	识别的超时时间
    *
    */
    void recognizeFaceFromCamera(int timeOut=5000);

	/**
    对视频流进行持续识别，取识别次数最多的人脸Label，降低误识别的概率
    @param	period	识别的持续时间，单位毫秒
    *
    */
    void recognizeFaceFromCameraPeriod(int period=3000);

    /**
    从图像文件识别人脸
    @param	filePath	文件路径
    *
    */
    void recognizeFaceFromFile(string filePath);

	/**
	计算两幅图片的相似度
	@param	img1	图片1
	@param	img2	图片2
	@return	返回相似度，越接近0越相似,0.2图像没有移动，0.4图像有移动，0.3做阈值		
	*/
	double calSimilarity(Mat img1,Mat img2);

    /**
    @return	返回采集到的人脸数据
    */
    ST_GATHER_FACE_INFO getGatheredFacesInfo();

    /**
     * @brief 根据csv文件内容，读取人脸数据信息
     */
    void readFaceDataCsv(vector<Mat>& faces,vector<int>& labels,vector<string>& names);

    /**
     * @brief 更新多个人脸数据信息
     * @param labels
     * @param names
     */
    void updateFaceDataInfo(vector<int> labels,vector<string> names);

    /**
     * @brief 更新单个人脸数据信息
     * @param labels
     * @param names
     */
    void updateFaceDataInfo(int label,string name);


    /**
     * @brief 清空采集的数据
     */
    void clearGatheredFaceInfo();

    /**
     * @brief 根据label返回对应的人名
     * @param label
     * @return
     */
    string getNameFromLabel(int label);

private:

    /**
    获取新的人脸图像标签
    */
    int getNewFaceLabel();



};
