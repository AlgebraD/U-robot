#pragma once
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/face.hpp"

using namespace std;
using namespace cv::face;
using namespace cv;


//人脸识别器枚举类型
enum EN_FACE_REC_TYPE{
    EIGEN,
    FISHER,
    LBPH
};




class CFaceRecognizer
{
private:
	Mat		curFace;
	Size	faceSize;	//人脸图像大小
	Ptr<BasicFaceRecognizer>	eigenFaceRec;		//特征脸人脸识别器
	Ptr<BasicFaceRecognizer>	fisherFaceRec;		//线性判别分析人脸识别器
	Ptr<LBPHFaceRecognizer>		LBPHFaceRec;		//LBPH人脸识别器
    bool    trained;    //是否已训练了模型
    EN_FACE_REC_TYPE            rec_type;           //当前使用的人脸识别器类型
//    ST_FACE_REC_MODEL           recModel;           //模型信息
	/**
	faceDataPath目录结构
	.
	models.yaml			--保存的已训练模型
	faces.csv			--人脸数据描述文件 
	data				--人脸数据目录
		目录名1				--	该目录下文件为同一人的人脸图像
			0.jpg				--人脸图像
			1.jpg
			...
		目录名n
	*/
    string	faceDataPath;	//人脸图像文件路径,

public:
    /**
    构造器
    @param	faceDataPath	人脸数据路径
    @param	type            人脸识别器类型
    */
    CFaceRecognizer(string faceDataPath,EN_FACE_REC_TYPE type);
	~CFaceRecognizer(void);

	/**
    训练识别模型，每次调用该方法，都会清空之前训练的模型数据
    @param	faces	人脸图像
    @param	labels  图像对应的标签
	*/
    void train(vector<Mat> faces,vector<int> labels);

	/**
	更新识别模型，faces与labels一一对应，只有LBPH识别器才可以更新模型
	@param	faces	训练的图像
	@param	labels	图像对应的标签
	*/
    void updateModel(vector<Mat> faces,vector<int> labels);

    /**
    更新识别模型，所有faces属于同一个label，只有LBPH识别器才可以更新模型
    @param	faces	训练的图像
    @param	label	图像对应的标签
    */
    void updateModel(vector<Mat> faces,int label);

	/**
	人脸识别
	@param	face		待识别人脸
	@param	confidence	识别结果可信度
	@param	label		识别出的人脸对应的标签
	*/
	void recognize(Mat face,double& confidence,int& label);
	
	/**
	加载已训练的模型数据
	*/
    bool loadTrainedModel();

	/**
	保存已训练的模型数据
	*/
	void saveTrainedModel();

//    /**
//    根据人脸标签返回人名
//    @param  label   人脸标签
//    */
//    string getNameFromLabel(int label);

private:

};

