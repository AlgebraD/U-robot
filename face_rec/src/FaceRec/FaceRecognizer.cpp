#include "FaceRecognizer.h"
#include "global.h"
#include "opencv2/highgui.hpp"
#include <time.h>
#include <iostream>
using namespace std;

CFaceRecognizer::CFaceRecognizer(string faceDataPath,EN_FACE_REC_TYPE type)
    :rec_type(type)
{
    this->faceDataPath = faceDataPath;
    eigenFaceRec    = createEigenFaceRecognizer();
    fisherFaceRec   = createFisherFaceRecognizer();
    LBPHFaceRec     = createLBPHFaceRecognizer();
	LBPHFaceRec->setThreshold(1000);
	fisherFaceRec->setThreshold(2);
	eigenFaceRec->setThreshold(2);
    trained = false;
}


CFaceRecognizer::~CFaceRecognizer(void)
{
}


void CFaceRecognizer::train(vector<Mat> faces,vector<int> labels){

    /****************开始训练模型*****************/
    if(faces.size() <= 0 || faces.size() != labels.size()){
        logger.log("训练模型输入参数有误",LOG_WARN);
        return;
    }

	clock_t tick_start	=	clock();
	cout<<"开始训练模型"<<endl;
    if(rec_type == EIGEN){
        eigenFaceRec->train(faces,labels);
    }else if(rec_type == FISHER){
        fisherFaceRec->train(faces,labels);
    }else if(rec_type == LBPH){
        LBPHFaceRec->train(faces,labels);
    }
	cout<<"模型训练结束，耗时："<<(clock()-tick_start)*1000/CLOCKS_PER_SEC<<"ms"<<endl;
    trained = true;
}


void CFaceRecognizer::updateModel(vector<Mat> faces,vector<int> labels){

    if(faces.size() == 0 || labels.size() == faces.size() )
        return;

    if(!trained){//如果还没有训练模型，先训练模型
        train(faces,labels);
        return;
    }
    if(rec_type == LBPH){
        LBPHFaceRec->update(faces,labels);
    }

}

void CFaceRecognizer::updateModel(vector<Mat> faces,int label){

    if(faces.size() == 0 )
        return;


    vector<int> labels;
    for(int i=0;i<faces.size();i++){
        labels.push_back(label);
    }
    if(!trained){//如果还没有训练模型，先训练模型
        train(faces,labels);
        return;
    }

    if(rec_type == LBPH){
        LBPHFaceRec->update(faces,labels);
    }
}

void CFaceRecognizer::recognize(Mat face,double& confidence,int& label){

	time_t tick_start = clock();
    if(rec_type == EIGEN){
        eigenFaceRec->predict(face,label,confidence);
    }else if(rec_type == FISHER){
        fisherFaceRec->predict(face,label,confidence);
    }else if(rec_type == LBPH){
        LBPHFaceRec->predict(face,label,confidence);
    }

	cout<<"人脸识别耗时：" << (clock()-tick_start)*1000/CLOCKS_PER_SEC <<" ms"<<endl;
}


void CFaceRecognizer::saveTrainedModel(){
    string path ;
#if defined( _WINDOWS )
        path = faceDataPath + "\\";
#elif defined(_LINUX)
        path = faceDataPath + "/";
#endif

    if(rec_type == EIGEN){
        eigenFaceRec->save(path+"eigenModel.yml");
    }else if(rec_type == FISHER){
        fisherFaceRec->save(path+"fisherModel.yml");
    }else if(rec_type == LBPH){
        LBPHFaceRec->save(path+"LBPHModel.yml");
    }
}


bool CFaceRecognizer::loadTrainedModel(){

    string path ;
#if defined( _WINDOWS )
        path = faceDataPath + "\\";
#elif defined(_LINUX)
        path = faceDataPath + "/";
#endif

    if(rec_type == EIGEN){

        path = path+"eigenModel.yml";
        if(!isFileOrDicExist(path.c_str()))
            return false;
        eigenFaceRec->load(path);

    }else if(rec_type == FISHER){

        path = path+"fisherModel.yml";
        if(!isFileOrDicExist(path.c_str()))
            return false;
        fisherFaceRec->load(path);

    }else if(rec_type == LBPH){

        path = path+"LBPHModel.yml";
        if(!isFileOrDicExist(path.c_str()))
            return false;
        LBPHFaceRec->load(path);

    }
	cout<<"模型已加载！"<<endl;
	trained = true;
    return true;
}


