#ifndef CRECOGNIZERESULT_H
#define CRECOGNIZERESULT_H

#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;

enum EN_REC_RESULT_STATUS{
    REC_SUCCESS,        //识别成功
    REC_TIMEOUT,        //识别超时
    REC_UNKNOWN_BODY,   //未知的识别对象
    REC_NOBODY          //未检测到人脸
};

//识别出的人脸信息
struct ST_REC_RESULT{
    int             label;          //识别模型中，人脸对应的标签
    double          confidence;     //识别结果可信度
    char            sex;
    unsigned short  age;
    string          name;

    Rect            faceRect;
    Rect            leftEyeRect;
    Rect            rightEyeRect;
    Rect            noseRect;
    Rect            mouseRect;

};

/**
 * @brief 人脸识别结果
 */
class CRecognizeResult
{
public:
    EN_REC_RESULT_STATUS    status;     //识别结果状态
    vector<ST_REC_RESULT>   results;    //所有识别出的对象信息

public:
    CRecognizeResult();
};

#endif // CRECOGNIZERESULT_H
