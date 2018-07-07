#include "EyeDetector.h"
#include "global.h"

CEyeDetector::CEyeDetector(void)
{


#if defined(_WINDOWS)
   char* file1 = "E:\\project\\opencv\\FaceRec\\FaceData\\haarcascade_eye_tree_eyeglasses.xml";
    char* file2 = "E:\\project\\opencv\\FaceRec\\FaceData\\haarcascade_lefteye_2splits.xml";
    char* file3 = "E:\\project\\opencv\\FaceRec\\FaceData\\haarcascade_righteye_2splits.xml";

#elif   defined(_LINUX)
	  char* file1 = "/usr/data/FaceData/haarcascade_eye.xml";
    char* file2 = "/usr/data/FaceData/haarcascade_lefteye_2splits.xml";
    char* file3 = "/usr/data/FaceData/haarcascade_righteye_2splits.xml";

#endif

    
    eyeDetector.load(file1);
    leftEyeDetector.load(file2);
    rightEyeDetector.load(file3);
}


CEyeDetector::~CEyeDetector(void)
{
}


vector<Rect> CEyeDetector::detectEyes(Mat face){

    //预处理输入图像
    Mat tempImg = face.clone();
    preprocessImage(tempImg);

	//检测眼睛
	vector<Rect> detectEyes;
	int		flags = CASCADE_SCALE_IMAGE;
	Size	minSize(10,10);
	float	searchScaleFactor = 1.1f;
	int		minNeighbors = 4;
	
	eyeDetector.detectMultiScale(tempImg,detectEyes,searchScaleFactor,minNeighbors,flags,minSize);

	return detectEyes;
}


Rect CEyeDetector::detectLeftEye(Mat face){
    //预处理输入图像
    Mat tempImg = face.clone();
    preprocessImage(tempImg);

    //检测左眼
    vector<Rect> detectEyes;
    int		flags = CASCADE_SCALE_IMAGE;
    Size	minSize(10,10);
    float	searchScaleFactor = 1.1f;
    int		minNeighbors = 4;

    leftEyeDetector.detectMultiScale(tempImg,detectEyes,searchScaleFactor,minNeighbors,flags,minSize);
    if(detectEyes.size() > 0)
        return detectEyes[0];
    else
        return Rect(0,0,0,0);
}


Rect CEyeDetector::detectRightEye(Mat face){
    //预处理输入图像
    Mat tempImg = face.clone();
    preprocessImage(tempImg);

    //检测右眼
    vector<Rect> detectEyes;
    int		flags = CASCADE_SCALE_IMAGE;
    Size	minSize(10,10);
    float	searchScaleFactor = 1.1f;
    int		minNeighbors = 4;

    rightEyeDetector.detectMultiScale(tempImg,detectEyes,searchScaleFactor,minNeighbors,flags,minSize);
    if(detectEyes.size() > 0)
        return detectEyes[0];
    else
        return Rect(0,0,0,0);
}

void CEyeDetector::preprocessImage(Mat img){
    convertToGrayImg(img);
    equalizeHist(img,img);
}
