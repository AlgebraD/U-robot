#include "FaceDetector.h"
#include "fstream"
#include "global.h"
#include "opencv2/imgproc.hpp"


CFaceDetector::CFaceDetector(char* classiferFilePath)
{
	//检测文件有效性
	ifstream fs(classiferFilePath,ifstream::in);
	if(!fs){
		char *errStr = "人脸检测器文件加载失败";
		logger.log(errStr,LOG_ERROR);
        throw exception();
	}
	fs.close();

	faceDetector.load(classiferFilePath);
}


CFaceDetector::~CFaceDetector(void)
{
}



float CFaceDetector::scaleImg(Mat& srcImg,int destWidth,int destHeight){
	float scale = 0;
	float srcHeight = srcImg.size().height;
	float srcWidth  = srcImg.size().width;
	if(destWidth > 0)
		scale = destWidth / srcWidth;
	else if(destHeight > 0)
		scale = destHeight / srcHeight;
	if(scale == 0){
		logger.log("scaleImg函数缩放级别有误",LOG_WARN);
		return -1;
	}
	int height	= srcHeight * scale;
	int width	= srcWidth * scale;
	resize(srcImg,srcImg,Size(width,height));
	return scale;
}



vector<Rect> CFaceDetector::detectFaces(Mat srcImg){
	
	//预处理输入图像
	Mat tempImg = srcImg.clone();
	convertToGrayImg(tempImg);
	float scale = scaleImg(tempImg,320);		//控制输入图像的尺寸，以最快的速度检测人脸
	equalizeHist(tempImg,tempImg);
	
	//检测人脸
	vector<Rect> detectFaces;	//使用人脸分类器检测出的人脸
	vector<Rect> realFaces;		//对detectFaces再使用眼睛分类器识别出的人脸
	int		flags = CASCADE_SCALE_IMAGE;
	Size	minFaceSize(50,50);
	float	searchScaleFactor = 1.1f;
	int		minNeighbors = 4;
	
	faceDetector.detectMultiScale(tempImg,detectFaces,searchScaleFactor,minNeighbors,flags,minFaceSize);

	
	int srcWidth = srcImg.size().width;
	int srcHeight = srcImg.size().height;
	int faceCount = detectFaces.size();
	for(int i=0;i<faceCount;i++){
		if(scale != 1){
			//恢复检测出的人脸的原始大小，并确保没有超出srcImg边界
			Rect scaledRect = detectFaces.at(i);
			Rect originalRect;
			int x		= cvRound( scaledRect.x / scale );
			int y		= cvRound( scaledRect.y / scale );
			int width	= cvRound( scaledRect.width / scale );
			int height	= cvRound( scaledRect.height / scale );
			x = x < 0 ? 0 : x;
			y = y < 0 ? 0 : y;
			x = x > srcWidth ? srcWidth : x;
			y = y > srcHeight ? srcHeight : y;
			width = x+width > srcWidth ? srcWidth - x : width;
			height = y+height > srcHeight ? srcHeight - y : height;
			detectFaces[i] = Rect(x,y,width,height);
		}

		//再对识别出的人脸检眼睛位置，过滤掉误识别的人脸图像
		Mat face = srcImg(detectFaces[i]);
		int eyeCount = eyeDetector.detectEyes(face).size();
        if(eyeCount > 0){
			realFaces.push_back(detectFaces[i]);
		}
	}

	return realFaces;
}


vector<Rect> CFaceDetector::detectFacesAndPreprocess(Mat srcImg,vector<Mat>& preprocessedFaces,
                                                     Size desireFaceSize){

    //预处理输入图像
    Mat tempImg = srcImg.clone();
    convertToGrayImg(tempImg);
    float scale = scaleImg(tempImg,320);		//控制输入图像的尺寸，以最快的速度检测人脸
    equalizeHist(tempImg,tempImg);

    //检测人脸
    vector<Rect> detectFaces;	//使用人脸分类器检测出的人脸
    vector<Rect> realFaces;		//对detectFaces再使用眼睛分类器识别出的人脸
    int		flags = CASCADE_FIND_BIGGEST_OBJECT | CASCADE_DO_ROUGH_SEARCH;
    Size	minFaceSize(50,50);
    float	searchScaleFactor = 1.1f;
    int		minNeighbors = 3;

	if(tempImg.empty()){
		return realFaces;
	}

	clock_t tick_start	=	clock();
    faceDetector.detectMultiScale(tempImg,detectFaces,searchScaleFactor,minNeighbors,flags,minFaceSize);
	cout<<"检测人脸花费时间：" << (clock()-tick_start)*1000/CLOCKS_PER_SEC <<" ms"<<endl;

    int srcWidth = srcImg.size().width;
    int srcHeight = srcImg.size().height;
    int faceCount = detectFaces.size();
    for(int i=0;i<faceCount;i++){
        if(scale != 1){
            //恢复检测出的人脸的原始大小，并确保没有超出srcImg边界
            Rect scaledRect = detectFaces.at(i);
            Rect originalRect;
            int x		= cvRound( scaledRect.x / scale );
            int y		= cvRound( scaledRect.y / scale );
            int width	= cvRound( scaledRect.width / scale );
            int height	= cvRound( scaledRect.height / scale );
            x = x < 0 ? 0 : x;
            y = y < 0 ? 0 : y;
            x = x > srcWidth ? srcWidth : x;
            y = y > srcHeight ? srcHeight : y;
            width = x+width > srcWidth ? srcWidth - x : width;
            height = y+height > srcHeight ? srcHeight - y : height;
            detectFaces[i] = Rect(x,y,width,height);
        }

        //做人脸识别前的预处理
		tick_start = clock();
        Mat face = srcImg(detectFaces[i]);
        if(preprocessFace(face,desireFaceSize)){
            realFaces.push_back(detectFaces[i]);
            preprocessedFaces.push_back(face);
        }
		//cout<<"人脸预处理花费时间：" << (clock()-tick_start)*1000/CLOCKS_PER_SEC <<" ms"<<endl;
    }

    return realFaces;
}

vector<Mat> CFaceDetector::detectFacesAndPreprocessFromFiles(String path,Size desireFaceSize){
        vector<String> filePaths;
        listDir(path.c_str(),filePaths);

        vector<Mat> faces;
        for(int i=0;i<filePaths.size();i++){
               Mat image = imread(filePaths[i]);
               detectFacesAndPreprocess(image,faces,desireFaceSize);
        }
        return faces;
}

bool CFaceDetector::preprocessFace(Mat& face,Size desireFaceSize){
	
    vector<Rect> leftEye    = eyeDetector.detectEyes(face(Rect(0,0,face.size().width/2,face.size().height)));
    vector<Rect> rightEye   = eyeDetector.detectEyes(face(Rect(face.size().width/2,0,face.size().width/2,face.size().height)));
//    if(IS_ZERO_RECT(leftEyeRect) || IS_ZERO_RECT(rightEyeRect))
//        return false;
    if(leftEye.size()<=0 || rightEye.size() <= 0)
        return false;
	
    Rect leftEyeRect    = leftEye[0];
    Rect rightEyeRect   = rightEye[0];
    rightEyeRect.x      += face.size().width/2;
    convertToGrayImg(face);
    if(false == faceGeometricTransfer(face,leftEyeRect,rightEyeRect,desireFaceSize))
		return false;
	//resize(face,face,desireFaceSize);
    equalizeFace(face);	//加权直方图均衡化人脸图像，最小化光照因素对人脸识别的影响
    Mat temp = Mat(face.rows,face.cols,face.type());
    bilateralFilter(face,temp,0,20.0,2.0);	//双边滤波，去除噪声
    maskFace(temp,desireFaceSize);
    temp.copyTo(face);
    return true;
}

bool CFaceDetector::faceGeometricTransfer(Mat& face,Rect leftEye,Rect rightEye,Size desireFaceSize){
    // Get the center between the 2 eyes.
    Point2f eyesCenter;
    eyesCenter.x = (leftEye.x + rightEye.x) * 0.5f;
    eyesCenter.y = (leftEye.y + rightEye.y) * 0.5f;
    // Get the angle between the 2 eyes.
    double dy = (rightEye.y - leftEye.y);
    double dx = (rightEye.x - leftEye.x);
    double len = sqrt(dx*dx + dy*dy);
    // Convert Radians to Degrees.
    double angle = atan2(dy, dx) * 180.0/CV_PI;
    // Hand measurements shown that the left eye center should
    // ideally be roughly at (0.16, 0.14) of a scaled face image.
    const double DESIRED_LEFT_EYE_X = 0.16;
    const double DESIRED_LEFT_EYE_Y = 0.14;
    const double DESIRED_RIGHT_EYE_X = (1.0f - 0.16);

     // Get the amount we need to scale the image to be the desired
     // fixed size we want.
    const int DESIRED_FACE_WIDTH = desireFaceSize.width;
    const int DESIRED_FACE_HEIGHT = desireFaceSize.height;
    double desiredLen = (DESIRED_RIGHT_EYE_X - 0.16);
    double scale = desiredLen * DESIRED_FACE_WIDTH / len;
	//scale -= 0.1;
    if( abs(angle) > 10){
		cout<<"请正对摄像头，angle:"<<angle<<" scale:"<<scale<<endl;
		return false;
	}

    // Get the transformation matrix for the desired angle & size.
    Mat rot_mat = getRotationMatrix2D(eyesCenter, angle, scale);
    //Mat rot_mat = getRotationMatrix2D(eyesCenter, 0, scale);
//     Shift the center of the eyes to be the desired center.
    double ex = DESIRED_FACE_WIDTH * 0.5f - eyesCenter.x;
    double ey = DESIRED_FACE_HEIGHT * DESIRED_LEFT_EYE_Y - eyesCenter.y;
    rot_mat.at<double>(0, 2) += ex-20;
    rot_mat.at<double>(1, 2) += ey-15;

     // Transform the face image to the desired angle & size &
     // position! Also clear the transformed image background to a
    // default grey.
//    Mat warped = Mat(DESIRED_FACE_HEIGHT, DESIRED_FACE_WIDTH,CV_8U, Scalar(128));
//    warpAffine(face, warped, rot_mat, warped.size());
    warpAffine(face, face, rot_mat, desireFaceSize);
	//cout<<"angle:"<<angle<<" scale:"<<scale<<endl;
	return true;
}

void CFaceDetector::equalizeFace(Mat& face){
    int width = face.size().width;
    int midWidth = width / 2;
    int height = face.size().height;
    Mat whole,leftSide,rightSide;
    leftSide = face(Rect(0,0,midWidth,height));
    rightSide = face(Rect(midWidth,0,midWidth,height));
    equalizeHist(face,whole);
    equalizeHist(leftSide,leftSide);
    equalizeHist(rightSide,rightSide);

    for(int h = 0;h<height;h++){
        for(int w = 0;w<width;w++){
            if(w<width*1/4){
                //用leftSide左半边均值化后对应的像素替换原图对应像素
                face.at<uchar>(h,w) = leftSide.at<uchar>(h,w);
            }else if(w<width*2/4){
                //用leftSide右半边均值化后对应的像素和whole对应像素加权混合后，替换原图对应像素，whole的权值从左到右逐渐变大
                float f = float(w-width/4)/(width/4);
                uchar p = leftSide.at<uchar>(h,w)*(1-f) + whole.at<uchar>(h,w)*f;
                face.at<uchar>(h,w) = p;
            }else if(w<width*3/4){
                //用rightSide左半边均值化后对应的像素和whole对应像素加权混合后，替换原图对应像素，whole的权值从左到右逐渐变小
                float f = float(w-width/2)/(width/4);
                uchar p = rightSide.at<uchar>(h,w-width/2)*f + whole.at<uchar>(h,w)*(1-f);
                face.at<uchar>(h,w) = p;
            }else{
                //用rightSide右半边均值化后对应的像素替换原图对应像素
                face.at<uchar>(h,w) = rightSide.at<uchar>(h,w-width/2);
            }
        }
    }
}


void CFaceDetector::maskFace(Mat& face,Size desireFaceSize){
    // Draw a black-filled ellipse in the middle of the image.
    // First we initialize the mask image to white (255).
    Mat mask = Mat(face.size(), CV_8UC1, Scalar(255));
    double dw = desireFaceSize.width;
    double dh = desireFaceSize.height;
    Point faceCenter = Point( cvRound(dw * 0.5),cvRound(dh * 0.5) );
    Size size = Size( cvRound(dw * 0.5), cvRound(dh * 0.9) );
   
//	Point faceCenter = Point( cvRound(dw * 0.5),cvRound(dh * 0.5) );
//    Size size = Size( cvRound(dw * 0.45), cvRound(dh * 0.7) );

	ellipse(mask, faceCenter, size, 0, 0, 360, Scalar(0),CV_FILLED);

    // Apply the elliptical mask on the face, to remove corners.
    // Sets corners to gray, without touching the inner face.
    face.setTo(Scalar(128), mask);
}

