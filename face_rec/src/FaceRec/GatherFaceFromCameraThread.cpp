#include "GatherFaceFromCameraThread.h"
#include "FaceDetector.h"
#include "global.h"

#ifdef _WINDOWS
#include <io.h>
#include <direct.h>
#endif

#ifdef _LINUX
//#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#endif

extern VideoCapture capture;

CFaceWorkThread::CFaceWorkThread(char* faceClassiferFilePath,
                                                         char *faceDataPath)
    :faceDetector(faceClassiferFilePath)
{
    this->faceDataPath = faceDataPath;
	b_exitThread	= false;
	gatherThread	= NULL;
	isGathering		= false;
}


CFaceWorkThread::~CFaceWorkThread(void){
	b_exitThread = true;
	delete gatherThread;
}


void CFaceWorkThread::operator()(){
//    VideoCapture vc(1);
//	if(!vc.isOpened())
//        vc.open(1);
	Mat frame;
    cvNamedWindow("Capture");

	while(!b_exitThread){
		if(isGathering){
            capture >> frame;

            if(frame.empty()){
                this_thread::sleep_for(chrono::milliseconds(800));//线程休眠100ms
                continue;
            }
//			vector<Rect> faces = faceDetector.detectFaces(frame);
            vector<Mat> faces;
            vector<Rect> rects = faceDetector.detectFacesAndPreprocess(frame,faces,faceSize);
            if(faces.size() > 0){
                for(int i=0;i<rects.size();i++){
                    Mat newFace = faces[i];
//					resize(newFace,newFace,faceSize);
                    if(gatheredFaces.size() > 0){
                        Mat lastFace = gatheredFaces.back();
                        if(calSimilarity(newFace,lastFace) > 0.3){
                            //如果新检测出的人脸和上一次有明显示姿态差异才加入vector，避免获取重复的人脸
                            gatheredFaces.push_back(newFace);
                            faceRectsInImg.push_back(rects[i]);
                        }
                    }else{
                        gatheredFaces.push_back(newFace);
                        faceRectsInImg.push_back(rects[i]);
                    }
                }
                imshow("Capture",faces[0]);
            }
		}
        this_thread::sleep_for(chrono::milliseconds(800));//线程休眠100ms
	}

}

void CFaceWorkThread::beginGatherFaceFromCamera(Size faceSize, int timeInterval){
	if(isGathering)
		return;
    assert(faceSize.width > 0 && faceSize.height > 0);
	timeInterval = timeInterval < 1000 ? 1000 : timeInterval;
    this->faceSize.width = faceSize.width;
    this->faceSize.height =faceSize.height;
	gatheredFaces.clear();
	faceRectsInImg.clear();
	isGathering = true;
	if(gatherThread == NULL)
		gatherThread = new thread(ref(*this));
}



int CFaceWorkThread::endGatherFaceFromCamera(){
    isGathering = false;
    return gatheredFaces.size();
}

//返回的数量有可能不是很准确，需要对采集线程做同步
void CFaceWorkThread::saveGatheredFacesToFile(char *name,int id){

    char path[256];
    char faceFilePath[256];
    char csvFilePath[256];
    char csvConent[256];
    time_t t;
    time(&t);


    //生成文件路径
#if defined( _WINDOWS )
    sprintf(path,"%s\\%llu",faceDataPath,(uint64)t);
    if(access(path,F_OK) != 0){
        int flag = _mkdir(path);
        if(flag != 0)
            logger.log("保存人脸图像时，创建文件夹失败!",LOG_WARN);
    }
    sprintf(csvFilePath,"%s\\faces.csv",faceDataPath);

#elif defined(_LINUX)

        sprintf(path,"%s/%llu",faceDataPath,t);
        if(access(path,F_OK) != 0){
            int flag =  mkdir(path,0777);
            if(flag != 0)
                logger.log("保存人脸图像时，创建文件夹失败!",LOG_WARN);
        }
        sprintf(csvFilePath,"%s/faces.csv",faceDataPath);
#endif

    //保存人脸图像并更新faces.csv文件
    fstream csvFile(csvFilePath,fstream::app);
    int faceLabel = getNewFaceLabel();
	for(int i=0;i<gatheredFaces.size();i++){
#if defined( _WINDOWS )
        sprintf(faceFilePath,"%s\\%d.jpg",path,i);
#elif defined(_LINUX)
        sprintf(faceFilePath,"%s/%d.jpg",path,i);
#endif
		Mat face =  gatheredFaces[i];
        imwrite(faceFilePath,face); //保存人脸图像文件

        sprintf(csvConent,"%s;%d;%s",faceFilePath,faceLabel,name);
        csvFile<<csvConent<<endl;   //更新cvs文件
	}

    csvFile.close();

}

double CFaceWorkThread::calSimilarity(Mat img1,Mat img2){

    CV_Assert( img1.rows == img2.rows && img1.cols == img2.cols && img1.type() == img2.type() );
	double L2 = norm(img1,img2,CV_L2); 
	double similarity = L2 / (double)(img1.rows*img1.cols);
	return similarity;
}

vector<Mat> CFaceWorkThread::getGatheredFaces(){
	return gatheredFaces;
}


int CFaceWorkThread::getNewFaceLabel(){
    char maxLabelFilePath[256];
    char maxLabel[100];
    int  newLabel = 0;


    //读取人脸标签值
#if defined( _WINDOWS )
        sprintf(maxLabelFilePath,"%s\\maxLabel.txt",faceDataPath);
#elif defined(_LINUX)
        sprintf(maxLabelFilePath,"%s/maxLabel.txt",faceDataPath);
#endif
    if(access(maxLabelFilePath,F_OK) == 0){
        ifstream maxLabelFileIn(maxLabelFilePath);
        maxLabelFileIn.getline(maxLabel,30);
        newLabel = atoi(maxLabel)+1;
        maxLabelFileIn.close();
    }

    //更新人脸标签
    sprintf(maxLabel,"%d",newLabel);
    ofstream maxLabelFileOut(maxLabelFilePath);
    maxLabelFileOut.write(maxLabel,strlen(maxLabel));
    maxLabelFileOut.close();
    return newLabel;

}
