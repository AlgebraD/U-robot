#include "FaceWorkThread.h"
#include "FaceDetector.h"
#include "global.h"
#include <fstream>

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

//confidence 的阙值
#define confidenceThreshold 100

extern VideoCapture capture;
extern Mat global_frame;


CFaceWorkThread::CFaceWorkThread(char* faceClassiferFilePath,
                                                         char *faceDataPath)
    :faceDetector(faceClassiferFilePath),faceRecognize(faceDataPath,LBPH)
{
    this->faceDataPath = faceDataPath;
	b_exitThread	= false;
	gatherThread	= NULL;
    workStatus      = IDLE;
	faceSize		= Size(100,100);
    initialize();
	
}


CFaceWorkThread::~CFaceWorkThread(void){
	b_exitThread = true;
	delete gatherThread;
}

void CFaceWorkThread::initialize(){

    vector<Mat> faces;
    vector<int> labels;
    vector<string> names;
    readFaceDataCsv(faces,labels,names);
    faceDataInfo.labels = labels;
    faceDataInfo.names = names;

    //如果有保存的模型数据，先加载保存的模型，否则，重新使用图像数据训练模型
    if(!faceRecognize.loadTrainedModel()){

        faceRecognize.train(faces,labels);
    }
}




EN_WORK_THREAD_STATUS CFaceWorkThread::getWorkThreadStatus(){
    return workStatus;
}

void CFaceWorkThread::beginGatherFaceFromCamera(Size size, int timeInterval){
    if(workStatus != IDLE)
		return;
    if(size.width <= 0 || size.height <= 0)
        return;
    workStatus = GATHER_FROM_CAMERA;
	timeInterval = timeInterval < 0 ? 1000 : timeInterval;
    gatherTimeInterval = timeInterval;
    this->faceSize.width = size.width;
    this->faceSize.height =size.height;
    gatheredFaceInfos.clearInfo();
	if(gatherThread == NULL)
		gatherThread = new thread(ref(*this));
}



int CFaceWorkThread::endGatherFaceFromCamera(){
    workStatus = IDLE;
    gatheredFaceInfos.label = getNewFaceLabel();
    return gatheredFaceInfos.gatheredFaces.size();
}

void CFaceWorkThread::gatherFaceFromFiles(String path,Size size){
    if(workStatus != IDLE)
        return;
    if(size.width <= 0 || size.height <= 0)
        return;
    gatherFilePath = path;
    workStatus = GATHER_FROM_FILES;
    this->faceSize.width = size.width;
    this->faceSize.height =size.height;
    gatheredFaceInfos.clearInfo();
    if(gatherThread == NULL)
        gatherThread = new thread(ref(*this));
}


void CFaceWorkThread::setGatherName(string name){
    gatheredFaceInfos.name = name;
}

//返回的数量有可能不是很准确，需要对采集线程做同步
void CFaceWorkThread::saveGatheredFacesToFile(){

    char path[256];
    char faceFilePath[256];
    char csvFilePath[256];
    char csvConent[256];
    time_t t;
    time(&t);


    //生成文件路径
#if defined( _WINDOWS )
    sprintf(path,"%s\\data\\%llu",faceDataPath,(uint64)t);
	if(!isFileOrDicExist(path)){
        int flag = _mkdir(path);
        if(flag != 0)
            logger.log("保存人脸图像时，创建文件夹失败!",LOG_WARN);
    }
    sprintf(csvFilePath,"%s\\faces.csv",faceDataPath);

#elif defined(_LINUX)

        sprintf(path,"%s/data/%llu",faceDataPath,t);
        if(access(path,F_OK) != 0){
            int flag =  mkdir(path,0777);
            if(flag != 0)
                logger.log("保存人脸图像时，创建文件夹失败!",LOG_WARN);
        }
        sprintf(csvFilePath,"%s/faces.csv",faceDataPath);
#endif

    //保存人脸图像并更新faces.csv文件
    fstream csvFile(csvFilePath,fstream::app);

    for(int i=0;i<gatheredFaceInfos.gatheredFaces.size();i++){
#if defined( _WINDOWS )
        sprintf(faceFilePath,"%s\\%d.jpg",path,i);
#elif defined(_LINUX)
        sprintf(faceFilePath,"%s/%d.jpg",path,i);
#endif
        Mat face =  gatheredFaceInfos.gatheredFaces[i];
        imwrite(faceFilePath,face); //保存人脸图像文件

        sprintf(csvConent,"%s;%d;%s",faceFilePath,gatheredFaceInfos.label,gatheredFaceInfos.name.c_str());
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
		if(isFileOrDicExist(maxLabelFilePath)){
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

void CFaceWorkThread::recognizeFaceFromCamera(int timeOut){
    if(workStatus != IDLE || timeOut <= 0)
        return;
    recResult.results.clear();
    gatheredFaceInfos.clearInfo();
    faceRecTimeout  = timeOut;
    workStatus = RECOGNIZING_ONCE;
    if(gatherThread == NULL)
        gatherThread = new thread(ref(*this));
}

void CFaceWorkThread::recognizeFaceFromCameraPeriod(int period){
	
	if(workStatus != IDLE || period <= 0)
        return;
	recognizePeriodTime = period;
	recognizeInfoPeriod.clear();
	recResult.results.clear();
    gatheredFaceInfos.clearInfo();
    workStatus = RECOGNIZING_PERIOD;
    if(gatherThread == NULL)
        gatherThread = new thread(ref(*this));
}

void CFaceWorkThread::recognizeFaceFromFile(string filePath){
    Mat img = imread(filePath);
    convertToGrayImg(img);
    int label;
    double confidence;
//    faceRecognize.recognize(img,confidence,label);
//    string name = getNameFromLabel(label);
//    cout<<"识别结果 >> name:"<<name<<" label:"<<label<<" confidence:"<<confidence<<" path:"<<filePath<<endl;
    vector<Mat> faces;
    faceDetector.detectFacesAndPreprocess(img,faces,Size(100,100));
    if(faces.size() > 0){
        faceRecognize.recognize(faces[0],confidence,label);
        string name = getNameFromLabel(label);
        cout<<"识别结果 >> name:"<<name<<" label:"<<label<<" confidence:"<<confidence<<" path:"<<filePath<<endl;
    }else{
        cout<<"没有检测到人脸"<<endl;
    }

}

ST_GATHER_FACE_INFO CFaceWorkThread::getGatheredFacesInfo(){
    return gatheredFaceInfos;
}

void CFaceWorkThread::readFaceDataCsv(vector<Mat>& faces,vector<int>& labels,vector<string>& names){

    faceDataInfo.clearInfo();
        /****************读取人脸图像数据*****************/
        char filename[256];
    #if     defined(_WINDOWS)
        sprintf(filename,"%s\\faces.csv",faceDataPath);
    #elif   defined(_LINUX)
        sprintf(filename,"%s/faces.csv",faceDataPath);
    #endif
        ifstream file(filename, ifstream::in);
        if (!file) {
//            string error_message = "No valid input file was given, please check the given filename.";
            return;
        }
        string line, path, classlabel,name;
        char separator=';';
        while (getline(file, line)) {
            stringstream liness(line);
            getline(liness, path, separator);
            getline(liness, classlabel,separator);
            getline(liness, name,separator);
            if(!path.empty() && !classlabel.empty()) {
                faces.push_back(imread(path, 0));
                labels.push_back(atoi(classlabel.c_str()));
                names.push_back(name);
            }
        }

}

void CFaceWorkThread::updateFaceDataInfo(vector<int> labels,vector<string> names){
    if(labels.size() == 0 || labels.size() != names.size())
        return;

    for(int i=0;i<labels.size();i++){
        faceDataInfo.names.push_back(names[i]);
        faceDataInfo.labels.push_back(labels[i]);
    }

}

void CFaceWorkThread::updateFaceDataInfo(int label,string name){
    faceDataInfo.names.push_back(name);
    faceDataInfo.labels.push_back(label);
}

string CFaceWorkThread::getNameFromLabel(int label){

    for(int i = 0;i<faceDataInfo.labels.size();i++){
        if(faceDataInfo.labels[i] == label && i < faceDataInfo.names.size())
            return faceDataInfo.names[i];
    }
	return "未知";
}

void CFaceWorkThread::clearGatheredFaceInfo(){
    gatheredFaceInfos.clearInfo();
}

void CFaceWorkThread::operator()(){
    Mat frame;
    //cvNamedWindow("Capture");

    clock_t  beginRecTime;//开始人脸识别的时间，用于计算超时
    bool    beginRec = false;//是否开始人脸识别

    vector<Mat> faces;
    vector<Rect> rects;

	try{

    while(!b_exitThread){
        if(workStatus != IDLE){

            faces.clear();
            rects.clear();

            if(workStatus == GATHER_FROM_FILES){
                faces = faceDetector.detectFacesAndPreprocessFromFiles(gatherFilePath,faceSize);
                gatheredFaceInfos.gatheredFaces = faces;
                gatheredFaceInfos.label = getNewFaceLabel();
                workStatus = IDLE;
                cout<<"采集完成，共获得"<<faces.size()<<"张人像"<<endl;
                this_thread::sleep_for(chrono::milliseconds(100));//线程休眠100ms
                continue;
            }


            if(!beginRec && (workStatus == RECOGNIZING_ONCE || workStatus == RECOGNIZING_PERIOD)){
                beginRec = true;
				beginRecTime = clock();//记录开始人脸识别的时间
            }

            // capture >> frame;   //读取视频帧
            frame = global_frame;

            if(frame.empty()){
                this_thread::sleep_for(chrono::milliseconds(100));//线程休眠100ms
                continue;
            }

			clock_t tick_start	=	clock();

            rects = faceDetector.detectFacesAndPreprocess(frame,faces,faceSize);
            if(faces.size() > 0){
                vector<Mat>& gatheredFaces = gatheredFaceInfos.gatheredFaces;
                vector<Rect>& faceRectsInImg = gatheredFaceInfos.faceRectsInImg;

                for(int i=0;i<rects.size();i++){
                    Mat newFace = faces[i];

                    if(gatheredFaces.size() > 0){
                        Mat lastFace = gatheredFaces.back();
                        if(calSimilarity(newFace,lastFace) > 0.3)
						{
                            //如果新检测出的人脸和上一次有明显示姿态差异才加入vector，避免获取重复的人脸
                            gatheredFaces.push_back(newFace);
                            faceRectsInImg.push_back(rects[i]);
							cout<<"采集了"<<gatheredFaces.size()<<"人脸"<<endl;
                        }
                    }else{
                        gatheredFaces.push_back(newFace);
                        faceRectsInImg.push_back(rects[i]);
						cout<<"采集了"<<gatheredFaces.size()<<"人脸"<<endl;
                    }

                    if(workStatus == RECOGNIZING_ONCE){
                        ST_REC_RESULT result;   //人脸识别结果
                        faceRecognize.recognize(faces[i],result.confidence,result.label);
						if(result.label >= 0){
							recResult.status = REC_SUCCESS;
							recResult.results.push_back(result);
							cout<<"识别结果 >> name:"<<getNameFromLabel(result.label)<<" label:"<<result.label
							  <<" confidence:"<<result.confidence<<endl;

                            if(face_rec_server != NULL){
                                face_rec_result.person.push_back(getNameFromLabel(result.label));
                                face_rec_result.confidence.push_back(result.confidence);
                                face_rec_completed = true;
                            }
                            
						}
                        
                    }else if(workStatus == RECOGNIZING_PERIOD){
                        ST_REC_RESULT result;   
                        faceRecognize.recognize(faces[i],result.confidence,result.label);
                        cout<<"lable:"<<result.label<<" confidence:"<<result.confidence<<endl;
						if(result.label >= 0){
							recResult.results.push_back(result);
							recResult.status = REC_SUCCESS;
							bool newFace = true;
							vector<int> recInfo;
							for(int n=0;n<recognizeInfoPeriod.size();n++){
								recInfo = recognizeInfoPeriod[n];
								int recLabel	= recInfo[0];
								int count		= recInfo[1];
								if(recLabel == result.label){//更新之前识别的人脸的识别次数
									newFace = false;
									recInfo[1] = ++count;
                                    recInfo[2] += (int)result.confidence;
									recognizeInfoPeriod[n] = recInfo;
								}
							}
							if(newFace){//新识别的人脸，初始识别次数为1
								vector<int> recInfo;
								recInfo.push_back(result.label);
								recInfo.push_back(1);
                                recInfo.push_back((int)result.confidence);
								recognizeInfoPeriod.push_back(recInfo);
							}
						}
						
                    }
                }
                //imshow("Capture",faces[0]);
				//waitKey(30);
            }

				//cout<<"线程单次循环花费时间：" << (clock()-tick_start)*1000/CLOCKS_PER_SEC <<" ms"<<endl;
        }


		if(workStatus == GATHER_FROM_CAMERA){
			this_thread::sleep_for(chrono::milliseconds(gatherTimeInterval));
		}else if(workStatus == RECOGNIZING_ONCE){
            if(faces.size() > 0){//识别完人脸后，把线程状态置为空闲
                workStatus = IDLE;
                beginRec    = false;
            }

            //超时没有识别出人脸的话，取消识别

			if((clock()-beginRecTime)*1000/CLOCKS_PER_SEC > faceRecTimeout){
                workStatus = IDLE;
                beginRec = false;
                recResult.status    = REC_TIMEOUT;
                cout<<"识别超时"<<endl;
            }

		}else if(workStatus == RECOGNIZING_PERIOD){

			if((clock()-beginRecTime)*1000/CLOCKS_PER_SEC > recognizePeriodTime){
                workStatus = IDLE;
                beginRec = false;

				//找出识别次数最多的label
				vector<int> recInfo;
				int maxCount = 0;
				int label = -1;
                int confidence = 0;
				for(int n=0;n<recognizeInfoPeriod.size();n++){
					recInfo = recognizeInfoPeriod[n];

                    if(recInfo[1] > maxCount /*&& recInfo[2]/recInfo[1] < confidenceThreshold */){
						maxCount = recInfo[1];
						label = recInfo[0];
                        confidence = recInfo[2]/recInfo[1];
//						cout<<"label:"<<recInfo[0]<<" 识别次数："<<recInfo[1]<<endl;
					}
				}


				if(label >= 0){
					recResult.status    = REC_SUCCESS;
                    cout<<"识别结果 >> name:"<<getNameFromLabel(label)<<" label:"<<label<<" 识别次数："<<maxCount<<" confidence："<<confidence<<endl;

                }else if(recognizeInfoPeriod.size() == 0){
					recResult.status    = REC_NOBODY;
					cout<<"未检测到人脸"<<endl;
				}else if(label == -1){
					recResult.status    = REC_UNKNOWN_BODY;
//					cout<<"未识别的人脸"<<endl;
                    cout<<"你是不是整容了，我都不太确定你是谁了，你可能是下面中的某一位(0_0!)"<<endl;
                    for(int n=0;n<recognizeInfoPeriod.size();n++){
                        recInfo = recognizeInfoPeriod[n];
                        cout<<getNameFromLabel(recInfo[0])<<" confidence:"<<recInfo[2]/recInfo[1]<<endl;
                    }
                }
                
			}else{
				recResult.results.clear();//持续识别的时候，每一次识别前都清除掉上一次的结果 
			}

		}else
            this_thread::sleep_for(chrono::milliseconds(100));
    }
	}catch(...){
		gatherThread = NULL;
	}
}
