/*
 * Copyright (c) 2011. Philipp Wagner <bytefish[at]gmx[dot]de>.
 * Released to public domain under terms of the BSD Simplified license.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the organization nor the names of its contributors
 *     may be used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *   See <http://www.opensource.org/licenses/bsd-license>
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h> 
#include <image_transport/image_transport.h>

#include "opencv2/core.hpp"
#include "opencv2/face.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include "FaceRec/FaceDetector.h"
#include "FaceRec/FaceWorkThread.h"
#include "FaceRec/global.h"
#include "FaceRec/FaceRecognizer.h"
#include <limits> 



using namespace cv;
using namespace cv::face;
using namespace std;
using namespace ros; 


void face_rec_callback(const face_rec::face_recGoalConstPtr& goal, FaceRecServer* as);

void drawFixRect( Mat frame);
void image_process(Mat img);
void imageReceived(const sensor_msgs::ImageConstPtr& msg);

string cascadeName;
string nestedCascadeName;
VideoCapture capture(0);
Mat global_frame;

FaceRecServer *face_rec_server = NULL;
FaceRecServer::Result face_rec_result;
bool face_rec_completed = false;
//CFaceRecognizer  faceRec(faceDataPath,EIGEN);

#if defined(_WINDOWS)
	  // CFaceWorkThread gatherThread("E:\\project\\opencv\\FaceRec\\FaceData\\lbpcascade_frontalface_improved.xml",
      //  "E:\\project\\opencv\\FaceRec\\FaceData");
	CFaceWorkThread gatherThread("E:\\project\\opencv\\FaceRec\\FaceData\\lbpcascade_frontalface_improved.xml",
        "E:\\project\\opencv\\FaceRec\\FaceData");
#elif   defined(_LINUX)
//    CFaceDetector faceDetector("/usr/data/FaceData/haarcascade_frontalface_alt.xml");
    CFaceWorkThread gatherThread("data/FaceData/haarcascade_frontalface_alt.xml",
                                             "data/FaceData");
#endif

int main( int argc, char** argv )
{
	cout<<"启动人脸识别系统..."<<endl;
    // vector<Mat> images;
    // vector<int> labels;
    // vector<string> names;
//    read_csv("/usr/data/FaceData/faces.csv",images,labels,names);

    init(argc,argv,"face_rec_node");

    ros::NodeHandle nh; //定义ROS句柄  
    image_transport::ImageTransport it_(nh); //定义一个image_transport实例  
    image_transport::Subscriber image_sub_; //定义ROS图象接收器  
    //image_transport::Publisher image_pub_; //定义ROS图象发布器  
    image_sub_ = it_.subscribe("/image_raw",10,imageReceived);

    cvNamedWindow("FaceRec",CV_WINDOW_AUTOSIZE);

    FaceRecServer frs(nh, "face_rec_server", boost::bind(&face_rec_callback, _1, &frs), false);
    face_rec_server = &frs;
    face_rec_server->start();
    spin();

    return 0;
}


//在摄像头画面正中央画一个取景框
void drawFixRect( Mat frame){

    int w = frame.size().width;
    int h = frame.size().height;
    Point lt(w/4,h/4-20);
    Point rt(w*3/4,h/4-20);
    Point lb(w/4,h*3/4+20);
    Point rb(w*3/4,h*3/4+20);
    Scalar color(0,255,0);
    int lineWidth = 2;
    int lineLength = 20;
    line(frame,lt,Point(lt.x+lineLength,lt.y),color,lineWidth);
    line(frame,lt,Point(lt.x,lt.y+lineLength),color,lineWidth);

    line(frame,rt,Point(rt.x-lineLength,rt.y),color,lineWidth);
    line(frame,rt,Point(rt.x,rt.y+lineLength),color,lineWidth);

    line(frame,lb,Point(lb.x+lineLength,lb.y),color,lineWidth);
    line(frame,lb,Point(lb.x,lb.y-lineLength),color,lineWidth);

    line(frame,rb,Point(rb.x-lineLength,rb.y),color,lineWidth);
    line(frame,rb,Point(rb.x,rb.y-lineLength),color,lineWidth);

}

void imageReceived(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr; // 声明一个CvImage指针的实例  
  
        try  
        {  
            
            cv_ptr =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针  
        }  
        catch(cv_bridge::Exception& e)  //异常处理  
        {  
            ROS_ERROR("cv_bridge exception: %s", e.what());  
            return;  
        }  

        global_frame = cv_ptr->image;
        image_process(cv_ptr->image); //得到了cv::Mat类型的图象，在CvImage指针的image中，将结果传送给处理函数     
}


void image_process(Mat frame){
	// if(!capture.isOpened())
	// 	capture.open(0);
    // if( capture.isOpened() )
    // {
        // cout << "Video capturing has been started ..." << endl;
		
        // cout<<frame.size().width<<" "<<frame.size().height<<endl;

		int pastMSecs = clock()* 1000 / CLOCKS_PER_SEC;
       
			
            // capture >> frame;
            drawFixRect(frame);
            vector<Rect> rects = gatherThread.getGatheredFacesInfo().faceRectsInImg;
            if(rects.size() > 0 && gatherThread.getWorkThreadStatus() != IDLE){
                rectangle(frame,rects.back(),Scalar(255,0,0));
            }

            if( frame.empty() )
                return;


            imshow("FaceRec",frame);
			if(gatherThread.getWorkThreadStatus() != IDLE){
				vector<Mat> faces = gatherThread.getGatheredFacesInfo().gatheredFaces;
				if(faces.size()>0){
					imshow("采集图像",faces.back());
				}
			}
            // return;
// gatherThread.recognizeFaceFromCamera();
            char c = (char)waitKey(20);
			switch(c){
			case 'q':
				exit(0);
				break;
            case 'g'://开始采集
                cout<<"开始采集人脸"<<endl;
                gatherThread.beginGatherFaceFromCamera(Size(100,100),100);
				break;
            case 'c'://停止采集
            {
                int count = gatherThread.endGatherFaceFromCamera(); //停止采集
				cout << "共采集了 "<<count<<" 人脸!"<<endl;
				cout << "请输入人名 "<<endl;
				char name[30];
				cin.getline(name,30);
                //gatherThread.setGatherName(string("刘德华"));           //设置人名
				//gatherThread.setGatherName(string("刘亦菲"));           //设置人名
				//gatherThread.setGatherName(string("高圆圆"));           //设置人名
				gatherThread.setGatherName(name);           //设置人名
                ST_GATHER_FACE_INFO gatherInfo =  gatherThread.getGatheredFacesInfo();  //获取采集信息
                gatherThread.updateFaceDataInfo(gatherInfo.label,gatherInfo.name);      //更新人脸信息
                gatherThread.saveGatheredFacesToFile();                            //保存人脸信息到文件
                cout << "人脸数据已保存"<<endl;
				break;
            }
            case 'f':
            {

				char *filePath = "E:\\project\\opencv\\FaceRec\\FaceData\\test";
                cout<<"开始从文件采集人像："<<filePath<<endl;
                gatherThread.gatherFaceFromFiles(filePath,Size(100,100));
            }
                break;
            //case 't'://训练
//                cout<<"Begin train..."<<endl;
//                gatherThread.readFaceDataCsv();
//                faceRec.train(gatherThread.faceLabelInfo.faces,gatherThread.faceLabelInfo.labels);
//                cout<<"Train end!"<<endl;
               // break;
            case 'p'://识别
                gatherThread.recognizeFaceFromCamera();
				// gatherThread.recognizeFaceFromCameraPeriod(1000);
                break;
            case 's'://保存模型
                gatherThread.faceRecognize.saveTrainedModel();
                cout<<"模型已保存!"<<endl;
                break;
            case 't':
            {

                gatherThread.recognizeFaceFromFile("E:\\project\\opencv\\FaceRec\\FaceData\\test\\1.jpg");

//                gatherThread.recognizeFaceFromFile("/usr/data/face1/4.jpg");
//                gatherThread.recognizeFaceFromFile("/usr/data/face2/1.jpg");
//                gatherThread.recognizeFaceFromFile("/usr/data/face2/2.jpg");
//                gatherThread.recognizeFaceFromFile("/usr/data/face2/3.jpg");
//                gatherThread.recognizeFaceFromFile("/usr/data/face2/4.jpg");
            }
                break;
            case 'l'://加载模型
                gatherThread.faceRecognize.loadTrainedModel();
                cout<<"模型已加载!"<<endl;
                break;
            case 'u'://更新模型
            {

                ST_GATHER_FACE_INFO gatherInfo = gatherThread.getGatheredFacesInfo();

                gatherThread.faceRecognize.updateModel(gatherInfo.gatheredFaces,gatherInfo.label);
                cout<<"模型已更新!"<<endl;
                break;
            }
			case 'm':
			{
				char *path = "D:\\1.jpg";
				imwrite(path,frame);
				cout<<"截图成功，路径:"<<path<<endl;
			}
				break;
            default:
                break;
			}
  
    // }
}


void face_rec_callback(const face_rec::face_recGoalConstPtr& goal, FaceRecServer* as)  // Note: "Action" is not appended to DoDishes here
{
   int timeout = goal->timeout;
   int rec_mode = goal->rec_mode;
   time_t beginTime = time(NULL);
   face_rec_completed = false;

   face_rec_result.person.clear();
   face_rec_result.confidence.clear();

   gatherThread.recognizeFaceFromCamera();
   while(!face_rec_completed){
        usleep(10000);
        if(time(NULL)-beginTime > timeout){
            as->setAborted(face_rec_result,"识别超时");
            ROS_DEBUG("识别超时");
            return;
        }
   }

   if(face_rec_result.person.size() == 0){
            as->setAborted(face_rec_result,"识别失败");
            ROS_DEBUG("识别失败");
   }else{
            as->setSucceeded(face_rec_result,"识别成功");
            ROS_DEBUG("识别成功");
   }
   
        
   

//    FaceRecServer::Result rs;
//                                 rs.person.push_back("刘帅锋");
//                                 rs.confidence.push_back(100);
                                // as->setSucceeded(rs);
    // as->setPreempted();
}