#ifndef _GLOBAL
#define _GLOBAL

#include "YYJSLogger.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <unistd.h>
#include <actionlib/server/simple_action_server.h>
#include <face_rec/face_recAction.h>

using namespace cv;
using namespace std;

typedef actionlib::SimpleActionServer<face_rec::face_recAction> FaceRecServer;

//#define _WINDOWS
#define _LINUX

/************全局变量****************/
extern CYYJSLogger logger;
extern FaceRecServer *face_rec_server;
extern FaceRecServer::Result face_rec_result; //识别action的请求结果
extern bool face_rec_completed; //识别请求是否完成

/************全局常量****************/
extern string faceDataPath;

/************宏****************/
#define IS_ZERO_RECT(r) ((r).size().width == 0 || (r).size().height == 0)   //是否是空矩形

/****************全局函数*****************/
/**
*	把源图转换为灰度图
*	@param	srcImg	源图
*/
void convertToGrayImg(Mat& srcImg);

/**
 * @brief 文件或目录是否存在
 * @param   path    路径
 * @return  存在返回true，不存在返回false
 */
bool isFileOrDicExist(const char *path);

/**
 * @brief 创建目录
 * @param path
 * @return  成功返回true，否则返回false
 */
bool createDic(const char *path);

/**
 * @brief 遍历目录下所有文件
 * @param path  目录路径
 * @param files 返回目录下所有文件的全路径名
 */
void listDir(const char *path,vector<String>& files);

#endif
