# UU_Robot

## 概述

### 简介

UUROBOT是一个集slam、听、说、看等功能于一体的室内三轮全向自主移动机器人，具备语音交互，视觉交互，自主导航定位，外部环境感知等功能。

### 开发环境

Ububtu 16.04 LTS
Python2.7/C++11
PyAIML ：Python中的AIML解释器
POS kinetic

### 功能介绍

#### 机器视觉

使用opencv实现物体识别，人脸识别，手势/动作识别，距离判断等功能。

#### 语音识别/语义理解

使用科大讯飞，百度，图灵机器人以及aiml等提供的语音识别和语义理解功能，使机器人能与人进行简单的对话和对人的指令做出响应。

#### 语音合成

使用科大讯飞或百度的语音合成功能。

#### 移动机器人

机器人采用轮式移动的方式，使用ros配合auduino的方式来控制舵机。

#### slam

使用现有开源的slam库实现自主导航建图功能。

#### 移动互联网

使用外界sim卡和wifi的方式接入移动互联网。

#### 其他传感

超声波，雷达，红外，陀螺仪，温湿度传感器。

#### LED

Led显示屏输出图像视频和文字。

### 模块组成

#### 主控板

负责运行ros系统以及整个上层功能代码，使用的是树莓派3b板子，搭载ubuntu mate系统。

####  底盘

负责整体承重和机器的移动，包含电机和电机驱动板等。
 
####  支架

负责上层承重和机器人形态的构建，以及为各模块的组装提供空间，包含亚克力板和六角铜柱等。

#### 电源

负责整个机器人各器件的供电，包含锂电池组和一个降压模块。
 
####  硬件驱动板

负责对硬件的驱动以及传感器数据的获取，使用的是arduino mega2560。

#### 音频模块

负责机器人的听与说，包含一个respeaker 6麦克的阵列，一个喇叭放音模块。

#### 视频模块

负责机器人的视觉功能，包含一个树莓派专用摄像头模块。
 
#### 雷达

为机器人的移动提供周围障碍物信息，包含一个rplidar模块。

#### 其他传感器

获取机器人自身或者周围环境状态的传感器件，包含电压传感器、超声波传感器、九轴惯性导航模块、温湿度传感器等。
 

### 典型代码及硬件分析

####  语音助手具体节点分析
 

 
####  离线语音识别

之前的语音对话，是你说一句，机器人回一句，不停的循环。为了实现特定词唤醒而不是简单的对话，我们选择使用离线识别等待命令词唤醒。用在线识别也可以实现这个效果，不过有一个非常大的缺点就是，既使是在静默状态下，也要不停地把本地的音频数据发往服务器做识别。这样就很浪费带宽，最重要的是有隐私泄露的问题，因为我们肯定只想在自已知情的情况下上传语音数据。
``
#pragma once
#ifndef POCKET_SPHINX_ASR 
#define POCKET_SPHINX_ASR

#include <iostream>
#include <pocketsphinx.h>
#include <sphinxbase/ad.h>
#include <pthread.h>
#include <unistd.h>

using namespace std;

typedef void(*psAsrResultFunc)(string) ;

class PocketSphinxAsr
{

private:
    pthread_t   tid;//asr线程id
    

public:
    //pocketsphinx使用变量 
    ps_decoder_t    *ps;        
    cmd_ln_t        *config;
    bool            stopThread;//控制线程结束标志   
    bool            isListening;//是否正在离线识别
    psAsrResultFunc asrCallbackFunc;//asr结果回调函数
    
public:
    PocketSphinxAsr(string hmmDir,string lmPath,string dictPath);
    virtual ~PocketSphinxAsr();

    /**
    开始离线语音识别
    asrResultFunc   :   离线语音误别结果
    */
    bool beginListen(psAsrResultFunc resultCallback);

    /**
    停止监听
    */
    void stopListen();

    // /**
    // asr线程识别出结果后，调用该函数
    // asrStr: asr识别结果
    // */
    // void getAsrResult(string asrStr);

private:
    void init(string hmmDir,string lmPath,string dictPath);
    

};

#endif

``

离线识别使用了cmu pocketsphinx的开源库，pocketsphinx支持很多种语言，我们做的是中文的识别，它对具体关键字的识别效果满足我们需求。
参照源码中的continuous.c的代码，我们先把离线语音识别集成到了机器人的语音功能当中，并和在线语音识别相配合。系统启动时，首先启动离线识别，当检测到唤醒命令词后，关闭离线识别并切换到在线识别。因为离线和在线识别，都要从麦克风读取数据，所以同一时间只能有一种识别模式。当不需要在线识别后，通过命令词切换回静默状态，由离线识别检测唤醒命令词。

#### 里程信息

里程信用用于估计机器人相对于原点的位置，为了达到好的精度，Riki Robot使用了两个里程源，机器人的线速度是根据电机上的编码器来计算的，并将计算的结果发布到/raw_vel结点上，而机器人的角速度则通过陀螺仪来控制，通过AHRS算法通过imu_filter_madgwick包过滤掉来自IMU的噪声，滤波后的输出发布到/imu/data，为需要机器人结点可靠的IMU数据，下面是/raw_imu中的数据发布到/imu/data/上 

 
#### 创建地图

创建地图，建图算法支持gmapping、hector算法，另外提供3种创建地图的方式，第一种是键盘创建地图，第二种是鼠标创建地图，第三种是通过选定区域自动创建地图。

##### Gampping
gmapping是一个比较完善的地图构建开源包，使用激光和里程计的数据来生成二维地图。

订阅主题：
tf (tf/tfMessage) 
scan (sensor_msgs/LaserScan)

发布主题：
map_metadata (nav_msgs/MapMetaData)
map (nav_msgs/OccupancyGrid) 
发布了地图的信息，比例，初始位置等。
需要注意的是，原ros里面有一个tf转换机制，所以需要用tf将各种数据的坐标系串联起来，变成一个树形结构（每个节点只能有一个父节点，可以有多个孩子节点），以便后面通讯和显示。根据ros官方文档的介绍，其实就是要将激光发布frame跟baselink和odom（里程计帧）连接起来 ，且里程计数据要转化成tf版本的里程计数据才可以使用。

比较重要的几个参数：

1. particles (int, default: 30) 这个参数决定gmapping算法中的粒子数，因为gmapping使用的是粒子滤波算法，粒子在不断地迭代更新，所以选取一个合适的粒子数可以让算法在保证比较准确的同时有较高的速度。 
2. minimumScore (float, default: 0.0) 最小匹配得分，这个参数很重要，它决定了你对激光的一个置信度，越高说明你对激光匹配算法的要求越高，激光的匹配也越容易失败而转去使用里程计数据，而设的太低又会使地图中出现大量噪声，所以需要大家好好调整。\

##### Hector
hector slam可以在没有里程计的情况下，以及展示俯仰的平台上使用建图
在hector_slam程序中，最重要的是hector_mapping节点。

订阅主题：

scan (sensor_msgs/LaserScan) ：激光雷达的扫描数据，通常由设备的运行的节点提供，例如：hokuyo node
syscommand (std_msgs/String) ： 系统命令，直接受“reset”，当接受该命令时，重设map frame 和robot 位置到初始的状态

发布主题：

map_metadata (nav_msgs/MapMetaData) ： 其余节点可以获取到map元数据，锁定并且定期更新
map (nav_msgs/OccupancyGrid) ： 其余节点可以获取到map数据，锁定并且定期更新
slam_out_pose (geometry_msgs/PoseStamped)：无协方差的预估机器人姿态
poseupdate (geometry_msgs/PoseWithCovarianceStamped) : 使用高斯预估不确定性的预估机器人姿态

比较重要的几个参数：

base_frame (string, default: base_link) ： 机器人基础坐标系的名称。这个坐标系用来定位和激光扫描器数据的改变(transformation)
map_frame (string, default: map_link) ：地图坐标系名称，通常是map，也就是所谓的全局world坐标系
odom_frame (string, default: odom)：里程计坐标系名称，默认是odom
map_resolution (double, default: 0.025) ： 地图分辨率，单位为m，一个网格单元的长度。默认为0.025
map_size (int, default: 1024) ：地图中一行网格的数量。地图是正方形的，并且有（map_size×map_size）个网格
