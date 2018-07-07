#!/usr/bin/python
# coding=utf-8
import os
import rospy
from actionlib import *
from actionlib_msgs.msg import *
from aivoice.msg import *
from aivoice.srv import *   
from face_rec.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import *
from nav_msgs.msg import *
import task
import cmd_handler
import sys
reload(sys)
sys.setdefaultencoding('utf8')

class Executor():

    task_thread     = None         #任务线程
    asr_ctrl_pub    = None         #asr控制主题发布
    tts_pub     = None              #tts主题发布器
    trans_serv  = None             #翻译service
    nlp_serv    = None              #自然语言理解service
    face_rec_action_client = None   #人脸识别action client
    move_base_goal_pub = None       #move_base移动目的发布器
    move_base_cancel_pub = None       #取消move_base移动的发布器    
    # move_base_action = None         #move_base action客户端

    odom    = None                  #当前底盘发布的odom
    map_frame   = None              #map frame id
    move_base_goal_id = None        #movebase的action goal id
    speaker_voice = None            #当前的发音人
    cmd_handler = None              #命令处理器
    enable_asr = True               #是否开启asr
    asr_online_mode = False         #asr是否是在线识别状态
    translate_type = 0              #0:无翻译，1：中译英，2：英译中。如果不为0，把识别出的文本全部翻译

    def __init__(self):
        pass

    @staticmethod
    def initialize():
        #读取配置文件 
        asr_result_topic =  rospy.get_param('~asr_result_topic','asr_result_topic')
        tts_topic   = rospy.get_param('~tts_topic','tts_topic')
        move_base_goal_topic  = rospy.get_param('~move_base_goal_topic','move_base_simple/goal')
        move_base_result_topic = rospy.get_param('~move_base_result_topic','move_base/result')
        # move_base_goal_topic = rospy.get_param('~move_base_goal_topic','move_base/goal')
        move_base_cancel_topic = rospy.get_param('~move_base_cancel_topic','move_base/cancel')
        move_base_status_topic = rospy.get_param('~move_base_status_topic','move_base/status')
        odom_topic = rospy.get_param('~odom_topic','odom')

        trans_service = rospy.get_param('~trans_service','translate_service')
        nlp_service   = rospy.get_param('~nlp_service','nlp_service')
        asr_ctrl_topic = rospy.get_param('~asr_ctrl_topic','asr_ctrl_topic')
        speak_notify_topic = rospy.get_param('~speak_notify_topic','speak_notify_topic')
        face_rec_action = rospy.get_param('~face_rec_action','face_rec_server')

        Executor.map_frame = rospy.get_param('~map_frame','map')
        Executor.speaker_voice = 'xiaokun'


        #订阅主题和创建主题发布者
        rospy.Subscriber(asr_result_topic,asr_result_msg,Executor.asr_result_callback)
        Executor.tts_pub = rospy.Publisher(tts_topic,tts_msg,queue_size=10)
        Executor.move_base_goal_pub = rospy.Publisher(move_base_goal_topic,PoseStamped,queue_size=10)
        Executor.move_base_cancel_pub = rospy.Publisher(move_base_cancel_topic,GoalID,queue_size=10)
        rospy.Subscriber(move_base_result_topic,MoveBaseActionResult,Executor.move_base_result_callback)
        rospy.Subscriber(move_base_status_topic,GoalStatusArray,Executor.move_base_status_callback)
        rospy.Subscriber(odom_topic,Odometry,Executor.odom_callback)

        Executor.asr_ctrl_pub = rospy.Publisher(asr_ctrl_topic,asr_ctrl_msg,queue_size=10)
        Executor.face_rec_action_client = SimpleActionClient(face_rec_action, face_recAction)
        # Executor.move_base_action = SimpleActionClient(move_base_goal_topic, MoveBaseAction)
        Executor.trans_serv = rospy.ServiceProxy(trans_service,trans)
        Executor.nlp_serv = rospy.ServiceProxy(nlp_service,nlp)

        #创建命令解析器
        Executor.cmd_handler = cmd_handler.CmdHandler(os.path.split(os.path.realpath(__file__))[0] + '/../cfg/commands.json')
        
        #创建并运行任务执行线程
        Executor.task_thread = task.DoTaskThread(os.path.split(os.path.realpath(__file__))[0] + '/../cfg/tasks.json')
        Executor.task_thread.run()

    #asr识别结果处理函数
    @staticmethod
    def asr_result_callback(msg) :
        print u'recv asr result', msg.text
        Executor.cmd_handler.handle_cmd(msg.text.decode('utf-8'))

    #move_base移动结果处理函数
    @staticmethod
    def move_base_result_callback(msg) :
        print u'recv move base result'+msg.status.text
        status = msg.status.status
        if status == GoalStatus.PREEMPTED:
            return
        elif status == GoalStatus.SUCCEEDED:
            Executor.speak('到达目标点')
        elif status == GoalStatus.ABORTED:
            Executor.speak('移动中断')
        else:
            Executor.speak('移动失败')   
        
        Executor.task_thread.send_notify( task.MoveNotify(status) )
        
    #move_base当前移动状态回调函数
    @staticmethod
    def move_base_status_callback(msg) :
        # print u'recv move base result', msg.status.text
        if len(msg.status_list) > 1:
            Executor.move_base_goal_id = msg.status_list[0].goal_id.id

    #asr识别结果处理函数
    @staticmethod
    def odom_callback(msg) :
        Executor.odom = msg

    '''
        让机器人说话
        content : 说话的内容
        voice : 发音人，默认使用当前全局发音人
        typw : tts的方式，0：百度，1：讯飞，默认为讯飞
    '''
    @staticmethod
    def speak(content,voice=None,type=1):
        if voice == None:
            voice = Executor.speaker_voice

        ttsMsg = tts_msg()
        ttsMsg.tts_type = type
        ttsMsg.text = content.decode('utf-8')
        ttsMsg.voice_name = voice.decode('utf-8')
        ttsMsg.text_encoding = 'utf8'
        Executor.tts_pub.publish(ttsMsg) 
        print 'say:%s' %(content)
        return

    '''
    调用翻译节点服务
    srcText : 要翻译的文本
    srcLan  : 源语言
    destLan : 目标语言
    返回 翻译后的文本
    '''
    @staticmethod
    def translate(srcText,srcLan,destLan):
        return Executor.trans_serv(srcText,srcLan,destLan)
        

    '''
    调用翻译节点服务
    text : 发送给nlp的文本
    返回 nlp处理后的文本
    '''
    @staticmethod
    def nlp(text):
        return Executor.nlp_serv(text)

    '''
    发送控制asr的消息
    order : 0:开启asr，1：关闭asr，2:切换到离线模式，3：切换到在线模式
    type：0：百度asr，1:讯飞asr，默认为1
    '''
    @staticmethod
    def ctrl_asr(order,type=1):
        asr_msg = asr_ctrl_msg()
        asr_msg.order = order
        asr_msg.asrType = 1
        Executor.asr_ctrl_pub.publish(asr_msg)

    '''
    人脸识别完成的回调函数
    status：actionlib_msgs/GoalStatus
    result：识别结果 face_recResult类型
    '''
    @staticmethod
    def face_rec_done_callback(status,result):
        if status == GoalStatus.SUCCEEDED:
            for index in range(len(result.person)):
                name = result.person[index]
                confidence = result.confidence[index]
                print '结识结果：name:'+name+' confidence:%s' %(confidence)
                if  name != None and name != '':
                    Executor.speak('你好'+name)

    '''
    调用人脸识别action
    '''
    @staticmethod
    def call_face_rec():
        goal = face_recGoal()
        goal.timeout = 5
        goal.rec_mode = 0
        Executor.face_rec_action_client.send_goal(goal,Executor.face_rec_done_callback)

    
    # '''
    # move_base移动结果回调函数
    # status：actionlib_msgs/GoalStatus
    # result：MoveBaseResult
    # '''
    # @staticmethod
    # def move_base_action_callback(status,result):
    #     status = msg.status.status
    #     if status == GoalStatus.PREEMPTED:
    #         return
    #     elif status == GoalStatus.SUCCEEDED:
    #         Executor.speak('到达目标点')
    #     elif status == GoalStatus.ABORTED:
    #         Executor.speak('移动中断')
    #     else:
    #         Executor.speak('移动失败')    

    '''
    向move_base发送移动坐标
    '''
    @staticmethod
    def send_move_goal(pose):

        pose_msg = PoseStamped()

        if isinstance(pose,Pose) :
            pose_msg.pose = pose
        else:
            li = pose.strip().split(' ')
            if len(li) != 7 :
                print 'move_base goal param format error'
                return
            pose_msg.pose.position.x = float(li[0])
            pose_msg.pose.position.y = float(li[1])
            pose_msg.pose.position.z = float(li[2])
            pose_msg.pose.orientation.x = float(li[3]) 
            pose_msg.pose.orientation.y = float(li[4])
            pose_msg.pose.orientation.z = float(li[5])
            pose_msg.pose.orientation.w = float(li[6])

        pose_msg.header.frame_id = Executor.map_frame
        pose_msg.header.stamp = rospy.Time.now()

        Executor.move_base_goal_pub.publish(pose_msg)
        # Executor.move_base_action.send_goal(goal,Executor.move_base_action_callback)

    '''
    向move_base发送停止移动命令
    '''
    @staticmethod
    def send_move_stop():

        msg = GoalID()
        msg.stamp = rospy.Time.now()
        msg.id = Executor.move_base_goal_id
        print 'send stop move cmd,goal_id: ' + msg.id.decode('utf-8')

        Executor.move_base_cancel_pub.publish(msg)