#!/usr/bin/env python
# coding=utf-8

""" do_task_node.py - Version 1.1 2013-12-20

    recv voice command,then do specified task

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import os
import rospy
from executor import *
from cmd_handler import *
# import parse_cmd
from std_msgs.msg import *
from aivoice.msg import *
from aivoice.srv import *        
from face_rec import *      

# #asr识别结果处理函数
# def asr_result_callback(msg) :
#     print u'recv asr result', msg.text
#     Executor.cmd_handler.handle_cmd(msg.text.decode('utf-8'))

#机器人说话状态通知
# def speak_notify_callback(msg):
#     if msg.speak_state == 0: #开始讲话，关闭asr
#         Executor.ctrl_asr(1)
#     else: #停止讲话，开启asr
#         if Executor.enable_asr:
#             Executor.ctrl_asr(0)        

if __name__ == '__main__':
    try:
        rospy.init_node('do_task_node',log_level=rospy.INFO)

        Executor.initialize()
        #创建全局的命令处理器
        # Executor.cmd_handler = CmdHandler(os.path.split(os.path.realpath(__file__))[0] + '/../cfg/commands.json')
        # #读取配置文件 
        # asr_result_topic =  rospy.get_param('~asr_result_topic','asr_result_topic')
        # tts_topic   = rospy.get_param('~tts_topic','tts_topic')
        # trans_service = rospy.get_param('~trans_service','translate_service')
        # nlp_service   = rospy.get_param('~nlp_service','nlp_service')
        # asr_ctrl_topic = rospy.get_param('~asr_ctrl_topic','asr_ctrl_topic')
        # speak_notify_topic = rospy.get_param('~speak_notify_topic','speak_notify_topic')
        # face_rec_action = rospy.get_param('~face_rec_action','face_rec_server')

        # #创建全局的命令处理器
        # Executor.cmd_handler = CmdHandler(os.path.split(os.path.realpath(__file__))[0] + '/../cfg/commands.json')

        # #订阅主题和创建主题发布者
        # rospy.Subscriber(asr_result_topic,asr_result_msg,asr_result_callback)
        # # rospy.Subscriber(speak_notify_topic,speak_notify_msg,)
        # Executor.tts_pub = rospy.Publisher(tts_topic,tts_msg,queue_size=10)
        # Executor.asr_ctrl_pub = rospy.Publisher(asr_ctrl_topic,asr_ctrl_msg,queue_size=10)
        # Executor.face_rec_action_client = 
        # # rospy.wait_for_service(trans_service)
        # # rospy.wait_for_service(nlp_service)
        # Executor.trans_serv = rospy.ServiceProxy(trans_service,trans)
        # Executor.nlp_serv = rospy.ServiceProxy(nlp_service,nlp)

        rospy.spin()
        # rate = rospy.Rate(1)
        # while not rospy.is_shutdown():
        #     Executor.cmd_handler.handle_cmd(u'我说中文你说英文')
        #     rate.sleep()

    except:
        pass
        

        
