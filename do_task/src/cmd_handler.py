#!/usr/bin/python
# coding=utf-8

import json
# from executor.Executor import executor.Executor 
import executor
from aivoice.msg import *
import random
import task

class CmdHandler():
    def __init__(self,cmd_define_path):
        if cmd_define_path == None or cmd_define_path == '' :
            print 'cmd_define_path empty'
            return
        else:
            print 'cmd_define_path:' + cmd_define_path

        #读取命令配置文件
        try:
            cmd_file = open(cmd_define_path)
            content = cmd_file.read()
            self.cmd_def_list = json.loads(content) 
            cmd_file.close()
        except:
            print 'read commands definition file failed'

    #命令解析函数
    def parse(self,text):
        '''
            从输入的一段文本中解析出对应的命令，成功返回命令id，否则返回-1
        '''
        if text == None or text == '' :
            return -1

        for cmd_dict in self.cmd_def_list :
            patterns = cmd_dict['patterns']
            
            for pattern in patterns :
                items = pattern['items']
                if pattern['match'] == 'true' :#全字匹配
                    for item in items :
                        equals = item.split('|')
                        for equal in equals:
                            if equal == text :
                                return cmd_dict['command_id']
                                
                elif pattern['by_order'] == 'false':#items中的文本出现在text中
                    matched = True
                    for item in items:
                        equals = item.split('|')
                        contain = False
                        for equal in equals:
                            if equal in text:
                                contain = True
                                break
                        if not contain:
                            matched = False
                            break

                    if matched:
                        return cmd_dict['command_id']

                elif pattern['by_order'] == 'true':#items中的文本按顺序出现在text中
                    matched = True
                    start_index = 0
                    for item in items:
                        equals = item.split('|')
                        contain = False
                        for equal in equals:
                            find_index = text.find(equal,start_index)
                            if find_index != -1:
                                contain = True
                                start_index = find_index
                                break
                        if not contain:
                            matched = False
                            break

                    if matched:
                        return cmd_dict['command_id']                
        return -1

    

    #命令处理函数
    def handle_cmd(self,text):
        cmd_id = self.parse(text)

        if not executor.Executor.asr_online_mode : #asr离线识别模式
            if cmd_id == -1 :
                return
            elif cmd_id == 1:#唤醒
                # executor.Executor.send_move_goal('-1.06 0.05 0 0 0 0 1')
                executor.Executor.asr_online_mode = True
                executor.Executor.ctrl_asr(3) #切换到在线识别
                responder = ['是在叫我吗','在的','你好','在，请问有什么事']
                executor.Executor.speak(responder[random.randint(0,len(responder)-1)]) 
                # executor.Executor.speak('让我来仔细瞧一瞧')
                # executor.Executor.call_face_rec()
                return 
            elif cmd_id == 17: #停止移动
                # executor.Executor.task_thread.do_task(2)
                executor.Executor.send_move_stop()
            
        else:
            if executor.Executor.translate_type == 1:
                if cmd_id == 4:#停止翻译
                    executor.Executor.translate_type = 0
                    executor.Executor.speak('好的')
                    return
                else:
                    trans = executor.Executor.translate(text,'zh','en')
                    executor.Executor.speak(trans.result_text.decode('utf-8'),'catherine')
                    return
            if cmd_id == 2:#中译英
                executor.Executor.translate_type = 1
                executor.Executor.speak('ok','catherine')
                return
            elif cmd_id == 3:#英译中
                executor.Executor.translate_type = 2
                executor.Executor.speak('好的')
                return
            elif cmd_id == 4:#停止翻译
                executor.Executor.translate_type = 0
                executor.Executor.speak('好的')
                return
            elif cmd_id == 5:#讲普通话男声
                executor.Executor.speaker_voice = 'yufeng'
                executor.Executor.speak('好的')
                return
            elif cmd_id == 6:#讲普通话女声
                executor.Executor.speaker_voice = 'jinger'
                executor.Executor.speak('好的')
                return
            elif cmd_id == 7:#讲广东话
                executor.Executor.speaker_voice = 'xiaomei'
                executor.Executor.speak('好的')
                return
            elif cmd_id == 8:#讲四川话
                executor.Executor.speaker_voice = 'xiaorong'
                executor.Executor.speak('好的')
                return
            elif cmd_id == 9:#讲湖南话
                executor.Executor.speaker_voice = 'xiaoqiang'
                executor.Executor.speak('好的')
                return
            elif cmd_id == 10:#讲台湾话
                executor.Executor.speaker_voice = 'xiaolin'
                executor.Executor.speak('好的')
                return
            elif cmd_id == 11:#讲河南话
                executor.Executor.speaker_voice = 'xiaokun'
                executor.Executor.speak('好的')
                return
            elif cmd_id == 12:#讲东北话
                executor.Executor.speaker_voice = 'xiaoqian'
                executor.Executor.speak('好的')
                return
            elif cmd_id == 13:#讲童声
                executor.Executor.speaker_voice = 'nannan'
                executor.Executor.speak('好的')
                return
            elif cmd_id == 14:#讲英语
                executor.Executor.speaker_voice = 'catherine'
                executor.Executor.speak('ok')
                return
            elif cmd_id == 15:#待机
                executor.Executor.asr_online_mode = False
                executor.Executor.ctrl_asr(2) #切换到离线识别
                responder = ['已待机','好的，有事您叫我','静默，想我的时候请叫我的名字优优','您忙，我也去休息下']
                executor.Executor.speak(responder[random.randint(0,len(responder)-1)]) 
                return
            elif cmd_id == 16:#人脸识别
                executor.Executor.speak('让我来仔细瞧一瞧')
                executor.Executor.call_face_rec()
                return
            elif cmd_id == 17: #停止移动
                # executor.Executor.task_thread.do_task(2)
                executor.Executor.send_move_stop()
                return
            # elif cmd_id == 18:#打水
            #     executor.Executor.task_thread.do_task(1) 
            #     return
            elif cmd_id == 19:#拿啤酒
                executor.Executor.task_thread.do_task(1) 
                return
            else:
                # task_thread = 
                if executor.Executor.task_thread.listening :    #如果任务正在等待语音应答，优先处理asr结果
                    executor.Executor.task_thread.send_notify(task.SpeakNotify(text))
                else:
                    print 'nlp request',text
                    response = executor.Executor.nlp(text)
                    print 'nlp response:',response
                    executor.Executor.speak(response.result_text.decode('utf-8'))
                return
        