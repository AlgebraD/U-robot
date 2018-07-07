#!/usr/bin/python
# coding=utf-8

import threading
import rospy
import json
from std_msgs.msg import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
import time
import datetime
import executor
# import sys
# reload(sys)
# sys.setdefaultencoding('utf8')

class ActionState():
    INIT = 0        #初始化
    PROCESS = 1     #正在执行过程中
    SUCCEED = 2     #执行成功
    FAILED  = 3     #执行失败
    TIMEOUT = 3     #执行超时

class TaskState():
    IDLE = 0        #空闲状态
    READY = 1       #准备执行
    PROCESS = 2     #正在执行过程中
    SUCCEED = 3     #执行成功
    FAILED  = 4     #执行失败
    TIMEOUT = 5     #执行超时

#动作基类
class Action():
    def __init__(self,dict):
        self.type = dict['action_type']    #0:MoveAction,1:SpeakAction
        self.description = dict['description']
        self.state = ActionState.INIT
        self.state_time = datetime.datetime.now()  #动作状态改变的时间 
        self.timeout = None #动作的执行超时秒数，在PROCESS状态下有用
        self.dict = dict

    def check_timeout(self):
        if ( self.state == ActionState.INIT or self.state > ActionState.PROCESS ) and \
            self.state != ActionState.TIMEOUT:
            return False
        elif self.state == ActionState.PROCESS:
            if self.timeout == None:
                return False
            else:
                
                sec = (datetime.datetime.now() - self.state_time).seconds
                if sec >= self.timeout :
                    self.change_state(ActionState.TIMEOUT)
                    return True
                else:
                    return False

    def change_state(self,state):
        self.state = state
        self.state_time = datetime.datetime.now()
        print '当前action状态改变为：%d' %(state)

#讲话动作
class SpeakAction(Action):
    def __init__(self,dict):
        Action.__init__(self,dict)
        self.say = dict['say']
        if 'listen' in dict:
            self.listen = dict['listen']
        else:
            self.listen = None

        if 'timeout' not in dict:
            self.timeout = 60 #默认超时时间为30秒
        else:
            self.timeout = dict['timeout']

    '''
    检测response是否是期望的回答
    response : asr文本
    '''
    def check_response(self,response):
        print 'check_response '+response
        patterns = self.listen
        # print self.listen
        text = response
        for pattern in patterns :
                items = pattern['items']
                if pattern['match'] == 'true' :#全字匹配
                    print 'match = true'
                    for item in items :
                        equals = item.split('|')
                        for equal in equals:
                            if equal == text :
                                return True
                                
                elif pattern['by_order'] == 'false':#items中的文本出现在text中
                    print 'by_order = false'
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
                        return True

                elif pattern['by_order'] == 'true':#items中的文本按顺序出现在text中
                    print 'by_order = true'
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
                        return True
                else:
                    print 'nothing'
        return False

#移动动作
class MoveAction(Action):
    def __init__(self,dict):
        Action.__init__(self,dict)     
        li = dict['pose'].strip().split(' ')
        if len(li) != 7 :
            print 'MoveAction pose format error'
            return
        self.pose = Pose() 
        self.pose.position.x = float(li[0])
        self.pose.position.y = float(li[1])
        self.pose.position.z = float(li[2])
        self.pose.orientation.x = float(li[3]) 
        self.pose.orientation.y = float(li[4])
        self.pose.orientation.z = float(li[5])
        self.pose.orientation.w = float(li[6])
        if 'timeout' not in dict:
            self.timeout = 120 #默认超时时间为120秒
        else:
            self.timeout = dict['timeout']

    #检测是否到达action的目标点，status为GoalStatus类型
    def check_arrived(self,status):
        if status != GoalStatus.SUCCEEDED:
            return False
        else:
            return True

class TaskType():
    TASK_IMMEDIATE = 0      #立即执行的任务
    TASK_TIMER = 1          #定时执行的任务

class Task():
    def __init__(self,dict):
        self.id = dict['id']
        self.description = dict['description']
        self.type = dict['type']    #0:立即执行的任务，1：定时执行的任务
        if 'time' in dict :
            dt = datetime.datetime.strptime(dict['time'], '%Y-%m-%d %H:%M:%S')  #读取定时任务的时间 
            self.time = dt

        self.actions = list()
        for action in dict['actions']:  #读取动作列表
            action_type = action['action_type']
            if action_type == 0:
                self.actions.append(MoveAction(action))
            elif action_type ==1:
                self.actions.append(SpeakAction(action))

        self.state = TaskState.IDLE #0:空闲，1:准备执行，2：执行中，3：执行成功，4：执行失败，5：超时
        if len(self.actions) > 0:
            self.cur_action = self.actions[0]
        self.action_index = 0  
        
    #执行下一个动作
    def next_action(self):
        self.cur_action.state = ActionState.SUCCEED 
        self.action_index += 1
        if self.action_index == len(self.actions):
            self.state = TaskState.SUCCEED  #任务的全部动作都已完成，设置任务成功状态
            print '任务完成:' + self.description
        elif self.action_index < len(self.actions):
            self.cur_action = self.actions[self.action_index]
            print '执行任务['+self.description+']下一动作\n'
            # print '执行任务下一动作'

    def init_task(self):
        self.state = TaskState.IDLE
        if len(self.actions) > 0:
            for action in self.actions:
                action.state = ActionState.INIT
            self.cur_action = self.actions[0]
        self.action_index = 0  

#任务通知
class TaskNotify():
    def __init__(self,type):
        self.type = type #0:asr识别通知，1:移动执行结果通知

#说话通知
class SpeakNotify(TaskNotify):
    def __init__(self,text):
        print 'create SpeakNotify'
        TaskNotify.__init__(self,0)
        self.text = text


#移动结果通知
class MoveNotify(TaskNotify):

    #status为GoalStatus类型
    def __init__(self,status):
        TaskNotify.__init__(self,1)
        self.status = status


#任务执行线程
class DoTaskThread(threading.Thread):

    def __init__(self,tasks_define_file):
        threading.Thread.__init__(self)

        self.listening = False  #当前是否等待语音应答
        self.threadLock = threading.RLock()    #任务list同步锁

        if tasks_define_file == None or tasks_define_file == '' :
            print 'tasks_define_file empty'
            return
        else:
            print 'tasks_define_file:' + tasks_define_file

        #读取任务配置文件
        try:
            tasks_file = open(tasks_define_file)
            content = tasks_file.read()
            tasks = json.loads(content) 
            tasks_file.close()

            self.notifys = list()
            self.tasks = list()
            for task in tasks:
                self.tasks.insert(0,Task(task))
        except:
            print 'read task definition file failed'

    '''
    向notifys列表未尾添加新的notify
    notify:任务通知，TaskNotify类型
    '''
    def send_notify(self,notify):
        self.threadLock.acquire()
        self.notifys.append(notify)
        print 'add notify'
        print len(self.notifys)
        self.threadLock.release()

    '''
    把notifys列表第一个notify移除掉
    notify:任务通知，TaskNotify类型
    '''
    def __remove_notify(self):
        if len(self.notifys) == 0:
            return
        self.threadLock.acquire()
        self.notifys.remove(self.notifys[0])
        self.threadLock.release()

    '''
    执行任务
    id:任务id
    '''
    def do_task(self,id):
        self.threadLock.acquire()
        ret = True
        for task in self.tasks:
            if task.id == id:
                if task.state == 0:
                    task.state = TaskState.READY #准备执行状态  
                    # print task.description
                    print '启动任务:' + task.description
                    break
                else:
                    print 'task is not in idle state,task id:%d' %(id)
                    ret = False
                    break
        self.threadLock.release()
        return ret

    #任务执行线程函数
    def run(self):
        

        while True:
            if len(self.notifys) > 0:
                notify = self.notifys[0]
                print 'recv notify'
            else:
                notify = None

            for task in self.tasks:
                if task.type == 1 and task.state == TaskState.IDLE : #定时任务
                    if task.time != None and datetime.datetime.now() > task.time : 
                        task.state = TaskState.READY    #定时任务到时，切换到准备执行状态
                    print '定时任务'
                elif task.type == 0: #立即任务
                    if task.state == TaskState.IDLE:
                        continue
                    elif task.state == TaskState.SUCCEED or \
                            task.state == TaskState.FAILED or \
                            task.state == TaskState.TIMEOUT:
                        task.init_task()
                        continue
                    elif task.state == TaskState.READY: #任务准备执行
                        task.state = TaskState.PROCESS
                        print '准备执行任务:' + task.description

                    elif task.state == TaskState.PROCESS:   #任务执行中
                        if task.cur_action.check_timeout():
                            task.state = TaskState.TIMEOUT
                            executor.Executor.speak('任务超时')
                            if task.cur_action.listen != None:
                                    self.listening = False
                            continue

                        if  isinstance(task.cur_action,SpeakAction):        #说话动作 
                            if task.cur_action.state == ActionState.INIT:
                                executor.Executor.speak(task.cur_action.say)
                                task.cur_action.change_state(ActionState.PROCESS)
                            elif task.cur_action.state == ActionState.PROCESS:
                                if task.cur_action.listen != None:
                                    self.listening = True
                                    if isinstance(notify,SpeakNotify): 
                                        if task.cur_action.check_response(notify.text): #检测说话通知
                                            self.listening = False
                                            task.cur_action.change_state(ActionState.SUCCEED)
                                            task.next_action()
                                else:
                                    task.cur_action.change_state(ActionState.SUCCEED)
                                    task.next_action()

                        elif isinstance(task.cur_action,MoveAction):    #移动动作
                            if task.cur_action.state == ActionState.INIT:
                                executor.Executor.send_move_goal(task.cur_action.pose)
                                task.cur_action.change_state(ActionState.PROCESS)
                            elif task.cur_action.state == ActionState.PROCESS:
                                if isinstance(notify,MoveNotify): 
                                    if task.cur_action.check_arrived(notify.status):    #检测是否动达
                                        task.cur_action.change_state(ActionState.SUCCEED)
                                        task.next_action()
                                    else:
                                        task.cur_action.change_state(ActionState.FAILED)
                                        task.state = TaskState.FAILED
                                        executor.Executor.speak('任务失败')
                                        continue

            self.__remove_notify()
            time.sleep(1)

        print '任务执行线程退出'