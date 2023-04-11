#!/usr/bin/env python
#coding=utf-8

import rospy
import numpy as np
from Python_API import Sendmessage
import time

"參數表"
class SP_API:
    def __init__(self):
        self.walk_status = 'First'                      #走路狀態 First:第一次進策略; Forward:前進; Back:後退
        self.head = 2600                                #頭部馬達角度
        self.speed = 0                                  #速度
        self.last_speed = 0                             #上次速度ˋ
        self.forward_start_speed = 6000                 #前進初速度
        self.back_start_speed = -4000                   #後退初速度
        self.max_forward_speed = 8000                  #前進最快速度
        self.min_forward_speed = 6000                   #前進最快速度  
        self.max_back_start_speed = -8000              #後退最快速度
        self.speed_add = 100                            #速度增加量
        self.theta = 0                                  #旋轉量
        self.var_theta = 0                              #上一次旋轉量
        self.forward_theta = 0                          #前進YAw值補償
        self.back_theta = 0                             #後退YAw值補償
        self.target = 10000                             #目標面積
        self.Object_Left=np.array([0,0,0,0,0,0,0])      #左半目標物[xmax xmin ymax ymin size X Y]
        self.Object_Right=np.array([0,0,0,0,0,0,0])     #右半目標物[xmax xmin ymax ymin size x Y]
        self.Object=np.array([0,0,0])                   #目標物[X Y size]
        self.yaw = 0                                    #Yaw值
        self.pitch = 0                                  #Pitch值
        #計算程式速度用
        self.start = 0                                  
        self.end = 0
        self.FPS = 0
        self.back_start = 0
        self.update_time = 0
    def initial(self):                                  #初始化參數
        self.walk_status = 'First'                                
        self.head = 2600
        self.speed = 0
        self.last_speed = 0
        self.speed_add = 100
        self.theta = 0
        self.var_theta = 0
        self.Object_Left=np.array([0,0,0,0,0,0,0]) 
        self.Object_Right=np.array([0,0,0,0,0,0,0])
        self.Object=np.array([0,0,0])
        self.yaw = 0
        self.pitch = 0
        self.start = 0
        self.end = 0
        self.FPS = 0
        self.back_start = 0
        self.update_time = 0
    def print_data(self):                               #print
        print('============================')
        print('Walk Status : ',self.walk_status)
        print('Speed : ',self.speed)
        print('Theta : ',self.theta)
        print('Target Y : ',self.Object[1])
        print('Pitch : ',self.pitch)
        print('Yaw : ',self.yaw)
        print('FPS : ',1/self.FPS)
         
send = Sendmessage()
sp = SP_API()

def forward_control(): #前進控制
    if sp.yaw > 8:
        sp.theta = -2 
    elif sp.yaw < -8:
        sp.theta = 2
    else:
        sp.theta = 0
    if sp.theta*sp.var_theta < 0:
        sp.theta = 0
    sp.var_theta = sp.theta
    sp.theta += sp.forward_theta
    sp.speed += sp.speed_add 
    sp.speed = min(sp.max_forward_speed,sp.speed)
    sp.speed = max(sp.min_forward_speed,sp.speed)
    
def back_control(): #後退控制
    if sp.yaw > 8:
        sp.theta = -2 
    elif sp.yaw < -8:
        sp.theta = 2
    else:
        sp.theta = 0
    if sp.theta*sp.var_theta < 0:
        sp.theta = 0
    sp.var_theta = sp.theta
    sp.theta += sp.back_theta
    if time.time() - sp.back_start > 2: 
        sp.speed += sp.speed_add
    sp.speed = max(sp.max_back_start_speed,sp.speed)
     
def find_target():  #找目標物
    sp.Object_Left=np.array([0,0,0,0,0,0,0])
    sp.Object_Right=np.array([0,0,0,0,0,0,0])
    for j in range (send.color_mask_subject_cnts[0]):
        if send.color_mask_subject_size[0][j] > 1000 and send.color_mask_subject_size[0][j] > sp.Object_Left[4]:
            sp.Object_Left[0]=send.color_mask_subject_XMin[0][j]
            sp.Object_Left[1]=send.color_mask_subject_XMax[0][j]
            sp.Object_Left[2]=send.color_mask_subject_YMin[0][j]
            sp.Object_Left[3]=send.color_mask_subject_YMax[0][j]
            sp.Object_Left[4]=send.color_mask_subject_size[0][j]
            sp.Object_Left[5]=send.color_mask_subject_X[0][j]
            sp.Object_Left[6]=send.color_mask_subject_Y[0][j]
            send.drawImageFunction(1,1,sp.Object_Left[0],sp.Object_Left[1],sp.Object_Left[2],sp.Object_Left[3],255,0,0) 
    for j in range (send.color_mask_subject_cnts[2]):
        if send.color_mask_subject_size[2][j] > 1000 and send.color_mask_subject_size[2][j] > sp.Object_Right[4]:
            sp.Object_Right[0]=send.color_mask_subject_XMin[2][j]
            sp.Object_Right[1]=send.color_mask_subject_XMax[2][j]
            sp.Object_Right[2]=send.color_mask_subject_YMin[2][j]
            sp.Object_Right[3]=send.color_mask_subject_YMax[2][j]
            sp.Object_Right[4]=send.color_mask_subject_size[2][j]  
            sp.Object_Right[5]=send.color_mask_subject_X[2][j]
            sp.Object_Right[6]=send.color_mask_subject_Y[2][j]
            send.drawImageFunction(2,1,sp.Object_Right[0],sp.Object_Right[1],sp.Object_Right[2],sp.Object_Right[3],0,0,255)
    
    if sp.Object_Right[0]>sp.Object_Left[0] and sp.Object_Right[1]>sp.Object_Left[1] and sp.Object_Left[4] > 1000 and sp.Object_Right[4] > 1000: #and 0.5<(sp.Object_Left[3]-sp.Object_Left[2])/(sp.Object_Right[1]-sp.Object_Left[0])<1.5:
        sp.Object[0] = (sp.Object_Left[5]+sp.Object_Right[5])/2
        sp.Object[1] = (sp.Object_Left[6]+sp.Object_Right[6])/2
        if sp.Object[1] >= 40:
            sp.speed_add = -100
    else:
        sp.Object[0] = 0
        sp.Object[1] = 0
        sp.speed_add = 100
    #找到目標物
    if sp.Object[1] >= 100:
        sp.walk_status = 'Back'
        sp.speed = sp.back_start_speed
        send.sendHeadMotor(2,2400,50)
        time.sleep(0.02)      
        sp.back_start = time.time()

def get_FPS():
    sp.end = time.time()
    sp.FPS = sp.end-sp.start

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():                                  
            if send.is_start == True:    
                sp.start = time.time()                
                if sp.walk_status == 'First':  
                    sp.update_time = time.time()
                    send.sendHeadMotor(2,sp.head,50)
                    send.sendSensorReset()
                    time.sleep(0.1)   
                    sp.initial()
                    send.sendBodyAuto(0,0,0,0,1,0)
                    sp.speed = sp.forward_start_speed
                    sp.walk_status='Forward'
                elif sp.walk_status == 'Forward':  
                    find_target()                                                      
                    forward_control()
                elif sp.walk_status == 'Back':
                    back_control()
                sp.pitch=send.imu_value_Pitch
                sp.yaw=send.imu_value_Yaw
                get_FPS()
                if sp.start - sp.update_time >= 0.1:
                    send.sendContinuousValue(sp.speed,0,0,sp.theta,0)
                    sp.update_time = time.time()
                    sp.print_data()    
            elif send.is_start == False:
                if sp.walk_status != 'First':
                    send.sendBodyAuto(0,0,0,0,1,0)
                    send.sendHeadMotor(2,sp.head,50)
                    sp.initial()
                    time.sleep(2) 
    except rospy.ROSInterruptException:
            pass