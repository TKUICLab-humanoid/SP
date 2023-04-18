#!/usr/bin/env python
#coding=utf-8

import rospy
import numpy as np
from Python_API import Sendmessage
import time

send = Sendmessage()

class SP_API:
    def __init__(self):
        self.init()

    def init(self):
        self.walk_status = 'First'                      #走路狀態 First:第一次進策略; Forward:前進; Back:後退
        self.head = 2600                                #頭部馬達角度
        self.speed = 0                                  #速度
        self.last_speed = 0                             #上次速度ˋ
        self.forward_start_speed = 6000                 #前進初速度
        self.back_start_speed = -4000                   #後退初速度
        self.max_forward_speed = 10000                  #前進最快速度
        self.min_forward_speed = 6000                   #前進最快速度  
        self.max_back_speed = -10000              #後退最快速度
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
        #計算程式速度用
        self.FPS = 0
        self.back_start = 0
        self.update_time = 0

    def print_data(self):                               #print
        print('============================')
        print('Walk Status : ',self.walk_status)
        print('Speed : ',self.speed)
        print('Theta : ',self.theta)
        print('Target Y : ',self.Object[1])
        print('Yaw : ',self.yaw)
        print('FPS : ',1/self.FPS)

    def forward_control(self): #前進控制
        if self.yaw > 8:
            self.theta = -2 
        elif self.yaw < -8:
            self.theta = 2
        else:
            self.theta = 0
        if self.theta*self.var_theta < 0:
            self.theta = 0
        self.var_theta = self.theta
        self.theta += self.forward_theta
        self.speed += self.speed_add 
        self.speed = min(self.max_forward_speed,self.speed)
        self.speed = max(self.min_forward_speed,self.speed)
        
    def back_control(self): #後退控制
        if self.yaw > 8:
            self.theta = -2 
        elif self.yaw < -8:
            self.theta = 2
        else:
            self.theta = 0
        if self.theta*self.var_theta < 0:
            self.theta = 0
        self.var_theta = self.theta
        self.theta += self.back_theta
        if time.time() - self.back_start > 2: 
            self.speed += self.speed_add
        self.speed = max(self.max_back_speed,self.speed)
        
    def find_target(self):  #找目標物
        self.Object_Left=np.array([0,0,0,0,0,0,0])
        self.Object_Right=np.array([0,0,0,0,0,0,0])
        for j in range (send.color_mask_subject_cnts[0]):
            if send.color_mask_subject_size[0][j] > 1000:
                self.Object_Left[0]=send.color_mask_subject_XMin[0][j]
                self.Object_Left[1]=send.color_mask_subject_XMax[0][j]
                self.Object_Left[2]=send.color_mask_subject_YMin[0][j]
                self.Object_Left[3]=send.color_mask_subject_YMax[0][j]
                self.Object_Left[4]=send.color_mask_subject_size[0][j]
                self.Object_Left[5]=send.color_mask_subject_X[0][j]
                self.Object_Left[6]=send.color_mask_subject_Y[0][j]
                send.drawImageFunction(1,1,self.Object_Left[0],self.Object_Left[1],self.Object_Left[2],self.Object_Left[3],255,0,0) 
        for j in range (send.color_mask_subject_cnts[2]):
            if send.color_mask_subject_size[2][j] > 1000:
                self.Object_Right[0]=send.color_mask_subject_XMin[2][j]
                self.Object_Right[1]=send.color_mask_subject_XMax[2][j]
                self.Object_Right[2]=send.color_mask_subject_YMin[2][j]
                self.Object_Right[3]=send.color_mask_subject_YMax[2][j]
                self.Object_Right[4]=send.color_mask_subject_size[2][j]  
                self.Object_Right[5]=send.color_mask_subject_X[2][j]
                self.Object_Right[6]=send.color_mask_subject_Y[2][j]
                send.drawImageFunction(2,1,self.Object_Right[0],self.Object_Right[1],self.Object_Right[2],self.Object_Right[3],0,0,255)
        
        if self.Object_Right[0]>self.Object_Left[0] and self.Object_Right[1]>self.Object_Left[1]: #and 0.5<(self.Object_Left[3]-self.Object_Left[2])/(self.Object_Right[1]-self.Object_Left[0])<1.5:
            self.Object[0] = (self.Object_Left[5]+self.Object_Right[5])/2
            self.Object[1] = (self.Object_Left[6]+self.Object_Right[6])/2
            if self.Object[1] >= 40:
                self.speed_add = -100
        else:
            self.Object[0] = 0
            self.Object[1] = 0
            self.speed_add = 100
        #找到目標物
        if self.Object[1] >= 185:
            self.walk_status = 'Back'
            self.speed = self.back_start_speed
            send.sendHeadMotor(2,2400,50)
            time.sleep(0.02)      
            self.back_start = time.time()

    def get_FPS(self,start,end):
        self.FPS = end-start

if __name__ == '__main__':
    try:
        sp = SP_API()
        while not rospy.is_shutdown():                                  
            if send.is_start == True:    
                start = time.time()                
                if sp.walk_status == 'First':  
                    sp.update_time = time.time()
                    send.sendHeadMotor(2,sp.head,50)
                    send.sendSensorReset()
                    time.sleep(0.1)   
                    send.sendBodyAuto(0,0,0,0,1,0)
                    sp.speed = sp.forward_start_speed
                    sp.walk_status='Forward'
                elif sp.walk_status == 'Forward':  
                    sp.find_target()                                                      
                    sp.forward_control()
                elif sp.walk_status == 'Back':
                    sp.back_control()
                sp.yaw=send.imu_value_Yaw
                if start - sp.update_time >= 0.2:
                    send.sendContinuousValue(sp.speed,0,0,sp.theta,0)
                    sp.update_time = time.time()
                end = time.time()
                sp.get_FPS(start,end)  
                sp.print_data()
            elif send.is_start == False:
                if sp.walk_status != 'First':
                    sp.init()
                    send.sendBodyAuto(0,0,0,0,1,0)
                    send.sendHeadMotor(2,sp.head,50)
                    time.sleep(2)
    except rospy.ROSInterruptException:
        pass
