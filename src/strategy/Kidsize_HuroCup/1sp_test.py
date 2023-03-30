#!/usr/bin/env python
#coding=utf-8

import rospy
import numpy as np
from Python_API import Sendmessage
import time

"常用參數表"
class SP_API:
    def __init__(self):
        self.walk_status = 'First'                      #走路狀態
        self.head = 2600                                #頭部馬達角度
        self.speed = 6000                               #初速度
        self.back_speed = -4000                         #後退初速度
        self.max_forward_speed = 11000                  #前進最快速度
        self.min_forward_speed = 6000                   #前進最快速度  
        self.max_back_speed = -12000                    #後退最快速度
        self.speed_add = 100                            #前進增加量
        self.theta = 0                                  #旋轉量
        self.var_theta = 0                              #上一次旋轉量
        self.forward_theta = 0                          #前進YAw值補償
        self.back_theta = 0                             #後退YAw值補償
        self.target = 10000                             #目標面積
        self.Left_Object=np.array([0,0,0,0,0,0,0])      #紅色目標物[xmax xmin ymax ymin size X Y]
        self.Right_Object=np.array([0,0,0,0,0,0,0])     #藍色目標物[xmax xmin ymax ymin size x Y]
        self.Object=np.array([0,0,0])                   #目標物[X Y size]
        self.yaw = 0                                    #Yaw值
        self.pitch = 0                                  #Pitch值
        self.start = 0
        self.end = 0
        self.FPS = 0
        self.back_start = 0
    def initial(self):
        self.walk_status = 'First'
        sp.speed = 6000       
        self.back_speed = -4000
        self.speed_add = 100                                
        self.head = 2600
        self.yaw = 0
        self.pitch = 0
        self.theta = 0
        self.var_theta = 0
        self.Left_Object=np.array([0,0,0,0,0,0,0]) 
        self.Right_Object=np.array([0,0,0,0,0,0,0])
        self.Object=np.array([0,0,0])
        self.start = 0
        self.end = 0
        self.FPS = 0
        self.back_start = 0
    def print_data(self):
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

def forward_control(): #前進YAW值調整
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
    
def back_control(): #後退YAW值調整
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
    sp.speed = max(sp.max_back_speed,sp.speed)
     
def find_target():
    sp.Left_Object=np.array([0,0,0,0,0,0,0])
    sp.Right_Object=np.array([0,0,0,0,0,0,0])
    for j in range (send.color_mask_subject_cnts[0]):
        if send.color_mask_subject_size[0][j] > 1000 and send.color_mask_subject_size[0][j] > sp.Left_Object[4]:
            sp.Left_Object[0]=send.color_mask_subject_XMin[0][j]
            sp.Left_Object[1]=send.color_mask_subject_XMax[0][j]
            sp.Left_Object[2]=send.color_mask_subject_YMin[0][j]
            sp.Left_Object[3]=send.color_mask_subject_YMax[0][j]
            sp.Left_Object[4]=send.color_mask_subject_size[0][j]
            sp.Left_Object[5]=send.color_mask_subject_X[0][j]
            sp.Left_Object[6]=send.color_mask_subject_Y[0][j]
            send.drawImageFunction(1,1,sp.Left_Object[0],sp.Left_Object[1],sp.Left_Object[2],sp.Left_Object[3],255,0,0) 
    for j in range (send.color_mask_subject_cnts[2]):
        if send.color_mask_subject_size[2][j] > 1000 and send.color_mask_subject_size[2][j] > sp.Right_Object[4]:
            sp.Right_Object[0]=send.color_mask_subject_XMin[2][j]
            sp.Right_Object[1]=send.color_mask_subject_XMax[2][j]
            sp.Right_Object[2]=send.color_mask_subject_YMin[2][j]
            sp.Right_Object[3]=send.color_mask_subject_YMax[2][j]
            sp.Right_Object[4]=send.color_mask_subject_size[2][j]  
            sp.Right_Object[5]=send.color_mask_subject_X[2][j]
            sp.Right_Object[6]=send.color_mask_subject_Y[2][j]
            send.drawImageFunction(2,1,sp.Right_Object[0],sp.Right_Object[1],sp.Right_Object[2],sp.Right_Object[3],0,0,255)
    if sp.Right_Object[0]>sp.Left_Object[0] and sp.Right_Object[1]>sp.Left_Object[1] and sp.Left_Object[4] > 1000 and sp.Right_Object[4] > 1000: #and 0.5<(sp.Left_Object[3]-sp.Left_Object[2])/(sp.Right_Object[1]-sp.Left_Object[0])<1.5:
        sp.Object[0] = (sp.Left_Object[5]+sp.Right_Object[5])/2
        sp.Object[1] = (sp.Left_Object[6]+sp.Right_Object[6])/2
        if sp.Object[1] >= 40:
            sp.speed_add = -100
    else:
        sp.Object[0] = 0
        sp.Object[1] = 0
        sp.speed_add = 100

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():                                  
            if send.is_start == True:    
                sp.start = time.time()
                sp.pitch=send.imu_value_Pitch
                sp.yaw=send.imu_value_Yaw
                if sp.walk_status == 'First':  
                    send.sendHeadMotor(2,sp.head,50)
                    send.sendSensorReset()
                    time.sleep(0.1)   
                    sp.initial()             
                    sp.pitch=send.imu_value_Pitch       
                    sp.yaw=send.imu_value_Yaw
                    send.sendBodyAuto(0,0,0,0,1,0)
                    sp.walk_status='Forward'
                elif sp.walk_status == 'Forward':  
                    find_target()                                                      
                    forward_control()
                    if sp.Object[1] >= 100:
                        sp.walk_status = 'Back'
                        sp.speed = sp.back_speed
                        send.sendHeadMotor(2,2400,50)
                        time.sleep(0.02)      
                        sp.back_start = time.time()
                elif sp.walk_status == 'Back':
                    back_control()
                send.sendContinuousValue(sp.speed,0,0,sp.theta,0)                  
                time.sleep(0.02)
                sp.end = time.time()
                sp.FPS = sp.end-sp.start
                sp.print_data()
                sp.start = 0
                sp.end = 0             
            elif send.is_start == False:
                if sp.walk_status != 'First':
                    send.sendBodyAuto(0,0,0,0,1,0)
                    send.sendHeadMotor(2,sp.head,50)
                    sp.initial() 
    except rospy.ROSInterruptException:
            pass