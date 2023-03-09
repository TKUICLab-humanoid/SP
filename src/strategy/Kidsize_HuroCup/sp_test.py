#!/usr/bin/env python
#coding=utf-8

import rospy
import numpy as np
from Python_API import Sendmessage
import time

"常用參數表"
class SP_API:
    def __init__(self):
        self.get_target = False
        self.walk_status = 'First'              #走路狀態
        self.head = 1400                        #頭部馬達角度
        self.forward_speed = 2000               #前進初速度
        self.back_speed = -2000                 #後退初速度
        self.max_forward_speed = 2000           #前進最快速度
        self.min_forward_speed = 2000           #前進最慢速度  
        self.max_back_speed = -2000             #後退最快速度
        self.speed_add = 200                    #前進增加量
        self.speed_sub = 300                    #前進減速量
        self.bspeed_add = 100                   #後退增加量
        self.theta = 0                          #旋轉量
        self.var_theta = 0                      #上一次旋轉量
        self.forward_theta = 0                  #前進YAw值補償
        self.back_theta = 0                     #後退YAw值補償
        self.target = 10000                     #目標面積
        self.RedObject=np.array([0,0,0,0,0,0,0])    #紅色目標物[xmax xmin ymax ymin size X Y]
        self.BlueObject=np.array([0,0,0,0,0,0,0])   #藍色目標物[xmax xmin ymax ymin size x Y]
        self.Object=np.array([0,0,0])           #目標物[X Y size]
        self.yaw=0                              #Yaw值
        self.print_speed = ''
        self.print_theta = ''
        self.print_target = ''        
    def initial(self):
        self.walk_status = 'First'
        self.forward_speed = 2000        
        self.back_speed = -2000     
        self.head = 1400
        self.yaw = 0
        self.theta = 0
        self.var_theta = 0
        self.RedObject=np.array([0,0,0,0,0,0,0]) #xmin xmax ymin ymax size
        self.BlueObject=np.array([0,0,0,0,0,0,0])
        self.Object=np.array([0,0,0])
    def print_data(self):
        print('============================')
        print('Walk Status = ',self.walk_status)
        print('Speed = ',self.print_speed)
        print('Theta = ',self.print_theta)
        print('Theta = ',self.theta)
        print('Target = ',self.Object[1])
        print('Yaw = ',self.yaw)
         
send = Sendmessage()
sp = SP_API()

def forward_control(): #前進YAW值調整
    if sp.yaw > 8:
        sp.print_theta = '==========>'
        sp.theta = -2 
    elif sp.yaw < -8:
        sp.print_theta = '<========='
        sp.theta = 2
    if sp.theta*sp.var_theta < 0:
        sp.theta = 0
        sp.print_theta = ''
    sp.var_theta = sp.theta
    sp.forward_speed += sp.speed_add 
    time.sleep(0.05)
    sp.forward_speed = min(sp.max_forward_speed,sp.forward_speed)
        
def back_control(): #前進YAW值調整
    if sp.yaw > 8:
        sp.print_theta = '==========>'
        sp.theta = -2 
    elif sp.yaw < 8:
        sp.print_theta = '<========='
        sp.theta = 2
    if sp.theta*sp.var_theta < 0:
        sp.theta = 0
        sp.print_theta = ''
    sp.var_theta = sp.theta
    sp.back_speed -= sp.bspeed_add
    time.sleep(0.05)
    sp.back_speed=max(sp.max_back_speed,sp.back_speed)
     
def find_target():
    for j in range (send.color_mask_subject_cnts[0]):
        if 80 < send.color_mask_subject_X[0][j] < 240 and send.color_mask_subject_size[0][j] > 500:
            sp.RedObject[0]=send.color_mask_subject_XMin[0][j]
            sp.RedObject[1]=send.color_mask_subject_XMax[0][j]
            sp.RedObject[2]=send.color_mask_subject_YMin[0][j]
            sp.RedObject[3]=send.color_mask_subject_YMax[0][j]
            sp.RedObject[4]=send.color_mask_subject_size[0][j]
            sp.RedObject[5]=send.color_mask_subject_X[0][j]
            sp.RedObject[6]=send.color_mask_subject_Y[0][j]
            send.drawImageFunction(4,1,sp.RedObject[0],sp.RedObject[1],sp.RedObject[2],sp.RedObject[3],50,205,50) 
    for j in range (send.color_mask_subject_cnts[2]):
        if 80 < send.color_mask_subject_X[2][j] < 240 and send.color_mask_subject_size[2][j] > 500:
            sp.BlueObject[0]=send.color_mask_subject_XMin[2][j]
            sp.BlueObject[1]=send.color_mask_subject_XMax[2][j]
            sp.BlueObject[2]=send.color_mask_subject_YMin[2][j]
            sp.BlueObject[3]=send.color_mask_subject_YMax[2][j]
            sp.BlueObject[4]=send.color_mask_subject_size[2][j]  
            sp.BlueObject[5]=send.color_mask_subject_X[2][j]
            sp.BlueObject[6]=send.color_mask_subject_Y[2][j]
            send.drawImageFunction(5,1,sp.BlueObject[0],sp.BlueObject[1],sp.BlueObject[2],sp.BlueObject[3],80,50,205)
    if sp.BlueObject[0]>sp.RedObject[0] and sp.BlueObject[1]>sp.RedObject[1] and 0.5<(sp.RedObject[3]-sp.RedObject[2])/(sp.BlueObject[1]-sp.RedObject[0])<1.5:
        sp.Object[0] = (sp.RedObject[5]+sp.BlueObject[5])/2
        sp.Object[1] = (sp.RedObject[6]+sp.BlueObject[6])/2
        sp.get_target=True
    else:
        sp.Object[0] = 0
        sp.Object[1] = 0
        sp.get_target=False

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():                 
            # send.drawImageFunction(1,0,160,160,0,240,0,0,0)
            # send.drawImageFunction(2,0,0,320,120,120,0,0,0)                    
            if send.is_start == True:     
                sp.yaw=send.imu_value_Yaw
                if sp.walk_status == 'First':  
                    send.sendHeadMotor(2,sp.head,100)
                    time.sleep(0.1)   
                    sp.initial()
                    send.sendSensorReset()
                    sp.yaw=send.imu_value_Yaw
                    send.sendBodyAuto(0,0,0,0,1,0)
                    time.sleep(0.1)
                    sp.walk_status='Forward'
                elif sp.walk_status == 'Forward':  
                    find_target()
                    if sp.Object[1] >= 120:
                        sp.walk_status = 'Back'                                        
                    forward_control()
                    send.sendContinuousValue(sp.forward_speed,0,0,sp.theta+sp.forward_theta,0)
                    sp.print_speed = sp.forward_speed
                elif sp.walk_status == 'Back':
                    send.sendHeadMotor(2,2047,100)
                    back_control()
                    send.sendContinuousValue(sp.back_speed,0,0,sp.theta+sp.back_theta,0)
                    sp.print_speed = sp.back_speed
                sp.print_data()               
            elif send.is_start == False:
                if sp.walk_status != 'First':
                    send.sendBodyAuto(0,0,0,0,1,0)
                    time.sleep(0.1)
                    send.sendHeadMotor(2,2047,100)
                    time.sleep(0.1)
                    sp.initial()     
                sp.print_data()
                time.sleep(0.1)
    except rospy.ROSInterruptException:
            pass