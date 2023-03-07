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
        self.walk_status = 'First'              #走路模式
        self.mode = 2                           #目標模式 1單色球 2雙色球
        self.head = 2600                        #頭部馬達初始角度
        self.forward_speed = 7000               #前進初速度
        self.back_speed = -7000                 #後退初速度
        self.max_forward_speed = 10000           #前進最快速度
        self.min_forward_speed = 10000           #前進最慢速度  
        self.max_back_speed = -9000             #後退最快速度
        self.speed_add = 200                    #前進增加量
        self.speed_sub = 300                    #前進減速量
        self.bspeed_add = 100                   #後退增加量
        self.theta = 0                          #旋轉量
        self.var_theta = 0
        self.forward_theta = 0                    #前進YAw值補償
        self.back_theta = 0                      #後退YAw值補償
        self.target = 9000                      #目標面積
        self.redmax = 2.0                       #1.15單色球
        self.redmin = 1.5                       #0.85單色球
        self.RedObject=np.array([0,0,0,0,0])    #紅色目標物[xmax xmin ymax ymin size]
        self.BlueObject=np.array([0,0,0,0,0])   #藍色目標物[xmax xmin ymax ymin size]
        self.yaw_start=0                        #Yaw值
        self.red_size=0                         #虹球面積
        self.blue_size=0                        #籃球面積
        self.ballsize=0                         #球總面積
        self.print_speed = ''
        self.print_theta = ''
        self.print_Direction = ''
        self.print_target = ''        
    def initial(self):
        self.walk_status = 'First'
        self.forward_speed = 7000        
        self.back_speed = -7000     
        self.head = 2600
        self.yaw_start=0
        self.theta=0
        self.var_theta = 0
        self.RedObject=np.array([0,0,0,0,0]) #xmin xmax ymin ymax size
        self.BlueObject=np.array([0,0,0,0,0])
        self.red_size=0
        self.blue_size=0
        self.ballsize=100 
    def print_data(self):
        print('============================')
        print('Direction = ',self.print_Direction)
        print('Walk Status = ',self.walk_status)
        print('Speed = ',self.print_speed)
        print('Theta = ',self.print_theta)
        print('Theta = ',self.theta)
        print('Target = ',self.print_target)
        print('Head = ',self.head)
        print('Yaw = ',self.yaw_start)
         

send = Sendmessage()
sp = SP_API()

def forward_control(): #前進YAW值調整
    if sp.yaw_start > 8:
        sp.print_theta = '==========>'
        sp.theta = -3 
    elif sp.yaw_start < -8:
        sp.print_theta = '<========='
        sp.theta = 2
    if sp.theta*sp.var_theta < 0:
        sp.theta = 0
        sp.print_theta = ''
    sp.var_theta = sp.theta
    if sp.ballsize<4000:
        sp.forward_speed += sp.speed_add 
        time.sleep(0.1)
        sp.forward_speed = min(sp.max_forward_speed,sp.forward_speed)
    else:#slow speed
        sp.forward_speed -= sp.speed_sub
        time.sleep(0.06)
        sp.forward_speed = max(sp.min_forward_speed,sp.forward_speed)
    
def back_control(): #前進YAW值調整
    if sp.yaw_start > 8:
        sp.print_theta = '==========>'
        sp.theta = -2 
    elif sp.yaw_start < 8:
        sp.print_theta = '<========='
        sp.theta = 2
    if sp.theta*sp.var_theta < 0:
        sp.theta = 0
        sp.print_theta = ''
    sp.var_theta = sp.theta
    sp.back_speed -= sp.bspeed_add
    time.sleep(0.05)
    sp.back_speed=max(sp.max_back_speed,sp.back_speed)

def redball():   #紅色球面積
    BestRed=[]
    for j in range (send.color_mask_subject_cnts[0]):
        if 80 < send.color_mask_subject_X[0][j] < 240:
            sp.RedObject[0]=send.color_mask_subject_XMin[0][j]
            sp.RedObject[1]=send.color_mask_subject_XMax[0][j]
            sp.RedObject[2]=send.color_mask_subject_YMin[0][j]
            sp.RedObject[3]=send.color_mask_subject_YMax[0][j]
            sp.RedObject[4]=send.color_mask_subject_size[0][j]
            send.drawImageFunction(4,1,sp.RedObject[0],sp.RedObject[1],sp.RedObject[2],sp.RedObject[3],50,205,50)  
            if sp.RedObject[4]>100:
                BestRed.append(sp.RedObject[4])
                BestRed.sort(reverse = True)
                sp.red_size = BestRed[0]
            else:
                sp.red_size = sp.RedObject[4]

def blueball():   #藍色球面積
    BestBlue=[]
    for j in range (send.color_mask_subject_cnts[2]):
        if 80 < send.color_mask_subject_X[2][j] < 240:
            sp.BlueObject[0]=send.color_mask_subject_XMin[2][j]
            sp.BlueObject[1]=send.color_mask_subject_XMax[2][j]
            sp.BlueObject[2]=send.color_mask_subject_YMin[2][j]
            sp.BlueObject[3]=send.color_mask_subject_YMax[2][j]
            sp.BlueObject[4]=send.color_mask_subject_size[2][j]  
            send.drawImageFunction(5,1,sp.BlueObject[0],sp.BlueObject[1],sp.BlueObject[2],sp.BlueObject[3],80,50,205)
            if sp.BlueObject[4]>100:
                BestBlue.append(sp.BlueObject[4])
                BestBlue.sort(reverse = True)
                sp.blue_size = BestBlue[0]
            else:
                sp.blue_size = sp.BlueObject[4]

def total():    #紅籃球總面積
    if sp.BlueObject[0]>sp.RedObject[0] and sp.BlueObject[1]>sp.RedObject[1] and 0.5<(sp.RedObject[3]-sp.RedObject[2])/(sp.BlueObject[1]-sp.RedObject[0])<1.5:
        sp.get_target=True
        sp.ballsize = sp.red_size+sp.blue_size
    else:
        sp.get_target=False
        sp.ballsize = 0
    sp.print_target = sp.ballsize

def movehead():    #頭部馬達調整
    vertical = (sp.RedObject[2]+sp.RedObject[3])/2-100
    temp = 40.0*vertical/240
    time.sleep(0.03)
    if abs(temp)>3:
        sp.head += 11.378*temp*0.4
        time.sleep(0.01)
        if sp.head<2048:
            sp.head=2048
        elif sp.head>2600:
            sp.head=2600
    send.sendHeadMotor(2,round(sp.head),50)
    time.sleep(0.01)  
     
if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():                 
            send.drawImageFunction(1,0,160,160,0,240,0,0,0)
            send.drawImageFunction(2,0,0,320,120,120,0,0,0)
            # send.drawImageFunction(3,1,40,280,40,200,0,0,0)                     
            if send.is_start == True:     
                sp.yaw_start=send.imu_value_Yaw
                if sp.walk_status == 'First':  
                    send.sendHeadMotor(2,sp.head,100)
                    time.sleep(0.1)   
                    sp.initial()
                    send.sendSensorReset()
                    send.sendBodyAuto(0,0,0,0,1,0)
                    time.sleep(0.1)
                    sp.walk_status='Forward'
                elif sp.walk_status == 'Forward':  
                    if sp.mode == 1:
                        redball()
                        sp.ballsize = sp.red_size
                        if sp.red_size==None:
                            sp.print_target = '===== no catch ball ====='   
                    elif sp.mode == 2:    
                        redball()
                        blueball()
                    # if sp.red_size==None and sp.blue_size!=None:
                    #     sp.red_size=sp.blue_size
                    # elif sp.blue_size==None and sp.red_size!=None:
                    #     sp.blue_size=sp.red_size
                        if sp.red_size==None and sp.blue_size==None:
                            sp.red_size=500
                            sp.blue_size=500   
                            sp.print_target = '===== no catch ball ====='
                        total()
                    if sp.get_target:
                        movehead()                    
                    if sp.ballsize >= sp.target:
                        sp.walk_status = 'Back'
                                        
                    forward_control()
                    send.sendContinuousValue(sp.forward_speed,0,0,sp.theta+sp.forward_theta,0)
                    sp.print_speed = sp.forward_speed
                    sp.print_Direction = 'move on'
                elif sp.walk_status == 'Back':
                    send.sendHeadMotor(2,2047,100)
                    back_control()
                    send.sendContinuousValue(sp.back_speed,0,0,sp.theta+sp.back_theta,0)
                    sp.print_speed = sp.back_speed
                    sp.print_Direction = 'go back' 
                sp.print_data()               
            elif send.is_start == False:
                if sp.walk_status != 'First':
                    send.sendBodyAuto(0,0,0,0,1,0)
                    sp.initial()          
                    time.sleep(0.1)
                    send.sendBodySector(999)
                    time.sleep(0.1)
                redball()
                blueball()
                total()      
                sp.print_data()
                time.sleep(0.1)
    except rospy.ROSInterruptException:
            pass