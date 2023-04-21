#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from Python_API import Sendmessage
import time

FORWARD_START_SPEED = 10000
BACK_START_SPEED = -4000
FORWARD_MAX_SPEED = 10000
FORWARD_MIN_SPEED = 6000
BACK_MAX_SPEED = -10000
FORWARD_ORIGIN_THETA = 0
FORWARD_MAX_THETA = 2
FORWARD_MIN_THETA = -2
BACK_MAX_THETA = 2
BACK_MIN_THETA = -2
BACK_ORIGIN_THETA = 0
TARGET_Y = 185

send = Sendmessage()

class Object:
    def __init__(self):
        self.get_left_object = False
        self.get_right_object = False
    
    def get_object(self, left_color, right_color):
        max_left_object = max(send.color_mask_subject_size[left_color])
        if max_left_object >= 1000:
            max_left_index = send.color_mask_subject_size[left_color].index(max_left_object)
            get_left_object = True
            send.drawImageFunction(1, 1, send.color_mask_subject_XMin[left_color][max_left_index], send.color_mask_subject_XMax[left_color][max_left_index], send.color_mask_subject_YMin[left_color][max_left_index], send.color_mask_subject_YMax[left_color][max_left_index], 255, 0, 0)
        else:
            get_left_object = False
            send.drawImageFunction(1, 1, 0, 0, 0, 0, 255, 0, 0)

        max_right_object = max(send.color_mask_subject_size[right_color])
        if max_right_object >= 1000:
            max_right_index = send.color_mask_subject_size[right_color].index(max_right_object)
            get_right_object = True
            send.drawImageFunction(2, 1, send.color_mask_subject_XMin[right_color][max_right_index], send.color_mask_subject_XMax[right_color][max_right_index], send.color_mask_subject_YMin[right_color][max_right_index], send.color_mask_subject_YMax[right_color][max_right_index], 0, 0, 255)
        else:
            get_right_object = False
            send.drawImageFunction(2, 1, 0, 0, 0, 0, 0, 0, 255)

        if get_left_object and get_right_object:
            if send.color_mask_subject_XMin[right_color][max_right_index] > send.color_mask_subject_XMin[left_color][max_left_index] and send.color_mask_subject_XMax[right_color][max_right_index] > send.color_mask_subject_XMax[left_color][max_left_index]:
                return (int(send.color_mask_subject_Y[left_color][max_left_index] + send.color_mask_subject_Y[right_color][max_right_index]) / 2)
            else:
                return 0
        else:
            return 0

class Sprint:
    def __init__(self):
        self.init()

    def init(self):
        self.walk_status = 'First'                      #走路狀態 First:第一次進策略; Forward:前進; Back:後退
        self.speed = FORWARD_START_SPEED                #速度
        self.speed_add = 100                            #速度增加量
        self.theta = 0                                  #旋轉量
        self.yaw = 0                                    #Yaw值
        self.speed_add_delay = 0
        self.target = Object()
        self.target_y = 0
        send.sendHeadMotor(1, 2048, 50)
        send.sendHeadMotor(2, 2600, 50)
        send.sendSensorReset()

    def main(self):
        if send.is_start:            
            if self.walk_status == 'First':  
                self.init()
                time.sleep(0.1)
                send.sendBodyAuto(0 ,0 ,0 ,0 ,1 , 0)
                self.walk_status = 'Forward'
            elif self.walk_status == 'Forward':  
                self.forward_control()
                self.find_target()                                                      
            elif self.walk_status == 'Back':
                self.back_control()
            self.yaw = send.imu_value_Yaw
            time.sleep(0.02)
            send.sendContinuousValue(self.speed, 0, 0, self.theta, 0)
            self.print_data()
        if not send.is_start:
            if self.walk_status != 'First':
                send.sendBodyAuto(0 ,0 ,0 ,0 ,1 ,0)
                time.sleep(0.01)
                self.init()

    def print_data(self): 
        rospy.loginfo(f'Walk Status : {self.walk_status}')
        rospy.logdebug(f'Speed : {self.speed}')
        rospy.logdebug(f'Theta : {self.theta}')
        rospy.logdebug(f"Target Y : {self.target_y}")
        rospy.loginfo(f'Yaw : {self.yaw}')

    def forward_control(self): #前進控制
        if self.yaw > 8:
            self.theta = -2
            # self.theta = max(FORWARD_MIN_THETA, self.theta) 
        elif self.yaw < -8:
            self.theta = 2
            # self.theta = min(FORWARD_MAX_THETA, self.theta)
        else:
            self.theta = 0
        self.theta += FORWARD_ORIGIN_THETA

        self.speed += self.speed_add 
        self.speed = min(FORWARD_MAX_SPEED, self.speed)
        self.speed = max(FORWARD_MIN_SPEED, self.speed)
        
    def back_control(self): #後退控制
        if self.yaw > 8:
            self.theta = -2
            # self.theta = max(FORWARD_MIN_THETA, self.theta) 
        elif self.yaw < -8:
            self.theta = 2
            # self.theta = min(FORWARD_MAX_THETA, self.theta)
        else:
            self.theta = 0
        self.theta += BACK_ORIGIN_THETA

        if time.time() - self.speed_add_delay >= 0.01: 
            self.speed += self.speed_add
            self.speed = max(BACK_MAX_SPEED,self.speed)
            self.speed_add_delay = time.time()        
        
    def find_target(self):  #找目標物      
        self.target_y = self.target.get_object(0, 2)        
            
        if self.target_y >= 40:
            self.speed_add = -100
            if self.target_y >= TARGET_Y:
                self.speed = BACK_START_SPEED
                self.walk_status = 'Back'
                send.sendHeadMotor(2,2400,50)
                time.sleep(0.2)      
                self.speed_add_delay = time.time()                

if __name__ == '__main__':
    try:
        r = rospy.Rate(30)
        sp = Sprint()
        while not rospy.is_shutdown():                                  
            sp.main()
            r.sleep()
    except rospy.ROSInterruptException:
        pass