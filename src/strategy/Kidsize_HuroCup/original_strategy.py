#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from Python_API import Sendmessage
import time

FORWARD_START_SPEED = 10000
BACK_START_SPEED = -4000
FORWARD_MIN_SPEED = 6000
BACK_MAX_SPEED = -10000
FORWARD_ORIGIN_THETA = 0
BACK_ORIGIN_THETA = 0
TARGET_Y = 130
OBJECT_MIN_SIZE = 1000
DELAY_TIME = 0.5
COLOR_DICT = {  'Orange': 0,
                'Yellow': 1,
                'Blue':   2,
                'Green':  3,
                'Black':  4,
                'Red':    5,
                'White':  6 }

send = Sendmessage()
aaaa = rospy.init_node('talker', anonymous=True, log_level=rospy.INFO)

class Sprint:
    def __init__(self):
        self.left_object_color = COLOR_DICT['Blue']
        self.right_object_color = COLOR_DICT['Orange']
        self.init()

    def init(self):
        self.walk_status = 'First'                      #走路狀態 First:第一次進策略; Forward:前進; Back:後退
        self.speed = FORWARD_START_SPEED                #速度
        self.speed_add = 0                              #速度增加量
        self.theta = 0                                  #旋轉量
        self.speed_add_delay = 0
        self.yaw = 0
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
            time.sleep(0.05)
            send.sendContinuousValue(self.speed, 0, 0, self.theta, 0)
            rospy.loginfo(f'Walk Status : {self.walk_status}')
            rospy.logdebug(f'Speed : {self.speed}, Theta : {self.theta}')
            rospy.loginfo(f'Yaw : {self.yaw}, Pitch : {send.imu_value_Pitch}')
        if not send.is_start:
            if self.walk_status != 'First':
                send.drawImageFunction(1, 1, 0, 0, 0, 0, 0, 0, 0)
                send.drawImageFunction(2, 1, 0, 0, 0, 0, 0, 0, 0)
                send.sendBodyAuto(0 ,0 ,0 ,0 ,1 ,0)
                time.sleep(0.1)
                self.init()
        
    def forward_control(self): #前進控制
        if -8 <= self.yaw <= 8:
            self.theta = FORWARD_ORIGIN_THETA
        else:
            self.theta = -2 + FORWARD_ORIGIN_THETA if self.yaw > 0 else 2 + FORWARD_ORIGIN_THETA
        self.speed = max(FORWARD_MIN_SPEED, self.speed + self.speed_add)
        
    def back_control(self): #後退控制
        if -8 <= self.yaw <= 8:
            self.theta = FORWARD_ORIGIN_THETA
        else:
            self.theta = -2 + FORWARD_ORIGIN_THETA if self.yaw > 0 else 2 + FORWARD_ORIGIN_THETA
        if time.time() - self.speed_add_delay >= 0.1 : 
            self.speed = max(BACK_MAX_SPEED, self.speed + self.speed_add)
            self.speed_add_delay = time.time()        
        
    def find_target(self):  #找目標物     
        target_y = 0 
        left_object_index = self.get_object(self.left_object_color, 1, 255, 0, 0)
        right_object_index = self.get_object(self.right_object_color, 2, 0, 0, 255)
        rospy.logdebug(f'Left : {left_object_index}, Right : {right_object_index}')
        if left_object_index != None and right_object_index != None:
            if send.color_mask_subject_XMin[self.right_object_color][right_object_index] > send.color_mask_subject_XMin[self.left_object_color][left_object_index] and send.color_mask_subject_XMax[self.right_object_color][right_object_index] > send.color_mask_subject_XMax[self.left_object_color][left_object_index]:
                target_y =  (int(send.color_mask_subject_Y[self.left_object_color][left_object_index] + send.color_mask_subject_Y[self.right_object_color][right_object_index]) / 2)
        rospy.logdebug(f"Target Y : {target_y}")
        if target_y >= 40:
            self.speed_add = -100
            if target_y >= TARGET_Y:
                self.speed = BACK_START_SPEED
                self.walk_status = 'Back'
                self.speed_add_delay = time.time() - DELAY_TIME

    def get_object(self, color, cnt, r, g, b):
        max_object = max(send.color_mask_subject_size[color])
        if max_object >= OBJECT_MIN_SIZE:
            max_index = send.color_mask_subject_size[color].index(max_object)
            send.drawImageFunction(cnt, 1, send.color_mask_subject_XMin[color][max_index], send.color_mask_subject_XMax[color][max_index], send.color_mask_subject_YMin[color][max_index], send.color_mask_subject_YMax[color][max_index], r, g, b)
        else:
            send.drawImageFunction(cnt, 1, 0, 0, 0, 0, r, g, b)
        return max_index if max_object >= OBJECT_MIN_SIZE else None

if __name__ == '__main__':
    try:
        r = rospy.Rate(30)
        strategy = Sprint()
        while not rospy.is_shutdown():                                  
            strategy.main()
            r.sleep()
    except rospy.ROSInterruptException:
        pass