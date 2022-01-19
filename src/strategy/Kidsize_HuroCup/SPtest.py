#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from hello1 import Sendmessage
import time

send = Sendmessage()
get_target = False
first_in = True                     #第一次撥指撥flag
Forward = True                      #前進flag
###############################################
forward_initial_x = 2000                      #前進初始速度
forward_x_add = 1000                          #前進加速量
forward_x_max = 5000                          #前進最高速度
slowdown_x_basic = 1000                       #前進減速最低速度
slowdown_300 = 500                            #前進減速量
slowdown_size = 5000                           #前進減速size
forward_initial_theta = 0                     #前進基礎角度
head_classify_size = 1000                     #頭部馬達判斷結束點
back_total_size = 9000                        #進後退判斷的目標物大小
###############################################
backward_initial_x = -1000                    #前進初始速度
backward_x_add = -300                         #後退減速量
backward_x_max = -2000                        #後退最大速度
backward_initial_theta = 0                    #後退基礎角度
###############################################
send_x = forward_initial_x
Yaw = 0
total_size = 0
tmp_y_size = 0
obj_xmax = 0
obj_xmin = 0
obj_ymax = 0
obj_ymin = 0
obj_size = 0
head_motor_y = 2047                 #頭部馬達初始
head_motor_x = 2047                 #頭部馬達初始
center_y = 0 
center_x = 0

def IMU_Yaw():                      #收IMU的YAW值
    global  Yaw
    Yaw = send.imu_value_Yaw
    return  Yaw

def IMU_Yaw_Setting():              #透過IMU的YAW值設定直走區間
    Yaw = IMU_Yaw()
    Yaw_start = Yaw
    Yaw_right = Yaw_start-3
    Yaw_left = Yaw_start+3
    return Yaw_start,Yaw_right,Yaw_left

def do_value(initial_theta,theta_change,x_change):       #根據輸入值決定前進與後退的角度與速度        
    global send_x,send_theta
    send_theta = initial_theta + theta_change                      #initial_theta初始角度,theta_change角度更改量,initial_x初始速度,x_change速度更改量
    send_x += x_change
    return send_theta,send_x

def do_forward(total_size,Yaw_right,Yaw_left):
    Yaw = IMU_Yaw()
    global send_x
    print(Yaw_right,Yaw_left,Yaw)
    if total_size >= slowdown_size:
        if Yaw <= Yaw_right:
            send_theta,send_x = do_value(forward_initial_theta,2,slowdown_300)
            send_x = max(slowdown_x_basic,send_x - slowdown_300)
        elif Yaw >= Yaw_left:
            send_theta,send_x = do_value(forward_initial_theta,-2,slowdown_300)
            send_x = max(slowdown_x_basic,send_x - slowdown_300)
        else:
            send_theta,send_x = do_value(forward_initial_theta,0,slowdown_300)
            send_x = max(slowdown_x_basic,send_x - slowdown_300)
    else:
        if Yaw <= Yaw_right:
            send_theta,send_x = do_value(forward_initial_theta,4,forward_x_add)
            send_x = min(forward_x_max,send_x + forward_x_add)
        elif Yaw >= Yaw_left:
            send_theta,send_x = do_value(forward_initial_theta,-4,forward_x_add)
            send_x = min(forward_x_max,send_x + forward_x_add)
        else:
            send_theta,send_x = do_value(forward_initial_theta,0,forward_x_add)
            send_x = min(forward_x_max,send_x + forward_x_add)
    return send_theta,send_x

def do_backward(total_size,Yaw_right,Yaw_left):
    Yaw = IMU_Yaw()
    global send_x,send_theta,head_motor_x
    print(Yaw_right,Yaw_left,Yaw)
    if total_size >= head_classify_size:
        if head_motor_x == 1844 or head_motor_x == 2250:
            if head_motor_x ==1844:
                send_theta,send_x = do_value(backward_initial_theta,2,backward_x_add)
                send_x = max(backward_x_max,send_x + backward_x_add)
                print("18444444444444444444444444444444444444444444444")
            else:
                send_theta,send_x = do_value(backward_initial_theta,-2,backward_x_add)
                send_x = max(backward_x_max,send_x + backward_x_add)
                print("22500000000000000000000000000000000000000000000")
        else:
            if Yaw <= Yaw_right:
                send_theta,send_x = do_value(backward_initial_theta,2,backward_x_add)
                send_x = max(backward_x_max,send_x + backward_x_add)
            elif Yaw >= Yaw_left:
                send_theta,send_x = do_value(backward_initial_theta,-2,backward_x_add)
                send_x = max(backward_x_max,send_x + backward_x_add)
            else:
                send_theta,send_x = do_value(backward_initial_theta,0,backward_x_add)
                send_x = max(backward_x_max,send_x + backward_x_add)
    else:
        if Yaw <= Yaw_right:
            send_theta,send_x = do_value(backward_initial_theta,2,backward_x_add)
            send_x = max(backward_x_max,send_x + backward_x_add)
        elif Yaw >= Yaw_left:
            send_theta,send_x = do_value(backward_initial_theta,-2,backward_x_add)
            send_x = max(backward_x_max,send_x + backward_x_add)
        else:
            send_theta,send_x = do_value(backward_initial_theta,0,backward_x_add)
            send_x = max(backward_x_max,send_x + backward_x_add)
    return send_theta,send_x

def determine_object():
    global tmp_y_size,get_target,obj_xmax,obj_xmin,obj_ymax,obj_ymin,obj_size
    for yellow_cnt in range(send.color_mask_subject_cnts[1]):
        if ((send.color_mask_subject_YMax[1][yellow_cnt] - send.color_mask_subject_YMin[1][yellow_cnt])/(send.color_mask_subject_XMax[1][yellow_cnt] - send.color_mask_subject_XMin[1][yellow_cnt]))==1.0 :
            if send.color_mask_subject_size[1][yellow_cnt] < 15000:
                if send.color_mask_subject_size[1][yellow_cnt] > 1000:
                    if send.color_mask_subject_size[1][yellow_cnt] > tmp_y_size:
                        obj_xmax = send.color_mask_subject_XMax[1][yellow_cnt]
                        obj_xmin = send.color_mask_subject_XMin[1][yellow_cnt]
                        obj_ymax = send.color_mask_subject_YMax[1][yellow_cnt]
                        obj_ymin = send.color_mask_subject_YMin[1][yellow_cnt]
                        obj_size = send.color_mask_subject_size[1][yellow_cnt]
                        tmp_y_size = send.color_mask_subject_size[1][yellow_cnt]
                        send.drawImageFunction(4,1, obj_xmin, obj_xmax, obj_ymin, obj_ymax, 255, 255, 0)
                        print(obj_size)
                        print("get_target!!!!!!!!!!!!!!!!!!!!!!!!!")
                        get_target = True
    tmp_y_size = 0
    return obj_xmax,obj_xmin,obj_ymax,obj_ymin,obj_size

def classify_strategy():
    global get_target,center_y,center_x,total_size
    obj_xmax,obj_xmin,obj_ymax,obj_ymin,obj_size = determine_object()
    if get_target == True:
        center_y = (obj_ymax + obj_ymin)/2
        center_x = (obj_xmax + obj_xmin)/2
        total_size = obj_size
    
def head_move():
    global get_target,Forward,center_y,center_x,head_motor_y,head_motor_x
    if get_target == True:
        if Forward == True:
            move_vertical = center_y-100
            tmpangle_y = 40.0 * move_vertical /240
            time.sleep(0.04)
            if abs(tmpangle_y) > 3:
                head_motor_y -= 11.378 * tmpangle_y *0.4
                if head_motor_y > 2210:
                    head_motor_y = 2210
                elif head_motor_y < 1100:
                    head_motor_y = 1100
            send.sendHeadMotor(1,2047,50)
            time.sleep(0.01)
            send.sendHeadMotor(2,round(head_motor_y),200)
            time.sleep(0.01)
        else:
            move_vertical = center_y-120
            move_horizontal = center_x-160
            tmpangle_y = 35.0 * move_vertical/240
            tmpangle_x = 50.0 * move_horizontal/320
            if abs(tmpangle_y) > 2 or abs(tmpangle_x) > 6:
                head_motor_y -= 11.378 * tmpangle_y *0.4
                if head_motor_y > 2210:
                    head_motor_y = 2210
                elif head_motor_y < 1100:
                    head_motor_y = 1100
                head_motor_x -= 11.378 * tmpangle_x *0.4
                time.sleep(0.04)
                if head_motor_x > 2250:
                    head_motor_x = 2250
                elif head_motor_x < 1844:
                    head_motor_x = 1844
            send.sendHeadMotor(2,round(head_motor_y),200)
            time.sleep(0.01)
            send.sendHeadMotor(1,round(head_motor_x),200)
            time.sleep(0.01)
    else:
        send.sendHeadMotor(2,round(head_motor_y),200)
        time.sleep(0.01)
        send.sendHeadMotor(1,round(head_motor_x),200)
        time.sleep(0.01)


def initial_strategy():
    global Forward,send_x,send_theta,forward_initial_x,total_size,Yaw,obj_xmax,obj_xmin,obj_ymax,obj_ymin,obj_size,first_in,get_target,head_motor_y,head_motor_x
    head_motor_y = 2047
    head_motor_x = 2047
    send.sendHeadMotor(1,2047,100)
    time.sleep(0.01)
    send.sendHeadMotor(2,2047,100)
    time.sleep(0.01)
    Forward = True
    send_x = forward_initial_x
    send.drawImageFunction(4,1,0,0,0,0,255,255,0)
    send.drawImageFunction(1,1,0,0,0,0,255,255,0)
    send.drawImageFunction(2,1,0,0,0,0,255,255,0)
    total_size = 0
    get_target = False
    Yaw = 0
    total_size = 0
    obj_xmax = 0
    obj_xmin = 0
    obj_ymax = 0
    obj_ymin = 0
    obj_size = 0

if __name__ == '__main__':
    try:
        send = Sendmessage()
        while not rospy.is_shutdown():
            if send.is_start == True:
                send.drawImageFunction(1,0,0,320,120,120,152,245,255)
                send.drawImageFunction(2,0,160,160,0,240,152,245,255)
                send.drawImageFunction(3,1,80,240,60,180,125,38,205)
                classify_strategy()
                if get_target == True:
                    head_move()
                    get_target = False
                    print(total_size)
                if first_in == True:
                    imu_start,imu_right,imu_left = IMU_Yaw_Setting()
                    send.sendBodyAuto(0,0,0,0,1,0)
                    time.sleep(0.01)
                    first_in = False
                    print("first in first in ")
                else:
                    if Forward == True:
                        print("Forward Forward Forward")
                        if total_size >= back_total_size:
                            Forward = False
                            send_x = backward_initial_x
                        else:
                            send_theta,send_x = do_forward(total_size,imu_right,imu_left)
                            print("do forward do forward")
                    else:
                        send_theta,send_x = do_backward(total_size,imu_right,imu_left)
                        print("do backward do backward")
                    send.sendContinuousValue(send_x,0,0,send_theta,0)
                    print("!!!!!!!!!!!!!!!!!!!!",send_theta,"!!!!!!!!!!!!!!!!!!!")
                    print("####################",send_x,"#######################")
                    time.sleep(0.1)
            if send.is_start == False:
                #pass
                initial_strategy()
                if first_in == False:
                    send.sendBodyAuto(0,0,0,0,1,0)
                    time.sleep(0.01)
                    first_in = True
    except rospy.ROSInterruptException:
        pass



