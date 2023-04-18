#!/usr/bin/env python
#coding=utf-8
from ast import walk
from mimetypes import init
import rospy
import numpy as np
from Python_API import Sendmessage
import time
import math

send = Sendmessage()

class Walk_API:
    def __init__(self):
        self.initial()

    def initial(self):
        self.Walk_Status = 'Start'
        self.Speed = 5000
        self.Theta = 0
        self.Speed_Max = 10000
        self.Speed_Min = -10000
        self.Control_Start = 0
        self.Now_Time = 0
        send.sendSensorReset()
        time.sleep(0.1)

    def forward(self):
        self.Now_Time = time.time()
        if self.Now_Time - self.Control_Start >= 0.1:
            self.Speed = min(self.Speed_Max,self.Speed+200)
            send.sendContinuousValue(self.Speed,0,0,self.Theta,0)
            self.Control_Start = self.Now_Time
    
    def back(self):
        self.Now_Time = time.time()
        if self.Now_Time - self.Control_Start >= 0.1:
            self.Speed = max(self.Speed_Min,self.Speed-200)
            send.sendContinuousValue(self.Speed,0,0,self.Theta,0)
            self.Control_Start = self.Now_Time

MAR = Walk_API()

try:    
    while not rospy.is_shutdown():        
        if send.is_start :  
            if MAR.Walk_Status == 'Start' :
                MAR.initial()
                send.sendBodyAuto(0,0,0,0,1,0)
                MAR.Control_Start = time.time()
                MAR.Walk_Status = 'Contiune'
            elif MAR.Walk_Status == 'Contiune' :
                if send.DIOValue == 27:
                    MAR.forward()
                elif send.DIOValue == 28:
                    MAR.back()
        elif not send.is_start:
            if MAR.Walk_Status != 'Start':
                MAR.initial()
                send.sendBodyAuto(0,0,0,0,1,0)
        # print(MAR.Walk_Status)
except rospy.ROSInterruptException:
    pass