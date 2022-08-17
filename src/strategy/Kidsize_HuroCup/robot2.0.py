#!/usr/bin/env python
#coding=utf-8

import rospy
import numpy as np
from Python_API import Sendmessage
import time

send = Sendmessage()

"常用參數表"
strategy = False    #第一次指撥flag
forward = 0         #前進flag
mode = 1            #目標模式
head = 2047         #頭部馬達初始角度
speed = 8300        #前進初速度
bspeed1 = -3000     #後退初速度
max_speed = 8300    #前進最快速度
min_speed = 3000    #減速最慢速度  
max_bspeed = -7000  #後退最快速度
speed_add = 200     #前進增加量
speed_sub = 300     #前進減速量
bspeed_add = 100    #後退增加量
theta = 0           #副函式進退YAW值調整
thetafix=-2        #前進YAw值補償
thetafixb= -1       #後退YAw值補償
target = 4500       #目標面積

def yaw_forward(y): #前進YAW值調整
    global yaw_hold
    global theta
    if(y)>yaw_hold+3:
      print("turn right")
      print(y)
      theta=-2 
    elif(y)<yaw_hold-3:
      print("turn left")
      print(y)
      theta=2
    else:
      theta=-1
    return theta
    
def yaw_backward(by): #後退YAW值調整
    global yaw_hold
    global theta
    if(by)>yaw_hold+3:
      print("turn left")
      print(by)
      theta=-2
    elif(by)<yaw_hold-3:
      print("turn right")
      print(by)
      theta=1
    else:
      theta=-1
    return theta 


def colored():   #紅色球面積
  best=[]
  global objxmin
  global objxmax
  global objymin
  global objymax
  global objsize
  send.drawImageFunction(1,0,160,160,0,240,0,0,0)
  send.drawImageFunction(2,0,0,320,120,120,0,0,0)
  send.drawImageFunction(3,1,40,280,40,200,0,0,0)
  if mode==0:
    redmin=0.85
    redmax=1.15
  else:
    redmin=1.5
    redmax=2.0
  for j in range (send.color_mask_subject_cnts[0]):
    if redmin<(send.color_mask_subject_YMax[0][j]-send.color_mask_subject_YMin[0][j])/(send.color_mask_subject_XMax[0][j]-send.color_mask_subject_XMin[0][j])<redmax:
      objxmin=send.color_mask_subject_XMin[0][j]
      objxmax=send.color_mask_subject_XMax[0][j]
      objymin=send.color_mask_subject_YMin[0][j]
      objymax=send.color_mask_subject_YMax[0][j]
      objsize=send.color_mask_subject_size[0][j]
      red_ball=objsize  
      send.drawImageFunction(4,1,objxmin,objxmax,objymin,objymax,50,205,50)  
      if red_ball>100:
        best.append(red_ball)
        best.sort(reverse = True)
        return best[0]
      else:
        return objsize

def colorblue():   #藍色球面積
  best1=[]
  global objxminblue
  global objxmaxblue
  global objyminblue
  global objymaxblue
  global objsizeblue
  for j in range (send.color_mask_subject_cnts[2]):
    if 1.7<(send.color_mask_subject_YMax[2][j]-send.color_mask_subject_YMin[2][j])/(send.color_mask_subject_XMax[2][j]-send.color_mask_subject_XMin[2][j])<2.5:
      objxminblue=send.color_mask_subject_XMin[2][j]
      objxmaxblue=send.color_mask_subject_XMax[2][j]
      objyminblue=send.color_mask_subject_YMin[2][j]
      objymaxblue=send.color_mask_subject_YMax[2][j]
      objsizeblue=send.color_mask_subject_size[2][j]  
      blue_ball=objsizeblue
      send.drawImageFunction(5,1,objxminblue,objxmaxblue,objyminblue,objymaxblue,80,50,205)
      if blue_ball>100:
        best1.append(blue_ball)
        best1.sort(reverse = True)
        return best1[0]
      else:
        return objsizeblue

def total(zx,zy):    #紅籃球總面積
  global objxminblue
  global objxmaxblue
  global objxmin
  global objxmax  
  global objymin
  global objymax
  global ballsize
  global get_target   
  if objxmaxblue>objxmax and objxminblue>objxmin and 0.8<(objymax-objymin)/(objxmaxblue-objxmin)<1.2: 
      get_target=True
      ballsize=zx+zy
      print("sofjadopgjasopgsag")
      return ballsize
  else:
      get_target=False
      return 0
  
def fspeed():      #前進速度調整
  global max_speed
  global min_speed
  global color1
  global speed
  if color1<4000:
    speed+=speed_add 
    time.sleep(0.1)
    speed=min(max_speed,speed)
  else:#slow speed
    speed-=speed_sub
    time.sleep(0.06)
    speed=max(min_speed,speed)
  return speed
  
def backspeed():    #後退速度加速
  global bspeed
  bspeed-=bspeed_add
  time.sleep(0.05)
  bspeed=max(max_bspeed,bspeed)
  return bspeed

def movehead():    #頭部馬達調整
  global head
  global objymin
  global objymax
  vertical = (objymax+objymin)/2-100
  temp=40.0*vertical/240
  time.sleep(0.03)
  if abs(temp)>3:
    head-=11.378*temp*0.4
    time.sleep(0.01)
    if head>2210:
      head=2210
    elif head<1100:
      head=1100
  send.sendHeadMotor(2,round(head),50)
  time.sleep(0.01)  
  return head      
        
def initial():    #初始化
  global yaw_start,objxmax,objymax,objymin,objxmin,objsize,head,color1,speed1,bspeed1,headangle,forward,yaw_hold,theta,thetachange,thetachange2,speed,max_speed,min_speed,bspeed, max_bspeed,objxmaxblue,objyminblue,objxminblue,objymaxblue,objsizeblue,blue_size,red_size,ballsize,thetafix,thetafixb,target,get_target
  objxmax=0
  objymax=0
  objymin=0
  objxmin=0
  objsize=0
  objxmaxblue=0
  objymaxblue=0
  objxminblue=0
  objyminblue=0
  objsizeblue=0
  head=2047
  yaw_start=0
  color1=100
  speed=5000
  min_speed=2000
  speed1=0
  bspeed=0
  bspeed1=-2000
  headangle=0
  forward=0
  yaw_hold=0
  theta=0
  thetachange=0
  thetachange2=0
  red_size=0
  blue_size=0
  ballsize=0

strategy = False
if __name__ == '__main__':
  try:
    send = Sendmessage()
    while not rospy.is_shutdown():
      if send.is_start == True:
        if strategy == False:
          initial()
          send.sendBodyAuto(0,0,0,0,1,0)
          yaw_hold=send.imu_value_Yaw
          time.sleep(0.1)
          mode=1 #choice mode 0 one color 1 two color
          strategy=True
        else:
            
            yaw_start=send.imu_value_Yaw
            if mode==0:
              color1=colored()
              if color1==None:
                color1=1000
                print("no catch red ball no catch red ball ")   
            else:    
              red_size=colored()
              blue_size=colorblue()
              if red_size==None and blue_size!=None:
                red_size=blue_size
              elif blue_size==None and red_size!=None:
                blue_size=red_size
              elif red_size==None and blue_size==None:
                red_size=500
                blue_size=500   
                print("both nothing nothing nothing nothing ") 
              color1=total(red_size,blue_size)
      
            if forward==0:
                if get_target==True:
                  headangle=movehead()
                print("head theta:",headangle)
                thetachange=yaw_forward(yaw_start)+thetafix
                speed1=fspeed()
                send.sendContinuousValue(speed1,0,0,thetachange,0)
                print("theta= ",thetachange)
                print("ball ball ball",color1)
                print('target = ',target)
                print("move on move on move on")
                
            if color1>=target or forward==1:
              send.sendHeadMotor(2,2047,100)
              thetachange2=yaw_backward(yaw_start)+thetafixb
              bspeed1=backspeed()
              send.sendContinuousValue(bspeed1,0,0,thetachange2,0)
              print('thetachange2 = ',thetachange2)
              print("go back go back go back")
              forward=1

      if send.is_start == False:
          if strategy == True:
              send.sendBodyAuto(0,0,0,0,1,0)
              initial()
              send.sendHeadMotor(2,2047,100)
              strategy=False
      
  except rospy.ROSInterruptException:
        pass
