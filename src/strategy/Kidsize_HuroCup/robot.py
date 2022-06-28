#!/usr/bin/env python
#coding=utf-8

import rospy
import numpy as np
from Python_API import Sendmessage
import time

send = Sendmessage()

def yaw_forward(x):
    global yaw_hold
    global theta
    if(x)>yaw_hold+3:
      print("turn right")
      print(x)
      theta=-2
    elif(x)<yaw_hold-3:
      print("turn left")
      print(x)
      theta=3
    else:
      theta=0
    return theta
    
def yaw_backward(bx):
    global yaw_hold
    global theta
    global theta3
    global tt
    global objxmax
    global objxmin
    if(bx)>yaw_hold+3:
      print("turn left")
      print(bx)
      theta=-2
    elif(bx)<yaw_hold-3:
      print("turn right")
      print(bx)
      theta=1
    else:
      theta=0 
    print("total change:", theta) 


def colored():
  best=[]
  global objxmin
  global objxmax
  global objymin
  global objymax
  global objsize
  global target_blue
  global target_red
  send.drawImageFunction(1,0,160,160,0,240,0,0,0)
  send.drawImageFunction(2,0,0,320,120,120,0,0,0)
  send.drawImageFunction(3,1,40,280,40,200,0,0,0)
  for j in range (send.color_mask_subject_cnts[0]):
    if mode==0:
      redmin=0.85
      redmax=1.15
    else:
      redmin=1.2
      redmax=2.2
    if redmin<(send.color_mask_subject_YMax[0][j]-send.color_mask_subject_YMin[0][j])/(send.color_mask_subject_XMax[0][j]-send.color_mask_subject_XMin[0][j])<redmax:
      objxmin=send.color_mask_subject_XMin[0][j]
      objxmax=send.color_mask_subject_XMax[0][j]
      objymin=send.color_mask_subject_YMin[0][j]
      objymax=send.color_mask_subject_YMax[0][j]
      objsize=send.color_mask_subject_size[0][j]
      red_ball=objsize  
      send.drawImageFunction(4,1,objxmin,objxmax,objymin,objymax,50,205,50)  
      best.append(red_ball)
      best.sort(reverse = True)
      target_red=True
      return best[0]
  

def colorblue():
  best1=[]
  global objxminblue
  global objxmaxblue
  global objyminblue
  global objymaxblue
  global objsizeblue
  global target_blue
  global target_red
  for j in range (send.color_mask_subject_cnts[2]):
    if 1.2<(send.color_mask_subject_YMax[2][j]-send.color_mask_subject_YMin[2][j])/(send.color_mask_subject_XMax[2][j]-send.color_mask_subject_XMin[2][j])<2.5:
      objxminblue=send.color_mask_subject_XMin[2][j]
      objxmaxblue=send.color_mask_subject_XMax[2][j]
      objyminblue=send.color_mask_subject_YMin[2][j]
      objymaxblue=send.color_mask_subject_YMax[2][j]
      objsizeblue=send.color_mask_subject_size[2][j]  
      blue_ball=objsizeblue
      send.drawImageFunction(5,1,objxminblue,objxmaxblue,objyminblue,objymaxblue,80,50,205)
      best1.append(blue_ball)
      best1.sort(reverse = True)
      target_blue=True
      target_blue=True
      return best1[0]

def total(zx,zy):
  global objyminblue
  global objymaxblue
  global objymin
  global objymax
  global objxminblue
  global objxmaxblue
  global objxmin
  global objxmax  
  global ballsize 
  if target_blue==True and target_red==True:  
    if objxmaxblue>objxmax and objxminblue>objxmin: 
      ballsize=zx+zy
      print("sofjadopgjasopgsag")
      return ballsize
      target_blue=False 
      target_red=False
  
def fspeed():
  global firstspd
  global color1
  global slowspd
  global speed
  if color1==None:
      color1=1000
  if color1<4000:
      firstspd+=200
      time.sleep(0.1)
      speed=min(8000,firstspd)
  else:#slow speed
      speed-=300
      time.sleep(0.06)
      speed=max(2000,speed)
  print("speed:",speed)
  return speed
  
def backspeed():
  global bspeed
  bspeed-=100
  time.sleep(0.05)
  bspeed=max(-7000,bspeed)
  print("backspeeed:",bspeed)
  return bspeed


def movehead():
  global head
  global  color1
  global objymin
  global objymax
  global target_blue
  global target_red
  headchange=0
  center=(objymax+objymin)/2
  print("headmotor",head)
  print("max min :",objymax,objymin)
  if target_blue==True and target_red==True:
    move_vertical=center-100
    theta2=40.0* move_vertical/240
    time.sleep(0.04)
    print("center",center)
    if abs(theta2) > 3:
        head -= 11.378 * theta2 *0.4
        if head > 2210:
          head = 2210
        elif head < 1100:
          head = 1100
    send.sendHeadMotor(2,round(head),50)
    time.sleep(0.01)  
  else:
      send.sendHeadMotor(2,round(head),50)
      time.sleep(0.01)        
def initial():
  global yaw_start,objxmax,objymax,objymin,objxmin,objsize,head,firstspd,color1,speed1,bspeed1,forward,yaw_hold,theta,theta2,theta3,thetachange,thetachange2,straight,slowspd,speed,bspeed,objxmaxblue,objyminblue,objxminblue,objymaxblue,objsizeblue,ball_total,blue_size,red_size,ballsize,mode,target_red,target_blue
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
  ball_total=0
  head=2047
  firstspd=4000
  yaw_start=0
  color1=100
  speed=0
  speed1=0
  bspeed=0
  bspeed1=-2000
  forward=0
  yaw_hold=0
  theta=0
  theta2=0
  theta3=0
  thetachange=0
  thetachange2=0
  straight=0
  slowspd=0
  red_size=0
  blue_size=0
  ballsize=0
  mode=1
  target_red=False  
  target_blue=False
target_red=False  
target_blue=False
strategy = False
imgdata=[[ None for i in range(240)]for j in range(320)]
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
                print("gkwrtgreosrtretreterwtretretr")   
            else:    
              red_size=colored()
              blue_size=colorblue() 
              color1=total(red_size,blue_size)
              movehead()
            
            if forward==0:
                thetachange=yaw_forward(yaw_start)
                speed1=fspeed()
                send.sendContinuousValue(speed1,0,0,thetachange,0)
                print("aaaaaaaaaaaaaaaaaaaaaaa:",red_size)
                print("bbbbbbbbbbbbbbbbbbbbbbb:",blue_size)
                print("ball ball ball",color1)
                print("move on move on move on")
                
            if color1>7800 or forward==1:
              send.sendHeadMotor(2,2047,100)
              thetachange2=yaw_backward(yaw_start)
              bspeed1=backspeed()
              send.sendContinuousValue(bspeed1,0,0,thetachange2,0)
              print("go back go back go back")
              print("kkkkkkkkkkkkkkkkkkkkkkkkkkk",thetachange2)
              forward=1

      if send.is_start == False:
          if strategy == True:
              send.sendBodyAuto(0,0,0,0,1,0)
              initial()
              send.sendHeadMotor(2,2047,100)
              strategy=False
          """else:  
            #time.sleep(1)
            initial()
            send.sendHeadMotor(2,2047,100)
            print("ready??????????????????????????????????????????????????????????????")"""
      
  except rospy.ROSInterruptException:
        pass

