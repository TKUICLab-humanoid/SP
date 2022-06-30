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
      theta=-1
    elif(x)<yaw_hold-3:
      print("turn left")
      print(x)
      theta=2
    else:
      theta=2
    return theta
    
def yaw_backward(bx):
    global yaw_hold
    global theta
    if(bx)>yaw_hold+3:
      print("turn left")
      print(bx)
      theta=-1
    elif(bx)<yaw_hold-3:
      print("turn right")
      print(bx)
      theta=1
    else:
      theta=1
    return theta 


def colored():
  best=[]
  global objxmin
  global objxmax
  global objymin
  global objymax
  global objsize
  send.drawImageFunction(1,0,160,160,0,240,0,0,0)
  send.drawImageFunction(2,0,0,320,120,120,0,0,0)
  send.drawImageFunction(3,1,40,280,40,200,0,0,0)
  for j in range (send.color_mask_subject_cnts[0]):
    if mode==0:
      redmin=0.85
      redmax=1.15
    else:
      redmin=1.5
      redmax=2.0
    if redmin<(send.color_mask_subject_YMax[0][j]-send.color_mask_subject_YMin[0][j])/(send.color_mask_subject_XMax[0][j]-send.color_mask_subject_XMin[0][j])<redmax:
      objxmin=send.color_mask_subject_XMin[0][j]
      objxmax=send.color_mask_subject_XMax[0][j]
      objymin=send.color_mask_subject_YMin[0][j]
      objymax=send.color_mask_subject_YMax[0][j]
      objsize=send.color_mask_subject_size[0][j]
      red_ball=objsize  
      send.drawImageFunction(4,1,objxmin,objxmax,objymin,objymax,50,205,50)  
      if red_ball>500:
        best.append(red_ball)
        best.sort(reverse = True)
        return best[0]
      else:
        return objsize

def colorblue():
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
      if blue_ball>500:
        best1.append(blue_ball)
        best1.sort(reverse = True)
        return best1[0]
      else:
        return objsizeblue

def total(zx,zy):
  global objxminblue
  global objxmaxblue
  global objxmin
  global objxmax  
  global ballsize   
  if objxmaxblue>objxmax and objxminblue>objxmin: 
      ballsize=zx+zy
      print("sofjadopgjasopgsag")
      return ballsize
  else:
      return 0
  
def fspeed():
  global firstspd
  global color1
  global slowspd
  global speed
  if color1<4000:
    firstspd+=200
    time.sleep(0.05)
    speed=min(7500,firstspd)
  else:#slow speed
    speed-=300
    time.sleep(0.06)
    speed=max(3000,speed)
  print("speed:",speed)
  return speed
  
def backspeed():
  global bspeed
  bspeed-=100
  time.sleep(0.05)
  bspeed=max(-6500,bspeed)
  print("backspeeed:",bspeed)
  return bspeed

def movehead():
  global head
  global  color1
  global objymin
  global objymax
  vertical = (objymax+objymin)/2-100
  temp=40.0*vertical/240
  time.sleep(0.04)
  if abs(temp)>3:
    head-=11.378*temp*0.4
    time.sleep(0.04)
    if head>2210:
      head=2210
    elif head<1100:
      head=1100
  send.sendHeadMotor(2,round(head),50)
  time.sleep(0.01) 
  return head      
        
def initial():
  global yaw_start,objxmax,objymax,objymin,objxmin,objsize,head,firstspd,color1,speed1,bspeed1,headangle,forward,yaw_hold,theta,theta2,theta3,thetachange,thetachange2,straight,slowspd,speed,bspeed,objxmaxblue,objyminblue,objxminblue,objymaxblue,objsizeblue,ball_total,blue_size,red_size,ballsize,mode
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
  firstspd=5000
  yaw_start=0
  color1=100
  speed=0
  speed1=0
  bspeed=0
  bspeed1=-3000
  headangle=0
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
                print("no catch red ball no catch red ball no catch red ball")   
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
                print("both nothing nothing nothing nothing nothing") 
              color1=total(red_size,blue_size)
      
            if forward==0:
                headangle=movehead()
                print("head theta:",headangle)
                thetachange=yaw_forward(yaw_start)
                speed1=fspeed()
                send.sendContinuousValue(speed1,0,0,thetachange,0)
                print("ball ball ball",color1)
                print("move on move on move on")
                
            if color1>6000 or forward==1:
              send.sendHeadMotor(2,2047,100)
              thetachange2=yaw_backward(yaw_start)
              bspeed1=backspeed()
              send.sendContinuousValue(bspeed1,0,0,thetachange2,0)
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

