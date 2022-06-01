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
      theta=-4
    elif(x)<yaw_hold-3:
      print("turn left")
      print(x)
      theta=4
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
    for j in range (send.color_mask_subject_cnts[0]):
      objxmin=send.color_mask_subject_XMin[0][j]
      objxmax=send.color_mask_subject_XMax[0][j]
    """if objxmax==None or objxmin==None:#avoid lost ball
      objxmin=140
      objxmax=180
    if -40>((objxmax+objxmin)/2-160):
      theta3=1
      print("little right")
    if  40>((objxmax+objxmin)/2-160): 
      theta3=-1  
      print("little left")"""
    if(bx)>yaw_hold+3:
      print("turn left")
      print(bx)
      theta=-4
    elif(bx)<yaw_hold-3:
      print("turn right")
      print(bx)
      theta=4
    else:
      theta=1
    tt=theta#+theta3
    print("total change:", tt) 
    return tt 


def colored():
  best=[]
  global objxmin
  global objxmax
  global objymin
  global objymax
  global objsize
  send.drawImageFunction(1,0,160,160,0,240,0,0,0)
  send.drawImageFunction(2,0,0,320,120,120,0,0,0)
  send.drawImageFunction(3,1,80,240,70,170,0,0,0)
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
    if 1.5<(send.color_mask_subject_YMax[2][j]-send.color_mask_subject_YMin[2][j])/(send.color_mask_subject_XMax[2][j]-send.color_mask_subject_XMin[2][j])<2.0:
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
  global objyminblue
  global objymaxblue
  global objymin
  global objymax
  global objxminblue
  global objxmaxblue
  global objxmin
  global objxmax  
  global ballsize
  for j in range (send.color_mask_subject_cnts[0]):
    objymin=send.color_mask_subject_XMin[0][j]
    objymax=send.color_mask_subject_XMax[0][j]
    objxmin=send.color_mask_subject_XMin[0][j]
    objxmax=send.color_mask_subject_XMax[0][j]
  for j in range (send.color_mask_subject_cnts[2]):
    objyminblue=send.color_mask_subject_XMin[2][j]
    objymaxblue=send.color_mask_subject_XMax[2][j] 
    objxminblue=send.color_mask_subject_XMin[2][j]
    objxmaxblue=send.color_mask_subject_XMax[2][j]   
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
    time.sleep(0.08)
    speed=min(7000,firstspd)
  else:#slow speed
    speed-=300
    time.sleep(0.06)
    speed=max(1500,speed)
  print("speed:",speed)
  return speed
  
def backspeed():
  global bspeed
  bspeed-=100
  time.sleep(0.05)
  bspeed=max(-4900,bspeed)
  print("backspeeed:",bspeed)
  return bspeed


def movehead():
  global head
  global  color1
  headchange=0
  for j in range (send.color_mask_subject_cnts[0]):
    if 1.7<(send.color_mask_subject_YMax[0][j]-send.color_mask_subject_YMin[0][j])/(send.color_mask_subject_XMax[0][j]-send.color_mask_subject_XMin[0][j])<2.3:
      objymin=send.color_mask_subject_YMin[0][j]
      objymax=send.color_mask_subject_YMax[0][j]
      #objsize=send.color_mask_subject_size[0][j]
      if color1<=1000:
        headchange=1000
      else:
        headchange=color1
      if objymin<70 and objymax<170:
          print("head on  head on   head on")
          if head>=2200:
            head=2150
          else:  
            head+=(int)(headchange/160)
          #time.sleep(0.4)
          realhead=head
          send.sendHeadMotor(2,realhead,50)
          return realhead
      elif objymax>170 and objymin>70:
          print("head down   head down   head down")
          if head<=1000:
            head=1200
        #time.sleep(0.4)
          else:
            head-=(int)(headchange/160)
          realhead=head
          send.sendHeadMotor(2,realhead,100)
          return realhead
      else:
          realhead=head
          send.sendHeadMotor(2,realhead,100)
          return realhead  

"""def hor():
      if 0.8<(objymax-objymin)/(objxmax-objxmin)<1.2:
        if objxmin<80:
          send.sendContinuousValue(-500,1000,0,0,0)
          #time.sleep(0.5)
        elif objxmax>240:
          send.sendContinuousValue(0,-1000,0,0,0)
          #time.sleep(0.5)
        else:
          print("xmin,xmax",objxmin,objxmax)"""          
def initial():
  global yaw_start,objxmax,objymax,objymin,objxmin,objsize,head,firstspd,color1,speed1,bspeed1,zzz,ss,yaw_hold,theta,theta2,theta3,thetachange,thetachange2,straight,tt,slowspd,speed,bspeed,objxmaxblue,objyminblue,objxminblue,objymaxblue,objsizeblue,ball_total,y,x,ballsize
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
  firstspd=2000
  yaw_start=0
  color1=100
  speed=0
  speed1=0
  bspeed=0
  bspeed1=-1500
  zzz=0
  ss=0
  yaw_hold=0
  theta=0
  theta2=0
  theta3=0
  thetachange=0
  thetachange2=0
  straight=0
  tt=0
  slowspd=0
  x=0
  y=0
  ballsize=0
aa = False
imgdata=[[ None for i in range(240)]for j in range(320)]
if __name__ == '__main__':
  try:
    send = Sendmessage()
    while not rospy.is_shutdown():
      if send.is_start == True:
        if aa == False:
          initial()
          send.sendBodyAuto(0,0,0,0,1,0)
          yaw_hold=send.imu_value_Yaw
          time.sleep(0.1)
          print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
          mode=1 #choice mode
          aa=True
        else:
            
            yaw_start=send.imu_value_Yaw
            if mode==0:
              if color1==None:
                color1=1000
              color1=colored()  
            else:    
              x=colored()
              y=colorblue()
              if x==None and y!=None:
                x=y
              elif y==None and x!=None:
                y=x
              elif x==None and y==None:
                x=500
                y=500   
                print("nothing nothing nothing nothing nothing") 
                color1=total(x,y)
            
            zzz=movehead()
            print("head theta:",zzz)
            
            if ss==0:
                thetachange=yaw_forward(yaw_start)
                speed1=fspeed()
                send.sendContinuousValue(speed1,0,0,thetachange,0)
                print("ball ball ball",color1)
                print("move on move on move on")
                
            if color1>7850 or ss==1:
              thetachange2=yaw_backward(yaw_start)
              bspeed1=backspeed()
              send.sendContinuousValue(bspeed1,0,0,thetachange2,0)
              print("go back go back go back")
              print("kkkkkkkkkkkkkkkkkkkkkkkkkkk",thetachange2)
              ss=1

      if send.is_start == False:
          if aa == True:
              send.sendBodyAuto(0,0,0,0,1,0)
              initial()
              send.sendHeadMotor(2,2047,100)
              aa=False
          """else:  
            #time.sleep(1)
            initial()
            send.sendHeadMotor(2,2047,100)
            print("ready??????????????????????????????????????????????????????????????")"""
      
  except rospy.ROSInterruptException:
        pass
