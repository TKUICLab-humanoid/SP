#!/usr/bin/env python  
#coding=utf-8 

import rospy
import numpy as np
from Python_API import Sendmessage
import time                       

DICT_COLOR = {'Orange':0, 'Yellow':1, 'Blue':2, 'Green':3,'Black':4, 'Red':5, 'White':6}  #字典存放所有顏色
LEFT_BALL_COLOR = DICT_COLOR['Orange']    #雙色模 左邊  字典[顏色名稱: 橘色(更改此顏色名稱就能改建模使用)]
RIGHT_BALL_COLOR = DICT_COLOR['Blue']     #雙色模 右邊  字典[顏色名稱: 藍色(更改此顏色名稱就能改建模使用)]

SPEED_MAX_FORWARD = 6500    #前進最快速度                                #原MAX_SPEED
SPEED_MIN_FORWARD = 4000   #前進減速速度  （在球前）                          #原MIN_SPEED
SPEED_MAX_BACK = -5500      #後退最快速度                                #原MAX_BSPEED
SPEED_SUB_FORWARD = 100     #前進減速量                                  #原SPEED_SUB
SPEED_ADD_BACK = 200        #後退增加量                                  #原BSPEED_ADD
THETA_FIX_FORWARD = 0       #前進YAw值補償                               #原THETAFIX
THETA_FIX_BACK = -3         #後退YAw值補償    #-3會偏左  -2微微偏右       #原THETAFIXB
BALL_FRONT_SIZE = 5000      #預測目標面積                                #原TARGET
BALL_SUB_SIZE = 1500
TWO_COLOR_FLAG = True       #T 雙色球  F單色球 （ 只需要改此即可使用單雙色）
HEAD_Y_MAX = 1900
HEAD_Y_MIN = 1300 
BUFFER_TIME = 2                     #緩衝時間    
HEAD_UPDATE_TIME = 0.1 #3畫面更新率 
send = Sendmessage()
api_need_delet = rospy.init_node('talker', anonymous = True, log_level = rospy.INFO)

class Sprint:
  def __init__(self): 
    self.strategy_flag = False      #第一次指撥flag                              #加上flag 作為False True 的標籤     
    self.get_target_flag = True     #判定有無獲取目標物面積 有的話是True 沒有是False                                                               
    self.initial()

  def initial(self):    #初始化                                                           
    self.forward_speed = SPEED_MAX_FORWARD         #原speed                         #前進速度                   (會拿來加上減速量所以會變動) (如果只改 MAX值可以覆蓋 此值 )             
    self.speed_back = -3000           #原bspeed                        #後退速度                   (會拿來加上增速量所以會變動)
    self.back_flag = False            #原forward                       #前進與後退的flag                  (0,1變動)
    self.head_y = HEAD_Y_MAX                                                   #頭部馬達角度               (2047,1800,1300變動)
    self.theta_initial = 0            #原theta                         #副函式進退YAW值調整        (2,1,0,-1,-2)
    self.theta_fixed_forward = 0      #原thetachange                   #加上修正值後 用來輸出前進的theta值
    self.theta_fixed_back = 0         #原thetachange2                  #加上修正值後 用來輸出後退的theta值
    self.ball_size = 100              #原color1     #偵測目標面積 初始值為零 可能會以為是沒有讀取到數值 進而預設給1000面積 並且顯示沒抓取到某個球
    self.now_yaw = 0                  #原yaw_start                     #初始Yaw值
    self.left_size = 0                                                 #紅球面積 原red_size
    self.right_size = 0                                                #藍球面積 原blue_size
    self.left_line_xmin = 0            #原objxmin
    self.left_line_xmax = 0            #原objxmax                        
    self.left_line_ymin = 0            #原objymin
    self.left_line_ymax = 0            #原objymax 
    self.right_line_xmin = 0           #原objxminblue                             
    self.right_line_xmax = 0           #原objxmaxblu
    self.right_line_ymin = 0           #原objyminblue
    self.right_line_ymax = 0           #原objymaxblue   
    self.buffer_fix_time = 0                  
    self.control_time = 0

  def forward_control(self):     #前進YAW值調整                                      
    if(self.now_yaw > 5):          #需要修改y名稱                            
      if(self.theta_initial > 0):
        self.theta_initial = 0
      else:
        rospy.logdebug(f'turn right ') 
        self.theta_initial = -1
    elif (self.now_yaw < -5):           #
      if(self.theta_initial < 0):
        self.theta_initial = 0
      else:
        rospy.logdebug(f'turn left ')
        self.theta_initial = 2
    else:
      self.theta_initial = 0

    if self.ball_size > BALL_SUB_SIZE:                 
      self.forward_speed -= SPEED_SUB_FORWARD
      self.forward_speed = max(SPEED_MIN_FORWARD, self.forward_speed) 
      
    rospy.logdebug(f'theta = {self.theta_initial}, speed = {self.forward_speed}')
      
  def back_control(self):    #後退YAW值調整                                       
    if(self.now_yaw > 1):             #需要修改self.now_yaw名稱                           
      if(self.theta_initial > 0):
        self.theta_initial = 0
      else:
        rospy.logdebug(f'<<<<<----- ')
        self.theta_initial = -2
    elif(self.now_yaw < -5):
      if(self.theta_initial < 0):
        self.theta_initial = 0
      else:
        rospy.logdebug(f'----->>>>> ')
        self.theta_initial = 3
    else:
      self.theta_initial = 0

    if(time.time() > self.buffer_fix_time):
      self.speed_back -= SPEED_ADD_BACK                                          
      self.speed_back = max(SPEED_MAX_BACK, self.speed_back)
    
    rospy.logdebug(f'theta = {self.theta_initial}, speed = {self.speed_back}')

  def get_left_size(self, left_size_min, left_size_max):   #紅色球面積                                                                     
    left_size_matrix = []    #原beat 
    # left_size_min = 1.5
    # left_size_max = 2.0
    for j in range (send.color_mask_subject_cnts[LEFT_BALL_COLOR]):
      if left_size_min < (send.color_mask_subject_YMax[LEFT_BALL_COLOR][j] - send.color_mask_subject_YMin[LEFT_BALL_COLOR][j]) / (send.color_mask_subject_XMax[LEFT_BALL_COLOR][j] - send.color_mask_subject_XMin[LEFT_BALL_COLOR][j]) < left_size_max:
        self.left_line_xmin = send.color_mask_subject_XMin[LEFT_BALL_COLOR][j]
        self.left_line_xmax = send.color_mask_subject_XMax[LEFT_BALL_COLOR][j]
        self.left_line_ymin = send.color_mask_subject_YMin[LEFT_BALL_COLOR][j]
        self.left_line_ymax = send.color_mask_subject_YMax[LEFT_BALL_COLOR][j]  
        send.drawImageFunction(4, 1, self.left_line_xmin , self.left_line_xmax , self.left_line_ymin , self.left_line_ymax, 50, 205, 50)  
        if send.color_mask_subject_size[LEFT_BALL_COLOR][j] > 500:           
          left_size_matrix.append(send.color_mask_subject_size[LEFT_BALL_COLOR][j])  
          self.left_size = max(left_size_matrix)
        else:
          self.left_size = send.color_mask_subject_size[LEFT_BALL_COLOR][j] 
 
  def get_right_size(self):   #藍色球面積
    right_size_matrix = []  #原beat1                                                              
    for j in range (send.color_mask_subject_cnts[RIGHT_BALL_COLOR]):                                               
      if 1.7 < (send.color_mask_subject_YMax[RIGHT_BALL_COLOR][j] - send.color_mask_subject_YMin[RIGHT_BALL_COLOR][j]) / (send.color_mask_subject_XMax[RIGHT_BALL_COLOR][j] - send.color_mask_subject_XMin[RIGHT_BALL_COLOR][j]) < 2.5:
        self.right_line_xmin = send.color_mask_subject_XMin[RIGHT_BALL_COLOR][j]
        self.right_line_xmax = send.color_mask_subject_XMax[RIGHT_BALL_COLOR][j]
        self.right_line_ymin = send.color_mask_subject_YMin[RIGHT_BALL_COLOR][j]
        self.right_line_ymax = send.color_mask_subject_YMax[RIGHT_BALL_COLOR][j]
        send.drawImageFunction(5, 1, self.right_line_xmin, self.right_line_xmax ,self.right_line_ymin, self.right_line_ymax, 80, 50, 205)
        if send.color_mask_subject_size[RIGHT_BALL_COLOR][j] > 100:                  
          right_size_matrix.append(send.color_mask_subject_size[RIGHT_BALL_COLOR][j])
          self.right_size = max(right_size_matrix)
        else:
          self.right_size = send.color_mask_subject_size[RIGHT_BALL_COLOR][j]  

  def ball_size_total(self):    #紅籃球總面積            
    if self.right_line_xmax > self.left_line_xmax  and self.right_line_xmin > self.left_line_xmin  and 0.8 < (self.left_line_ymax - self.left_line_ymin ) / (self.right_line_xmax - self.left_line_xmin ) < 1.2: 
        self.get_target_flag = True
        self.ball_size = self.left_size + self.right_size
    else:
        self.get_target_flag = False
   
  def move_head(self):    #頭部馬達調整                                         
    vertical = (self.left_line_ymax + self.left_line_ymin ) / 2 - 100                      
    temp = vertical / 6.0
    if abs(temp) > 3:
      self.head_y -= 4.5512 * temp 
      if self.head_y > 2047:
        self.head_y = 2047
      elif self.head_y < 1300:
        self.head_y = 1300
    
  
  def main(self):
    send.drawImageFunction(1, 0, 160, 160, 0, 240, 0, 0, 0)
    send.drawImageFunction(2, 0, 0, 320, 120, 120, 0, 0, 0)
    send.drawImageFunction(3, 1, 40, 280, 40, 200, 0, 0, 0)
    if send.is_start:
      rospy.loginfo(f'imu_value = {self.now_yaw}')
      if not self.strategy_flag:                                                        #修正變數用法
        self.initial()
        send.sendHeadMotor(2, 2047, 50)
        send.sendSensorReset(1, 1, 1)
        send.sendBodyAuto(0, 0, 0, 0, 1, 0)
        time.sleep(0.1)
        self.mode = 0              #choice mode 0 one color 1 two color            #修正變數用法
        self.strategy_flag = True                                                       #修正變數用法                                       
      else:
        self.now_yaw = send.imu_value_Yaw
        if not self.back_flag:                                                      #將FORWARD_GO = 0 換成false                      
          if not TWO_COLOR_FLAG:                                                      #修正變數用法
            # self.get_one_size()
            self.get_left_size(0.85, 1.15)
            self.ball_size = self.left_size
                                                             #修正變數用法
            if self.ball_size == None:                                            
              self.ball_size = 1000
              rospy.logdebug(f'no catch red ball no catch red ball')                   #顯示沒有抓取紅球 
            else:
              self.get_target_flag = True
          else:    
            self.get_left_size(1.5, 2.0)                                                        #輸出進red_size
            self.get_right_size()                                                       #輸出進blue_size
            if self.left_size == None and self.right_size != None:
              self.left_size=self.right_size
            elif self.right_size == None and self.left_size != None:
              self.right_size = self.left_size
            elif self.left_size == None and self.right_size == None:
              self.left_size = 500
              self.right_size = 500   
              rospy.logdebug(f' no ball size')
            self.ball_size_total()                      #輸出到ball_size  拿掉不必要的變數
          rospy.logdebug(f'gettarget = {self.get_target_flag}')
          if self.get_target_flag:
            if time.time() - self.control_time  >  HEAD_UPDATE_TIME:
              self.move_head()
              self.control_time = time.time()
          send.sendHeadMotor(2, round(self.head_y), 50)
          time.sleep(0.01) 
          rospy.logdebug(f'Head = {self.head_y}')
          self.forward_control()
          time.sleep(0.02)
          send.sendContinuousValue(self.forward_speed, 0, 0, self.theta_initial + THETA_FIX_FORWARD, 0)
          rospy.logdebug(f'Detection size = {self.ball_size}') 
          rospy.logdebug(f'move on move on move on')
          if self.ball_size >= BALL_FRONT_SIZE:
            self.back_flag = True
            self.buffer_fix_time = time.time() + BUFFER_TIME
          
        if self.back_flag:
          send.sendHeadMotor(2, 2047, 100)
          self.back_control() 
          time.sleep(0.02)
          send.sendContinuousValue(self.speed_back, 0, 0, self.theta_initial + THETA_FIX_BACK, 0)
          rospy.logdebug(f'Detection size = {self.ball_size}')      
          rospy.logdebug(f'go back go back go back')

    if not send.is_start:                         #初始化
      if self.strategy_flag:
          send.sendBodyAuto(0, 0, 0, 0, 1, 0)
          self.initial()
          send.sendHeadMotor(2, 2047, 100)
          self.strategy_flag = False 
            
if __name__ == '__main__':
  try:
    program = Sprint()                           
    r = rospy.Rate(30)                     #更新率 範圍限制 20~30
    while not rospy.is_shutdown(): 
      program.main()                      
      r.sleep()
  except rospy.ROSInterruptException:
    pass