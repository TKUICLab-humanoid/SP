#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from Python_API import Sendmessage
import time

def main():
    rospy.init_node('aa', anonymous=True, log_level=rospy.INFO)
    send = Sendmessage()
    r = rospy.Rate(30)
    first_in = True
    while not rospy.is_shutdown():                                  
        
        if send.is_start:
            if first_in:
                send.sendBodyAuto(0, 0, 0, 0, 1, 0)
                first_in = False            
        else:
            if not first_in:
                # send.sendBodyAuto(0, 0, 0, 0, 1, 0)
                first_in = True

        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass