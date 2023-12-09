#!/usr/bin/env python3
# -*- coding: UTF-8 -*-


import rospy, cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from ogretici_paket.msg import EsmanurunGeometrisi


class Object():

    def __init__(self):
        
        self.bridge = CvBridge()
        
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',Image, self.func)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', EsmanurunGeometrisi, queue_size=1)
        self.lidar = rospy.Subscriber('/scan',LaserScan, self.lazerCallBack)
        self.mesafe = True
        self.hareket = EsmanurunGeometrisi()
        
    def lazerCallBack(self,data):
        sol_on = list(data.ranges[0:9])
        sag_on = list(data.ranges[350:359])
        on = sol_on + sag_on
        min_on = min(on)
        if min_on > 1:
            self.mesafe = True
        else:
            self.mesafe = False

    def func(self,ros_goruntu):

        cv_goruntu = self.bridge.imgmsg_to_cv2(ros_goruntu, 'bgr8')
        # redBGR = numpy.uint8([[[0,255,0 ]]]) ---> [[[ 60 255 255]]]
        hsv_goruntu = cv2.cvtColor(cv_goruntu, cv2.COLOR_BGR2HSV)
        
        # lower boundary RED color range values; Hue (0 - 10)
        lower1 = np.array([0, 100, 20])
        upper1 = np.array([10, 255, 255])

        # upper boundary RED color range values; Hue (160 - 180)
        lower2 = np.array([160,100,20])
        upper2 = np.array([179,255,255])
        
        lower_mask = cv2.inRange(hsv_goruntu, lower1, upper1)
        upper_mask = cv2.inRange(hsv_goruntu, lower2, upper2)
        
        full_mask = lower_mask + upper_mask;

        cv2.imshow('maske', full_mask)
        
        h, w, d = hsv_goruntu.shape

        cv2.imshow('kirpilmis_maske', full_mask)
        
        M = cv2.moments(full_mask)

        if self.mesafe == True:   
            if M['m00'] > 0:
                
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                
                cv2.circle(cv_goruntu, (cx, cy), 20, (0,0,255), -1)
                
                err = cx - w/2
            
                self.hareket.linear.x = 0.1
                self.hareket.angular.z = -float(err) / 100
                self.cmd_vel_pub.publish(self.hareket)

            else:
                self.hareket.linear.x = 0.01
                self.hareket.angular.z = 0.5
                self.cmd_vel_pub.publish(self.hareket)
        else:
            self.hareket.linear.x = 0.0
            self.hareket.angular.z = 0.0
            self.cmd_vel_pub.publish(self.hareket)

        
        cv2.imshow("To Tracking Center of Object",cv_goruntu)
        cv2.waitKey(3)
        

if __name__ == "__main__":

    rospy.init_node("object_midpoint_tracking")
    obje = Object()
    rospy.spin()
    
