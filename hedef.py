#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot hedefi gorunce duruyor fakat dönerken tam açıyla hedef noktayı goremedigi
icin takiliyor sadece denemek istedim 
"""

import rospy 
from find_object_2d.msg import ObjectsStamped 
from ogretici_paket.msg import EsmanurunGeometrisi
class NesneTanima():
    def __init__(self):
        rospy.init_node("nesne_tanima")
        self.pub = rospy.Publisher("cmd_vel",EsmanurunGeometrisi,queue_size = 10)
        self.hiz_mesaji = EsmanurunGeometrisi()
        rospy.Subscriber("objectsStamped",ObjectsStamped,self.nesneTani)
        rospy.spin()
        
    def nesneTani(self,mesaj):
        try:
            self.nesne_id = mesaj.objects.data[0]
            print(self.nesne_id)
            if self.nesne_id ==7:
                print("Koni Bulundu istediğimiz yere vardik")
                self.hiz_mesaji.linear.x = 0.0
                self.hiz_mesaji.angular.z = 0.0
                self.pub.publish(self.hiz_mesaji)
            else: 
                self.hiz_mesaji.linear.x = 0.1
                self.hiz_mesaji.angular.z = 0.1
                self.pub.publish(self.hiz_mesaji)
                
            
        except IndexError:
            print("Herhangi bir nesne bulunamadi !!!")
            self.hiz_mesaji.linear.x = 0.1
            self.hiz_mesaji.angular.z = 0.1
            self.pub.publish(self.hiz_mesaji)
            
NesneTanima()