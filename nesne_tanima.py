#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Find-Object ile Nesne Tanıma Uygulaması
"""

import rospy 
from find_object_2d.msg import ObjectsStamped

class NesneTanima():
    def __init__(self):
        rospy.init_node("nesne_tanima")
        rospy.Subscriber("objectsStamped",ObjectsStamped,self.nesneTani)
        rospy.spin()
        
    def nesneTani(self,mesaj):
        try:
            self.nesne_id = mesaj.objects.data[0]
            print(self.nesne_id)
            if self.nesne_id ==8:
                print("Koni Bulundu")
        except IndexError:
            print("Herhangi bir nesne bulunamadi !!!")
            
NesneTanima()