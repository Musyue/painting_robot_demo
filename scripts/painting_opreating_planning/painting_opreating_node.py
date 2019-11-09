#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from math import *
from std_msgs.msg import String,Float64,Bool
import time

import os
class PaintingOpreat():
    def __init__(self):
        pass
    def Init_node(self):
        rospy.init_node("painting_opreator_node")

def main():
    ratet=1
    Aub=PaintingOpreat()
    Aub.Init_node()
    rate = rospy.Rate(ratet)
    rospy.set_param('open_control_mobile_platform',1)
    climb_go_up_down_flag=0
    w_count=3
    path_num=6
    while not rospy.is_shutdown():
        open_control_mobile_platform=rospy.get_param('open_control_mobile_platform')
        mobile_tracking_stop_flag=rospy.get_param('mobile_tracking_stop_flag')
        climb_distance_tracking_over=rospy.get_param('climb_distance_tracking_over')
        rotation_distance_tracking_over=rospy.get_param('rotation_distance_tracking_over')
        painting_oprea_over=rospy.get_param('painting_oprea_over')
        if path_num>0:
            if w_count>0:
                if mobile_tracking_stop_flag==1:
                    rospy.loginfo("mobile tracking over climb go up----")
                    #enable climb robot
                    rospy.set_param('enable_climb_control',1)
                    #set climb velocity
                    rospy.set_param('velocity_climb_control',1000)
                    if climb_distance_tracking_over==0:
                        if climb_go_up_down_flag==0:
                            rospy.set_param('distance_control_stand_bar',3)#climb 3cm
                        else:
                            rospy.set_param('distance_control_stand_bar',-3)#climb 3cm
                    if climb_distance_tracking_over==1:
                        rospy.set_param('enable_control_rotation',1)
                        #set climb velocity
                        rospy.set_param('velocity_control_rotation',1000)
                        if rotation_distance_tracking_over==0:
                            rospy.set_param('rad_control_rotation',pi/2)#climb 3cm
                        if rotation_distance_tracking_over==1:
                            rospy.set_param('open_aubo_oprea_flag',1)
                            if painting_oprea_over==1:
                                w_count-=1
                                climb_go_up_down_flag=1
                                rospy.set_param('climb_distance_tracking_over',0)
                                rospy.set_param('open_aubo_oprea_flag',0)
                                rospy.set_param('painting_oprea_over',0)
                                if w_count==0:
                                    rospy.set_param('open_control_mobile_platform',1)
                                    path_num-=1
                                    w_count=3

        else:
            rospy.loginfo("-------all path over-----")
            rospy.set_param('climb_distance_tracking_over',0)
            rospy.set_param('open_aubo_oprea_flag',0)
            rospy.set_param('painting_oprea_over',0)
            rospy.set_param('enable_control_rotation',0)
            rospy.set_param('enable_climb_control',0)
        rate.sleep()

if __name__ == '__main__':
    main()