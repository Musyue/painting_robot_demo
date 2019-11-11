#!/usr/bin/env python
# -*- coding: utf_8 -*-
"""
id：1---->stand bar
id:2----->roation
id:3------>Upper and lower climbing pole
"""
import rospy
import time



class FLEX3DOFROBOTHOME():
    def __init__(self):
        pass
    def Init_node(self):
        rospy.init_node(self.nodename)
    
def main():
    nodename="climb_flex_bar_home_node"

    hc3dof=FLEX3DOFROBOTHOME(nodename)

    hc3dof.Init_node()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        home_climb_flex_bar = rospy.get_param("home_climb_flex_bar")
        rospy.loginfo("%s is %s", rospy.resolve_name('home_climb_flex_bar'), home_climb_flex_bar)
        write_flex_pole_motor_down = rospy.get_param("write_flex_pole_motor_down")
        rospy.loginfo("%s is %s", rospy.resolve_name('write_flex_pole_motor_down'), write_flex_pole_motor_down)
        if home_climb_flex_bar==1:
            rospy.set_param('distance_control_stand_bar',0)
            time.sleep(5)
            rospy.loginfo("waiting for stand bar go to start point")
            rospy.set_param('write_flex_pole_motor_down',2)
            time.sleep(5)
            rospy.loginfo("waiting for flex bar go down to start point")
            rospy.set_param('write_flex_pole_motor_down',0)
            rospy.set_param('distance_climb_control',0)
            time.sleep(2)
            rospy.loginfo("waiting for climb robot go down to start point")
            rospy.set_param('rad_control_rotation',0)
            time.sleep(3)
            rospy.loginfo("waiting for rotation robot go  to start rad/degree")            
            rospy.set_param('home_climb_flex_bar',0)
        rate.sleep()

if __name__=="__main__":
    main()
