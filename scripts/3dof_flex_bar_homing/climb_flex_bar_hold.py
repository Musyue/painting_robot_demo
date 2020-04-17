#!/usr/bin/env python
# -*- coding: utf_8 -*-
"""
id：1---->stand bar
id:2----->roation
id:3------>Upper and lower climbing pole
"""
import rospy
import time
import os


class FLEX3DOFROBOTHOLD():
    def __init__(self,nodename):
        self.nodename=nodename
    def Init_node(self):
        rospy.init_node(self.nodename)
    def stand_bar_and_flexiable_part_hold_to_ceil(self,up_distance):

        rospy.loginfo("flex pole up-----")
        os.system('rosparam set /renov_up_level/enable_control_stand_bar 1')
        time.sleep(0.05)
        os.system('rosparam set /renov_up_level/enable_control_stand_bar 1')
        os.system('rosparam set /renov_up_level/enable_control_stand_bar 0')
        os.system('rosparam set /renov_up_level/write_flex_pole_motor_up 1')
        time.sleep(0.05)
        os.system('rosparam set /renov_up_level/write_flex_pole_motor_up 1')
    
        time.sleep(10)
        rospy.loginfo("waiting for flex pole go to point")
        os.system('rosparam set /renov_up_level/write_flex_pole_motor_up 0')
        # rospy.set_param('distance_control_stand_bar',10)#30cm
        os.system('rosparam set /renov_up_level/distance_control_stand_bar '+str(up_distance))
        time.sleep(0.05)
        os.system('rosparam set /renov_up_level/distance_control_stand_bar '+str(up_distance))
        os.system('rosparam set /renov_up_level/open_hold_flag 1')
        time.sleep(0.05)
        os.system('rosparam set /renov_up_level/open_hold_flag 1')
        # time.sleep(10)
        # os.system('rosparam set /renov_up_level/open_hold_flag 0')
        rospy.loginfo("waiting for stand bar go to point")
    def caculate_top_to_ceil_distance(self,stand_bar_flex_distance,light_scan_to_top_distance,light_scan_to_ceil_distance):
        detax=light_scan_to_ceil_distance-light_scan_to_top_distance-stand_bar_flex_distance
        if detax<-0.0:
            rospy.logerr("you can not run the system here,the ceil is too low----")
            return 0
        else:
            return detax
    
def main():
    nodename="climb_flex_bar_hold_node"

    hc3dof=FLEX3DOFROBOTHOLD(nodename)

    hc3dof.Init_node()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        light_scan_to_ceil_distance = rospy.get_param("light_scan_to_ceil_distance")
        light_scan_to_top_distance = rospy.get_param("light_scan_to_top_distance")
        stand_bar_flex_distance = rospy.get_param("stand_bar_flex_distance")
        hold_distance_tracking_over = rospy.get_param("hold_distance_tracking_over")
        open_hold_to_ceil_flag= rospy.get_param("open_hold_to_ceil_flag")
        if open_hold_to_ceil_flag==1:
            detax=hc3dof.caculate_top_to_ceil_distance(stand_bar_flex_distance,light_scan_to_top_distance,light_scan_to_ceil_distance)
            rospy.logerr("detax---%s",detax)
            hc3dof.stand_bar_and_flexiable_part_hold_to_ceil(detax+0.05)
            # os.system('rosparam set /renov_up_level/open_hold_flag 0')
            os.system('rosparam set /renov_up_level/open_hold_to_ceil_flag 0')
        if hold_distance_tracking_over==1:
            os.system('rosparam set /renov_up_level/open_hold_flag 0')
            os.system('rosparam set /renov_up_level/hold_distance_tracking_over 0')
            rospy.logerr("hold over-------congratulations----")
            # rospy.set_param('open_hold_to_ceil_flag',0)
        rate.sleep()

if __name__=="__main__":
    main()
