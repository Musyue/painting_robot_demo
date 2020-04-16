#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from math import *
from std_msgs.msg import String,Float64,Bool
import time
import commands
import os
from coverage_planning_offline_without_sensor import *
from geometry_msgs.msg import PoseStamped
import tf
class PaintingOpreat():
    def __init__(self):
        self.mobile_go_point_pub = rospy.Publisher('/renov_down_mobile/mobile_go_to_point', PoseStamped, queue_size=1)
    def Init_node(self):
        rospy.init_node("painting_opreating_node_with_bim_model")
    
    def euler_to_quaternion(self,euler_data):
        return tf.transformations.quaternion_from_euler(euler_data[0],euler_data[1],euler_data[2])
    def pub_posestamped(self,frame_id,posedata,euler_data):
        p = PoseStamped()
        p.header.frame_id = frame_id
        p.header.stamp = rospy.Time.now()
        p.pose.position.x=posedata[0]
        p.pose.position.y=posedata[1]
        p.pose.position.z=0
        q = self.euler_to_quaternion(euler_data)
        p.pose.orientation = Quaternion(*q)
        self.mobile_go_point_pub.publish(p)
def main():
    ratet=1
    Aub=PaintingOpreat()
    Aub.Init_node()
    rate = rospy.Rate(ratet)
    planning_source_dict={}
    mat_path=rospy.get_param('mat_data_path')#"/data/ros/renov_robot_ws/src/painting_robot_demo/data/data.mat"
    parameterx=rospy.get_param('mat_parameterx')#0.430725381079
    parametery=rospy.get_param('mat_parametery')#-0.00033063639818
    parameterz=rospy.get_param('mat_parameterz')#0.028625
    interval=rospy.get_param('mat_interval')#0.10
    rbmo=Renovation_BIM_Model_Opreating(parameterx,parametery,parameterz,interval)
    planning_source_dict=rbmo.get_mat_data_json()
    
    rospy.set_param('mobile_path_way_planning',1)
    climb_go_up_down_flag=0
    open_stand_bar_flag=0
    total_plane_num=len(planning_source_dict)
    plane_num_count=0
    

    climb_minus_count=0
    mobile_path_way_num=1
    close_all_path_flag=0
    mobile_base_count_num=0
    climb_base_count_num=0

    """
    This is main program,the main control logical is used time sequence to controll all state.so you may see some time.sleep function
    Optimizing method:get delay time from node,not set by mannual.
    """
    while not rospy.is_shutdown():
        mobile_tracking_stop_flag=rospy.get_param('mobile_tracking_stop_flag')


        climb_distance_tracking_over=rospy.get_param('climb_distance_tracking_over')
        rotation_distance_tracking_over=rospy.get_param('rotation_distance_tracking_over')


        painting_oprea_over=rospy.get_param('painting_opreating_over')

        top_limit_switch_status=rospy.get_param('top_limit_switch_status')
        read_line_encode=rospy.get_param('read_line_encode')
        climb_max_length=rospy.get_param('climb_max_length')
        

        if total_plane_num>0 and plane_num_count<=total_plane_num-1:
            rospy.logerr("-------plane_num_count----%d",plane_num_count)

            if len(planning_source_dict)!=0:
                if len(planning_source_dict["plane_num_"+str(plane_num_count)])!=0:
                    if mobile_base_count_num<len(planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(mobile_base_count_num)]):

                        Aub.pub_posestamped(planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(mobile_base_count_num)])
                        if mobile_tracking_stop_flag==1:

                            if top_limit_switch_status!=1 and open_stand_bar_flag==0:
                                
                                os.system('rosparam set /renov_up_level/open_hold_to_ceil_flag 1')
                                time.sleep(0.05)
                                os.system('rosparam set /renov_up_level/open_hold_to_ceil_flag 1')
                                os.system('rosparam set /renov_up_level/open_hold_to_ceil_flag 0')
                                open_stand_bar_flag=1
                            if open_stand_bar_flag==1 and climb_base_count_num<len(planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_climb_num_"+str(mobile_base_count_num)]):
                                climb_data=planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_climb_num_"+str(mobile_base_count_num)]["climb_num_"+ str(climb_base_count_num)]
                                climb_height=climb_data[0]
                                climb_rotation=climb_data[1]
                                tempstr='rosparam set /renov_up_level/distance_climb_control '+str(climb_height)
                                os.system(tempstr)
                                if climb_distance_tracking_over==1:
                                    if rotation_distance_tracking_over==1:
                                        
                                os.system('rosparam set /renov_up_level/climb_distance_tracking_over 0')
                                os.system('rosparam set /renov_up_level/rotation_distance_tracking_over 0')        
                                climb_base_count_num+=1

                    mobile_base_count_num+=1


            else：
                rospy.logerr("Your mat data error,please check------")
            climb_base_count_num=0
            mobile_base_count_num=0
            plane_num_count+=1  
        else:
            rospy.loginfo("-------all path over-----")
            rospy.set_param('climb_distance_tracking_over',0)
            rospy.set_param('open_aubo_oprea_flag',0)
            rospy.set_param('painting_oprea_over',0)
            rospy.set_param('enable_control_rotation',2)
            rospy.set_param('enable_control_rotation',0)
            rospy.set_param('enable_climb_control',2)
            rospy.set_param('enable_climb_control',0)

            rospy.set_param('enable_control_stand_bar',2)
            rospy.set_param('enable_control_stand_bar',0)
            rospy.set_param("home_climb_flex_bar",0)
            if close_all_path_flag==0:
                os.system('rosparam set /renov_up_level/mobile_path_way_planning '+str(0))
                rospy.logerr("I will go to [ %s ]the mobile_path_way_point-----",str(0))
                os.system('rosparam set /renov_up_level/open_control_mobile_platform 0')
                time.sleep(0.05)
                os.system('rosparam set /renov_up_level/open_control_mobile_platform 0')
                close_all_path_flag=1           
        rate.sleep()

if __name__ == '__main__':
    main()