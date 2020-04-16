#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from math import *
from std_msgs.msg import String,Float64,Bool
import time
import commands
import os
from coverage_planning_offline_without_sensor import *
from geometry_msgs.msg import PoseStamped,Quaternion
import tf
class PaintingOpreat():
    def __init__(self):
        self.mobile_go_point_pub = rospy.Publisher('/renov_down_mobile/mobile_go_to_point', PoseStamped, queue_size=1)
        self.aubo_move_track_pub = rospy.Publisher('/aubo_ros_script/movet', String, queue_size=10)
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
    def group_joint_to_string(self,q_list):
        resdata=""
        for i in range(len(q_list)):
            resdata+=str(tuple(q_list))
        return resdata
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
    rbmo=Renovation_BIM_Model_Opreating(mat_path,parameterx,parametery,parameterz,interval)
    planning_source_dict=rbmo.get_mat_data_json()
    
    rospy.set_param('mobile_path_way_planning',1)

    open_stand_bar_flag=0
    total_plane_num=len(planning_source_dict)
    plane_num_count=0
    


    mobile_base_point_count=0
    climb_base_count_num=0
    mobile_point_send_flag=0
    aubo_open_onece_flag=0
    """
    This is main program,the main control logical is used time sequence to controll all state.so you may see some time.sleep function
    Optimizing method:get delay time from node,not set by mannual.
    """
    while not rospy.is_shutdown():
        mobile_tracking_stop_flag=rospy.get_param('mobile_tracking_stop_flag')


        climb_distance_tracking_over=rospy.get_param('climb_distance_tracking_over')
        rotation_distance_tracking_over=rospy.get_param('rotation_distance_tracking_over')


        aubo_painting_oprea_over=rospy.get_param('aubo_painting_opreating_over')

        top_limit_switch_status=rospy.get_param('top_limit_switch_status')

        home_climb_flex_bar=rospy.get_param('home_climb_flex_bar')
        

        if total_plane_num>0 and plane_num_count<=total_plane_num-1:
            rospy.logerr("-------plane_num_count----%d",plane_num_count)

            if len(planning_source_dict)!=0:
                if len(planning_source_dict["plane_num_"+str(plane_num_count)])!=0:
                    if mobile_base_point_count<len(planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]):
                        if mobile_tracking_stop_flag==0 and mobile_point_send_flag==0:
                            mobiledata=planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]["mobile_data_num_"+str(mobile_base_point_count)]
                            Aub.pub_posestamped("mobile_base_link",[mobiledata[0],mobiledata[1],0],[0,0,mobiledata[2]])
                            mobile_point_send_flag=1

                        if mobile_tracking_stop_flag==1:

                            if top_limit_switch_status!=1 and open_stand_bar_flag==0:
                                rospy.loginfo("in hold to ceil----")
                                os.system('rosparam set /renov_up_level/open_hold_to_ceil_flag 1')
                                time.sleep(0.05)
                                os.system('rosparam set /renov_up_level/open_hold_to_ceil_flag 1')
                                os.system('rosparam set /renov_up_level/open_hold_to_ceil_flag 0')
                                open_stand_bar_flag=1
                            if open_stand_bar_flag==1 and climb_base_count_num<len(planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_climb_num_"+str(mobile_base_point_count)]):
                                climb_data=planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_climb_num_"+str(mobile_base_point_count)]["climb_num_"+ str(climb_base_count_num)]
                                climb_height=climb_data[0]
                                climb_rotation=climb_data[1]
                                tempstr='rosparam set /renov_up_level/distance_climb_control '+str(climb_height)
                                os.system(tempstr)
                                tempstr2='rosparam set /renov_up_level/rad_control_rotation '+str(climb_rotation)
                                os.system(tempstr2)
                                if climb_distance_tracking_over==1:
                                    if rotation_distance_tracking_over==1:
                                        aubo_q_list=planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_climb_num_"+str(mobile_base_point_count)]["aubo_planning_voxel_num_"+ str(climb_base_count_num)]
                                        for i in range(len(aubo_q_list)):
                                            tempstr3=aubo_q_list["aubo_data_num_"+str(i)]
                                        if aubo_open_onece_flag==0:
                                            Aub.aubo_move_track_pub.publish(tempstr3)
                                            aubo_open_onece_flag=1
                                if aubo_painting_oprea_over==1:
                                    os.system('rosparam set /renov_up_level/climb_distance_tracking_over 0')
                                    os.system('rosparam set /renov_up_level/rotation_distance_tracking_over 0')
                                    os.system('rosparam set /renov_up_level/aubo_painting_oprea_over 0')        
                                    climb_base_count_num+=1
                                    aubo_open_onece_flag=0
                            if climb_base_count_num>len(planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_climb_num_"+str(mobile_base_point_count)]):
                                os.system('rosparam set /renov_up_level/home_climb_flex_bar 1')
                                os.system('rosparam set /renov_up_level/mobile_tracking_stop_flag 0')
                                if  home_climb_flex_bar==0:
                                    climb_base_count_num=0
                                    mobile_point_send_flag=0

                        else:
                            rospy.loginfo("please wait mobile base go to the target point----")

                        if mobile_tracking_stop_flag==0 and mobile_point_send_flag==0:
                            mobile_base_point_count+=1
                            
                if mobile_base_point_count >= len(planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]):
                    plane_num_count+=1 
            else:
                rospy.logerr("some error in your mat data----")
             
        else:
            rospy.loginfo("-------all path over-----")       
        rate.sleep()

if __name__ == '__main__':
    main()