#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from std_msgs.msg import String,Float64,Bool
from aubo_robotcontrol import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TwistStamped
from aubo_bringup.msg import catersian_vel

import numpy as np
import time
import re
import os
class AuboRosDriver():
    def __init__(self):
        Auboi5Robot.initialize()
        self.robot = Auboi5Robot()
        
        self.aubo_joint_movej_sub = rospy.Subscriber('/aubo_ros_script/movej', String, self.aubo_joint_movej, queue_size=1)
        self.aubo_joint_movel_sub = rospy.Subscriber('/aubo_ros_script/movel', String, self.aubo_joint_movel, queue_size=1)
        self.aubo_joint_movet_sub = rospy.Subscriber('/aubo_ros_script/movet', String, self.aubo_joint_movet, queue_size=1)

        self.move_to_point=[]
        self.move_line_points={}

        self.joint_maxacctuple=self.Tuple_string_to_tuple(rospy.get_param('/aubo_startup_ns/joint_maxacc_tuple'))
        self.joint_maxvelctuple=self.Tuple_string_to_tuple(rospy.get_param('/aubo_startup_ns/joint_maxvelc_tuple'))
        self.ee_maxacc=rospy.get_param('/aubo_startup_ns/ee_maxacc')    
        self.ee_maxvelc=rospy.get_param('/aubo_startup_ns/ee_maxvelc')    
        self.blend_radius=rospy.get_param('/aubo_startup_ns/blend_radius')

    def Init_node(self):
        rospy.init_node("aubo_driver_node1")

    def aubo_joint_movej(self,msg):
        tuplefloatdata=self.Tuple_string_to_tuple(msg.data)
        if "movej" in msg.data:
            #set joint max acc
            self.robot.set_joint_maxacc(self.Tuple_string_to_tuple(rospy.get_param('/aubo_startup_ns/joint_maxacc_tuple')))
            #set joint max vel
            self.robot.set_joint_maxvelc(self.Tuple_string_to_tuple(rospy.get_param('/aubo_startup_ns/joint_maxvelc_tuple')))
            #set ee max acc
            self.robot.set_end_max_line_acc(rospy.get_param('/aubo_startup_ns/ee_maxacc'))
            #set ee max vel
            self.robot.set_end_max_line_velc(rospy.get_param('/aubo_startup_ns/ee_maxvelc'))
            #add waypoints
            rospy.loginfo("movej start point={0}".format(tuplefloatdata[0:6]))
            self.move_to_point=tuplefloatdata
            flag=self.robot.move_joint(tuplefloatdata[0:6])
            if flag:
                rospy.loginfo("movej command work successfully")
            else:
                rospy.logerr("movej command doesn't work")
        else:
            rospy.logerr("Please send right movej message")
    def aubo_joint_movel(self,msg):
        tuplefloatdata=self.Tuple_string_to_tuple(msg.data)
        if "movel" in msg.data:
            #set joint max acc
            self.robot.set_joint_maxacc(self.Tuple_string_to_tuple(rospy.get_param('/aubo_startup_ns/joint_maxacc_tuple')))
            #set joint max vel
            self.robot.set_joint_maxvelc(self.Tuple_string_to_tuple(rospy.get_param('/aubo_startup_ns/joint_maxvelc_tuple')))
            #set ee max acc
            self.robot.set_end_max_line_acc(rospy.get_param('/aubo_startup_ns/ee_maxacc'))
            #set ee max vel
            self.robot.set_end_max_line_velc(rospy.get_param('/aubo_startup_ns/ee_maxvelc'))
            #add waypoints
            rospy.loginfo("movel start point={0}".format(tuplefloatdata[0:6]))
            rospy.loginfo("movel end point={0}".format(tuplefloatdata[6:]))
            self.move_line_points={"startpoint":tuplefloatdata[0:6],"endpoint":tuplefloatdata[6:]}
            self.robot.move_joint(tuplefloatdata[0:6])
            flag=self.robot.move_line(tuplefloatdata[6:])
            if flag:
                rospy.loginfo("movel command work successfully")
            else:
                rospy.logerr("movel command doesn't work")
        else:
            rospy.logerr("Please send right movel message")
    def aubo_joint_movet(self,msg):
        tuplefloatdata=self.Tuple_string_to_tuple(msg.data)
        if "movet" in msg.data:
            #set joint max acc
            self.robot.set_joint_maxacc(self.Tuple_string_to_tuple(rospy.get_param('/aubo_startup_ns/joint_maxacc_tuple')))
            #set joint max vel
            self.robot.set_joint_maxvelc(self.Tuple_string_to_tuple(rospy.get_param('/aubo_startup_ns/joint_maxvelc_tuple')))
            #set ee max acc
            self.robot.set_end_max_line_acc(rospy.get_param('/aubo_startup_ns/ee_maxacc'))
            #set ee max vel
            self.robot.set_end_max_line_velc(rospy.get_param('/aubo_startup_ns/ee_maxvelc'))
            #set blender radius
            self.robot.set_blend_radius(blend_radius=rospy.get_param('/aubo_startup_ns/blend_radius'))
            #add waypoints
            waypoints_num=len(tuplefloatdata)/6
            # for i in range(waypoints_num):
            #     rospy.loginfo("movet waypoints={0}".format(tuplefloatdata[6*i:6*(i+1)]))
            self.robot.move_joint(tuplefloatdata[0:6])
            self.robot.remove_all_waypoint()
            for i in range(waypoints_num):
                self.robot.add_waypoint(tuplefloatdata[6*i:6*(i+1)])
            flag=self.robot.move_track(RobotMoveTrackType.CARTESIAN_MOVEP)

            if flag:
                rospy.loginfo("movet command work successfully")
            else:
                rospy.logerr("movet command doesn't work")
        else:
            rospy.logerr("Please send right movet message")

    def Tuple_string_to_tuple(self,tuplestring):
        tupletemp = re.findall(r'\-?\d+\.?\d*', tuplestring)
        resdata=[]
        for i in tupletemp:
            resdata.append(float(i))
        return tuple(resdata)

    def deg_to_rad(self,tuplelist):
        dd = []
        for i in tuplelist:
            dd.append(i * math.pi / 180)
        return tuple(dd)

    def Init_aubo_driver(self,Aubo_IP):
        # 初始化logger
        #logger_init()
        # 启动测试
        rospy.loginfo("{0} test beginning...".format(Auboi5Robot.get_local_time()))
        # 系统初始化
        # Auboi5Robot.initialize()
        # # 创建机械臂控制类
        # robot = Auboi5Robot()
        # 创建上下文
        handle = self.robot.create_context()
        # 打印上下文
        rospy.loginfo("robot.rshd={0}".format(handle))
        try:

            # 链接服务器
            ip = Aubo_IP#'192.168.1.11'
            port = 8899
            result = self.robot.connect(ip, port)
            if result != RobotErrorType.RobotError_SUCC:
                rospy.loginfo("connect server{0}:{1} failed.".format(ip, port))
            else:
                # # 重新上电
                # robot.robot_shutdown()
                #
                # # 上电
                #robot.robot_startup()
                #
                # # 设置碰撞等级
                # robot.set_collision_class(7)

                # 设置工具端电源为１２ｖ
                # robot.set_tool_power_type(RobotToolPowerType.OUT_12V)

                # 设置工具端ＩＯ_0为输出
                self.robot.set_tool_io_type(RobotToolIoAddr.TOOL_DIGITAL_IO_0, RobotToolDigitalIoDir.IO_OUT)

                # 获取工具端ＩＯ_0当前状态
                tool_io_status = self.robot.get_tool_io_status(RobotToolIoName.tool_io_0)
                rospy.loginfo("tool_io_0={0}".format(tool_io_status))

                # 设置工具端ＩＯ_0状态
                self.robot.set_tool_io_status(RobotToolIoName.tool_io_0, 1)

                # 获取控制柜用户DI
                io_config = self.robot.get_board_io_config(RobotIOType.User_DI)

                # 输出DI配置
                rospy.loginfo(io_config)

                # 获取控制柜用户DO
                io_config = self.robot.get_board_io_config(RobotIOType.User_DO)

                # 输出DO配置
                rospy.loginfo(io_config)
                # 当前机械臂是否运行在联机模式
                rospy.loginfo("robot online mode is {0}".format(robot.is_online_mode()))
                # initial file system
                self.robot.init_profile()
                # self.Aubo_trajectory_init(maxacctuple,maxvelctuple,ee_maxacc,ee_maxvelc,blend_radius)
        except RobotError,e:
            rospy.logerr("{0} robot Event:{1}".format(self.robot.get_local_time(), e))
        return robot
    def AuboStartPower(self):
        self.robot.robot_startup()
    def AuboDownPower(self):
        self.robot.robot_shutdown()
    def DisConnect_Aubo_No_ShutDown(self):
        # 断开服务器链接
        self.robot.disconnect()
    def DisConnect_Aubo(self):
        # 断开服务器链接
        if self.robot.connected:
            # 关闭机械臂
            self.robot.robot_shutdown()
            # 断开机械臂链接
            self.robot.disconnect()
        # 释放库资源
        self.robot.uninitialize()
        rospy.loginfo("{0} test completed.".format(self.robot.get_local_time()))



def main():
    ratet=30
    Aub=AuboRosDriver()

    Aub.Init_node()
    rate = rospy.Rate(ratet)

    IPP=rospy.get_param('/aubo_startup_ns/aubo_ip')

    try:
        Aub.Init_aubo_driver(IPP)
    except:
        logger.error("Aubo robot disconnect,Please check!")
    try:
        while not rospy.is_shutdown():
            print("send commands to aubo driver-----")
            rate.sleep()
    except:
        rospy.logerr("ros node down")
        Aub.robot.remove_all_waypoint()
if __name__ == '__main__':
    main()