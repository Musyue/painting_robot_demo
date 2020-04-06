#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from std_msgs.msg import String,Float64,Bool
from aubo_robotcontrol import *

import time
import re
import os
class AuboRosDriver():
    def __init__(self,):
        Auboi5Robot.initialize()
        # 创建机械臂控制类
        self.robot = Auboi5Robot()
        self.aubo_joint_movej_sub = rospy.Subscriber('/aubo_ros_script/movej', String, self.aubo_joint_movej, queue_size=10)
        self.aubo_joint_movej_sub = rospy.Subscriber('/aubo_ros_script/movel', String, self.aubo_joint_movel, queue_size=10)
        self.move_to_point=[]
        self.move_line_points={}
    def Init_node(self):
        rospy.init_node("aubo_driver_node")
    def aubo_joint_movej(self,msg):
        tuplefloatdata=self.Tuple_string_to_tuple(msg.data)
        if "movej" in msg.data:
            rospy.loginfo("movej start point={0}".format(tuplefloatdata[0:6]))
            self.Aubo_Move_to_Point(tuplefloatdata[0:6])
            self.move_to_point=tuplefloatdata
        else:
            rospy.logerr("Please send right movej message")
    def aubo_joint_movel(self,msg):
        tuplefloatdata=self.Tuple_string_to_tuple(msg.data)
        if "movel" in msg.data:
            rospy.loginfo("movel start point={0}".format(tuplefloatdata[0:6]))
            rospy.loginfo("movel end point={0}".format(tuplefloatdata[6:]))
            # self.Aubo_Move_to_Point(tuplefloatdata[0:6])
            self.Aubo_Line_trajectory(tuplefloatdata[0:6],tuplefloatdata[6:])
            self.move_line_points={"startpoint":tuplefloatdata[0:6],"endpoint":tuplefloatdata[6:]}
        else:
            rospy.logerr("Please send right movel message")
    def deg_to_rad(self,tuplelist):
        dd = []
        for i in tuplelist:
            dd.append(i * math.pi / 180)
        return tuple(dd)
    def Init_aubo_driver(self,Aubo_IP,maxacctuple,maxvelctuple):
        # 初始化logger
        #logger_init()
        # 启动测试
        rospy.loginfo("{0} test beginning...".format(Auboi5Robot.get_local_time()))
        # 系统初始化
        # Auboi5Robot.initialize()
        # # 创建机械臂控制类
        # robot = Auboi5Robot()
        # 创建上下文
        handle = robot.create_context()
        # 打印上下文
        rospy.loginfo("robot.rshd={0}".format(handle))
        try:

            # 链接服务器
            ip = Aubo_IP#'192.168.1.11'
            port = 8899
            result = robot.connect(ip, port)
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
                robot.set_tool_io_status(RobotToolIoName.tool_io_0, 1)

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
                self.Aubo_trajectory_init(self.robot,maxacctuple,maxvelctuple)
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
    def Aubo_trajectory_init(self,maxacctuple,maxvelctuple):
        joint_status = self.robot.get_joint_status()
        rospy.loginfo("joint_status={0}".format(joint_status))
        # initial file system
        self.robot.init_profile()
        #set joint max acc
        self.robot.set_joint_maxacc(maxacctuple)#(2.5, 2.5, 2.5, 2.5, 2.5, 2.5)
        #set joint max vel
        self.robot.set_joint_maxvelc(maxvelctuple)#(1.5, 1.5, 1.5, 1.5, 1.5, 1.5)
        #set ee max acc
        self.robot.set_end_max_line_acc(0.5)
        #set ee max vel
        self.robot.set_end_max_line_velc(0.5)

    def Aubo_Line_trajectory(self,start_point,End_point):
        """
        start_point:rad
        End_point:rad
        """
        # joint_radian = self.deg_to_rad(start_point)
        rospy.loginfo("move joint to {0}".format(joint_radian))
        self.robot.move_joint(joint_radian)
        joint_radian = self.deg_to_rad(End_point)
        rospy.loginfo("move joint to {0}".format(joint_radian))
        self.robot.move_line(joint_radian)

    def Aubo_Move_to_Point(self,jointRad):
        """
        JointAngular:rad
        """
        # joint_radian = self.deg_to_rad(jointAngular)
        rospy.loginfo("move joint to {0}".format(joint_radian))
        self.robot.move_joint(joint_radian)
    def Tuple_string_to_tuple(self,tuplestring):
        tupletemp = re.findall(r'\-?\d+\.?\d*', tuplestring)
        resdata=[]
        for i in tupletemp:
            resdata.append(float(i))
        return tuple(resdata)
def main():
    ratet=10

    Aub=AuboRosDriver()
    Aub.Init_node()
    rate = rospy.Rate(ratet)
    flag_roation=0
    count=0
    IP=rospy.get_param('aubo_ip')
    StartPoint=Aub.Tuple_string_to_tuple(rospy.get_param('aubo_start_point'))
    maxacctuple=Aub.Tuple_string_to_tuple(rospy.get_param('ee_maxacc_tuple'))
    maxvelctuple=Aub.Tuple_string_to_tuple(rospy.get_param('ee_maxvelc_tuple'))

    try:
        
        # Robot = Aub.Init_aubo_driver(IP,maxacctuple, maxvelctuple)
        print("StartPoint",StartPoint,type(StartPoint),type(StartPoint[0]))
    except:
        logger.error("Aubo robot disconnect,Please check!")
    # finally:
    #     Aub.DisConnect_Aubo(Robot)
    try:
        while not rospy.is_shutdown():
            print("aubo driver run -----")
            rate.sleep()
    except:
        rospy.logerr("ros node down")
if __name__ == '__main__':
    main()