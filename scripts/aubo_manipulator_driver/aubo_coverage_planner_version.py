#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from std_msgs.msg import String,Float64,Bool
from aubo_robotcontrol import *
from sensor_msgs.msg import JointState
import time

import os
"""
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string[] name
float64[] position
float64[] velocity
float64[] effort  this use for open electric selector
"""
class AuboControlRos():
    def __init__(self,Aubo_IP,):
        self.Aubo_IP=Aubo_IP
        self.PubState = rospy.Publisher("/open_aubo_state_flag", Bool, queue_size=10)
        self.Pubsignal = rospy.Publisher("/signal", Bool, queue_size=10)
        self.SubJointstates=rospy.Subscriber("/aubo_joint_states_from_planner",JointState,self.aubo_joint_states_from_planner_callback)
        self.OpenElectricSwitch=0
        self.CloseElectricSwitch=0
        self.home_point_aubo=()
        self.aubo_opreating_jointstate_buffer=[]
        self.open_electric_flag_buffter=[]
    def Init_node(self):
        rospy.init_node("aubo_control_planning")
    def aubo_joint_states_from_planner_callback(self,msg):
        if len(self.aubo_opreating_jointstate_buffer)>=10:
            self.aubo_opreating_jointstate_buffer=self.aubo_opreating_jointstate_buffer[1:]
            self.aubo_opreating_jointstate_buffer.append(msg.position)

        else:
            self.aubo_opreating_jointstate_buffer.append(msg.position)
            #use for open electric
        if len(self.open_electric_flag_buffter)>=10:
            self.open_electric_flag_buffter=self.open_electric_flag_buffter[1:]
            self.open_electric_flag_buffter.append(msg.position)
            
        else:
            self.open_electric_flag_buffter.append(msg.position)
        
    def deg_to_rad(self,tuplelist):
        dd = []
        for i in tuplelist:
            dd.append(i * math.pi / 180)
        return tuple(dd)
    def Init_aubo_driver(self):
        # 初始化logger
        #logger_init()
        # 启动测试
        print("{0} test beginning...".format(Auboi5Robot.get_local_time()))
        # 系统初始化
        Auboi5Robot.initialize()
        # 创建机械臂控制类
        robot = Auboi5Robot()
        # 创建上下文
        handle = robot.create_context()
        # 打印上下文
        print("robot.rshd={0}".format(handle))
        try:

            # 链接服务器
            ip = self.Aubo_IP#'192.168.1.11'
            port = 8899
            result = robot.connect(ip, port)
            if result != RobotErrorType.RobotError_SUCC:
                print("connect server{0}:{1} failed.".format(ip, port))
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
                robot.set_tool_io_type(RobotToolIoAddr.TOOL_DIGITAL_IO_0, RobotToolDigitalIoDir.IO_OUT)

                # 获取工具端ＩＯ_0当前状态
                tool_io_status = robot.get_tool_io_status(RobotToolIoName.tool_io_0)
                print("tool_io_0={0}".format(tool_io_status))

                # 设置工具端ＩＯ_0状态
                robot.set_tool_io_status(RobotToolIoName.tool_io_0, 1)

                # 获取控制柜用户DI
                io_config = robot.get_board_io_config(RobotIOType.User_DI)

                # 输出DI配置
                print(io_config)

                # 获取控制柜用户DO
                io_config = robot.get_board_io_config(RobotIOType.User_DO)

                # 输出DO配置
                print(io_config)
                # 当前机械臂是否运行在联机模式
                print("robot online mode is {0}".format(robot.is_online_mode()))
        except RobotError,e:
            logger.error("{0} robot Event:{1}".format(robot.get_local_time(), e))
        return robot
    def DisConnect_Aubo_No_ShutDown(self,auboRobot):
        # 断开服务器链接
        auboRobot.disconnect()
    def DisConnect_Aubo(self,auboRobot):
        # 断开服务器链接
        if auboRobot.connected:
            # 关闭机械臂
            auboRobot.robot_shutdown()
            # 断开机械臂链接
            auboRobot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()
        print("{0} test completed.".format(Auboi5Robot.get_local_time()))
    def Aubo_trajectory_init(self,robot,maxacctuple,maxvelctuple):
        joint_status = robot.get_joint_status()
        print("joint_status={0}".format(joint_status))
        # 初始化全局配置文件
        robot.init_profile()
        # 设置关节最大加速度
        robot.set_joint_maxacc(maxacctuple)#(2.5, 2.5, 2.5, 2.5, 2.5, 2.5)

        # 设置关节最大加速度
        robot.set_joint_maxvelc(maxvelctuple)#(1.5, 1.5, 1.5, 1.5, 1.5, 1.5)
        # 设置机械臂末端最大线加速度(m/s)
        robot.set_end_max_line_acc(0.5)
        # 获取机械臂末端最大线加速度(m/s)
        # robot.set_end_max_line_velc(0.2)
        robot.set_end_max_line_velc(0.5)
    def Aubo_forward_kinematics(self,robot,jointangular):
        joint_radian = self.deg_to_rad(jointangular)
        fk_ret = robot.forward_kin(joint_radian)
        print("fk--------")
        print(fk_ret)
        return fk_ret
    def Aubo_inverse_kinematics(self,robot,jointangular,newpose,neworientaion_Quaternion):
        # 获取关节最大加速度
        print(robot.get_joint_maxacc())
        joint_radian = jointangular#self.deg_to_rad(jointangular)
        print("pose and ori")
        print(newpose)
        print(neworientaion_Quaternion)
        pos_test = newpose#(-0.5672477590258516, 0.51507448660946279, 0.57271770314023)  # the right
        ori_test = neworientaion_Quaternion#(0.49380082661500474, 0.5110471735827042, -0.5086787259664434, -0.48604267688817565)
        print("----ik----after--------")
        ik_result = robot.inverse_kin(joint_radian, pos_test, ori_test)
        print(ik_result)
        return ik_result

    def Aubo_Move_to_Point(self,robot,jointAngular):
        joint_radian = self.deg_to_rad(jointAngular)
        print("move joint to {0}".format(joint_radian))
        robot.move_joint(joint_radian)
    def Control_aubo_from_jointstate(self,robot,cn):
        if len(self.aubo_opreating_jointstate_buffer)!=0:
            robot.move_joint(tuple(self.aubo_opreating_jointstate_buffer[cn]))
            if len(self.open_electric_flag_buffter)!=0:
                if self.open_electric_flag_buffter[cn][0]==1:
                    rospy.loginfo("Go to the first point")
                    time.sleep(5)
                    os.system('rosparam set /search_port/write_electric_switch_painting_open 1')
                elif self.open_electric_flag_buffter[cn][0]==-1:
                    rospy.loginfo("Go to this opreating point last joint waypoint")
                    time.sleep(5)
                    rospy.logerr("close electric switch-------")
                    os.system('rosparam set /search_port/write_electric_switch_painting_close 1')
                    os.system('rosparam set /search_port/write_electric_switch_painting_open 0')
                    
                    # time.sleep(1.2)
                    time.sleep(0.15)
                    os.system('rosparam set /search_port/write_electric_switch_painting_close 0')
                    self.open_electric_flag_buffter=[]
                    self.aubo_opreating_jointstate_buffer=[]
                    os.system('rosparam set /search_port/open_aubo_oprea_flag 0')
                else:
                    pass

def main():
    ratet=10
    IP=rospy.get_param('aubo_ip')
    StartPoint=tuple(rospy.get_param('aubo_start_point'))
    PullbacktobarPoint=(0,0,0,0,0,0)
    opreating_start_point_aubo=(-3.07,-2.094,80.07,64.0747,95.11,89.98)

    maxacctuple=(2.5, 2.5, 2.5, 2.5, 2.5, 2.5)
    maxvelctuple=(1.5, 1.5, 1.5, 1.5, 1.5, 1.5)
    Aub=AuboTrajectory(IP)
    Aub.Init_node()
    rate = rospy.Rate(ratet)
    flag_roation=0
    count=0
    try:
        Robot = Aub.Init_aubo_driver()
        Aub.Aubo_trajectory_init(Robot, maxacctuple, maxvelctuple)
    except:
        rospy.loginfo("init aubo not OK")
    try:
        while not rospy.is_shutdown():
            aubo_go_back_initial_point=rospy.get_param('aubo_go_back_initial_point')
            open_aubo_oprea_flag=rospy.get_param('open_aubo_oprea_flag')
            if aubo_go_back_initial_point==1:
                # Aub.open_home_point=1
                Aub.Aubo_Move_to_Point(Robot,PullbacktobarPoint)
                rospy.set_param('aubo_go_back_initial_point',0)
            if open_aubo_oprea_flag==1:
                Aub.Control_aubo_from_jointstate(Robot,count)
                time.sleep(1)
                rospy.loginfo("Sleep time is over,then climb robot goes to initial point")
                # rospy.set_param('open_aubo_oprea_flag',0)
                count+=1
                if count>len(Aub.aubo_opreating_jointstate_buffer):
                    count=0
                    rospy.loginfo("All waypoint finished")
            else:
                rospy.loginfo("Please wait Mobile platform waypoint over")
                count=0
            rate.sleep()
    except:
        pass
    #except:
      # logger.error("Aubo or Climb robot disconnect,Please check those devices.!")
    # finally:
    #     Aub.DisConnect_Aubo(Robot)
if __name__ == '__main__':
    main()