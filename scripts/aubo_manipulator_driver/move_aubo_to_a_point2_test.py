#!/usr/bin/env python
# coding=utf-8

import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import re
from robotcontrol import *

def deg_to_rad(tuplelist):
    dd=[]
    for i in tuplelist:
        dd.append(float(i)*math.pi/180)
    return tuple(dd)

def deg_to_rad2(tuplelist):
    dd=[]
    for i in tuplelist:
        dd.append(float(i))
    return tuple(dd)


def string_to_float(msgs):
    new_msgs = []

    # rospy.loginfo('msgs: %s', msgs)
    filtered_msgs = re.findall(r'\-?\d+\.?\d*', msgs)

    for i in filtered_msgs:
        new_msgs.append(i)
        # print(i)

    return tuple(new_msgs)




class Aubocontrol:
    def __init__(self):
        rospy.init_node('listener', anonymous=True)

        rospy.loginfo("Construc")

        
        # 初始化logger
        # logger_init()

        # 启动测试
        # logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))

        # 系统初始化
        Auboi5Robot.initialize()

        # 创建机械臂控制类
        self.robot = Auboi5Robot()

        # 创建上下文
        handle = self.robot.create_context()

        # 打印上下文
        # logger.info("robot.rshd={0}".format(handle))

        try:
            # 链接服务器
            ip = '192.168.1.11'
            port = 8899
            result = self.robot.connect(ip, port)

            if result != RobotErrorType.RobotError_SUCC:
                print("error")
                # logger.info("connect server{0}:{1} failed.".format(ip, port))

            else:
                # # 重新上电
                # robot.robot_shutdown()
                #
                # # 上电
                self.robot.robot_startup()
                #
                # # 设置碰撞等级
                # robot.set_collision_class(7)

                # 设置工具端电源为１２ｖ
                # robot.set_tool_power_type(RobotToolPowerType.OUT_12V)

                # 设置工具端ＩＯ_0为输出
                self.robot.set_tool_io_type(RobotToolIoAddr.TOOL_DIGITAL_IO_0, RobotToolDigitalIoDir.IO_OUT)

                # 获取工具端ＩＯ_0当前状态
                tool_io_status = self.robot.get_tool_io_status(RobotToolIoName.tool_io_0)
                # logger.info("tool_io_0={0}".format(tool_io_status))

                # 设置工具端ＩＯ_0状态
                self.robot.set_tool_io_status(RobotToolIoName.tool_io_0, 1)

                # 获取控制柜用户DI
                io_config = self.robot.get_board_io_config(RobotIOType.User_DI)

                # 输出DI配置
                # logger.info(io_config)

                # 获取控制柜用户DO
                io_config = self.robot.get_board_io_config(RobotIOType.User_DO)

                # 输出DO配置
                # logger.info(io_config)

                # 当前机械臂是否运行在联机模式
                # logger.info("robot online mode is {0}".format(self.robot.is_online_mode()))

        except RobotError, e:
            print("eeee",e)
            # logger.error("{0} robot Event:{1}".format(self.robot.get_local_time(), e))

        # finally:
        #     # 断开服务器链接
        #     if self.robot.connected:
        #         # 关闭机械臂
        #         self.robot.robot_shutdown()
        #         # 断开机械臂链接
        #         self.robot.disconnect()
        #     # 释放库资源
        #     Auboi5Robot.uninitialize()
        #     logger.info("{0} test completed.".format(Auboi5Robot.get_local_time()))

        

        self.joint_msg_sub = rospy.Subscriber('joint_msgs', String, self.get_joint_states, queue_size=1)

        self.current_joint_pub = rospy.Publisher('current_joints', JointState, queue_size=1)
        self.current_pose_pub = rospy.Publisher('curren_pose', Pose, queue_size=1)
        
        
        rate = rospy.Rate(1) #10Hz
        while not rospy.is_shutdown():
            current_pos = self.robot.get_current_waypoint()
            print("show msg")
            print(current_pos)
            
            current_joint_msg = JointState()
            current_joint_msg.header.frame_id = ""
            current_joint_msg.header.stamp = rospy.Time.now()
            current_joint_msg.position = current_pos['joint']
            current_joint_msg.velocity = []
            current_joint_msg.effort = []

            current_pose_msg = Pose()
            current_pose_msg.position.x = current_pos['pos'][0]
            current_pose_msg.position.y = current_pos['pos'][1]
            current_pose_msg.position.z = current_pos['pos'][2]
            current_pose_msg.orientation.x = current_pos['ori'][0]
            current_pose_msg.orientation.y = current_pos['ori'][1]
            current_pose_msg.orientation.z = current_pos['ori'][2]
            current_pose_msg.orientation.w = current_pos['ori'][3]
           

            self.current_joint_pub.publish(current_joint_msg)
            self.current_pose_pub.publish(current_pose_msg)
            rate.sleep()



    def __del__ (self):
        rospy.loginfo("-----------------------")
        rospy.loginfo("Desctruct !!!!!!!!!!!!!")

        # 断开服务器链接
        self.robot.disconnect()

    def get_joint_states(self, joint_stats_msg):
        rospy.loginfo("-----------------------")
        # rospy.loginfo("robot: %d", self.robot)


        filtered_msgs = string_to_float(joint_stats_msg.data)
        # print("origin_msgs:")
        # print(joint_stats_msg.data)
        print("filtered_msgs:")
        print(filtered_msgs)

        joint_status = self.robot.get_joint_status()
        logger.info("joint_status={0}".format(joint_status))

        # # 初始化全局配置文件
        self.robot.init_profile()


        # 设置关节最大加速度
        # robot.set_joint_maxacc((5.5, 5.5, 5.5, 5.5, 5.5, 5.5))
        #
        # 设置关节最大加速度
        # robot.set_joint_maxvelc((1.5, .5, 2.5, 2.5, 2.5, 2.5))

        # 设置关节最大加速度
        self.robot.set_joint_maxacc((3.5, 3.5, 3.5, 3.5, 3.5, 3.5))

        # 设置关节最大速度
        self.robot.set_joint_maxvelc((2.5, 2.5, 2.5, 2.5, 2.5, 2.5))

        # 设置机械臂末端最大线加速度(m/s)
        self.robot.set_end_max_line_acc(0.5)
        logger.info("-------go-----to-----start-------step--01")

        # 获取机械臂末端最大线加速度(m/s)
        # robot.set_end_max_line_velc(0.2)
        self.robot.set_end_max_line_velc(0.5)



        joint_radian_msgs = deg_to_rad2(filtered_msgs)
        # joint_radian_msgs = float(joint_radian_msgs)
        print("joint_radian_msgs")
        print(joint_radian_msgs)

        
        if 'movej' in joint_stats_msg.data:
            rospy.loginfo("move J")
            joint_radian_values = joint_radian_msgs[0:6]
            print("value:")
            print(joint_radian_values)
            self.robot.move_joint(joint_radian_values)


        elif 'movel' in joint_stats_msg.data:
            rospy.loginfo("move L")
            joint_radian_values = joint_radian_msgs[0:6]
            print("value:")
            print(joint_radian_values)
            self.robot.move_line(joint_radian_values)

        elif 'movet' in joint_stats_msg.data:
            rospy.loginfo("move T")
        else:
            pass
        



def main():
    Aub=Aubocontrol()   
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print(1)
        rate.sleep()


if __name__ == '__main__':
    main()

