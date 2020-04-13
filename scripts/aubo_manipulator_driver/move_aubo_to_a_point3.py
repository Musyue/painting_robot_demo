#! /usr/bin/env python
# coding=utf-8
import math
from robotcontrol import *
import json
def deg_to_rad(tuplelist):
    dd=[]
    for i in tuplelist:
        dd.append(i*math.pi/180)
    return tuple(dd)
def print_json(data):
    print(json.dumps(data, sort_keys=True, indent=4, separators=(', ', ': '), ensure_ascii=False))
    
def main(test_count):
    # 初始化logger
    logger_init()

    # 启动测试
    logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))

    # 系统初始化
    Auboi5Robot.initialize()

    # 创建机械臂控制类
    robot = Auboi5Robot()

    # 创建上下文
    handle = robot.create_context()

    # 打印上下文
    logger.info("robot.rshd={0}".format(handle))
    try:

        # 链接服务器
        ip = '192.168.1.11'
        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            # # 重新上电
            # robot.robot_shutdown()
            #
            # # 上电
            robot.robot_startup()
            #
            # # 设置碰撞等级
            # robot.set_collision_class(7)

            # 设置工具端电源为１２ｖ
            # robot.set_tool_power_type(RobotToolPowerType.OUT_12V)

            # 设置工具端ＩＯ_0为输出
            robot.set_tool_io_type(RobotToolIoAddr.TOOL_DIGITAL_IO_0, RobotToolDigitalIoDir.IO_OUT)

            # 获取工具端ＩＯ_0当前状态
            tool_io_status = robot.get_tool_io_status(RobotToolIoName.tool_io_0)
            # logger.info("tool_io_0={0}".format(tool_io_status))

            # 设置工具端ＩＯ_0状态
            robot.set_tool_io_status(RobotToolIoName.tool_io_0, 1)

            # 获取控制柜用户DI
            io_config = robot.get_board_io_config(RobotIOType.User_DI)

            # 输出DI配置
            logger.info(io_config)

            # 获取控制柜用户DO
            io_config = robot.get_board_io_config(RobotIOType.User_DO)

            # 输出DO配置
            logger.info(io_config)

            # 当前机械臂是否运行在联机模式
            logger.info("robot online mode is {0}".format(robot.is_online_mode()))

            joint_status = robot.get_joint_status()
            logger.info("joint_status={0}".format(joint_status))

            # 初始化全局配置文件
            robot.init_profile()
            # 设置关节最大加速度
            # robot.set_joint_maxacc((5.5, 5.5, 5.5, 5.5, 5.5, 5.5))
            # # 设置关节最大加速度
            # robot.set_joint_maxvelc((1.5, .5, 1.5, 1.5, 1.5, 1.5))
            # 设置关节最大加速度
            robot.set_joint_maxacc((1.5, 1.5, 1.5, 1.5, 1.5, 1.5))
            # 设置关节最大加速度
            robot.set_joint_maxvelc((1.5, 1.5, 1.5, 1.5, 1.5, 1.5))
            # 设置机械臂末端最大线加速度(m/s)
            robot.set_end_max_line_acc(0.3)
            logger.info("-------go-----to-----start-------step--01")
            # 获取机械臂末端最大线加速度(m/s)
            # robot.set_end_max_line_velc(0.2)
            robot.set_end_max_line_velc(0.1)                
            joint_radian=(-0.28525098, -0.53203763,  1.36669062, -1.24286441, -1.85604731, 1.57079633)
            robot.move_joint(joint_radian)
            
            # 循环测试
            while test_count > 0:
                test_count -= 1
                robot.remove_all_waypoint()
                
                joint_radian=(-0.28525098, -0.53203763,  1.36669062, -1.24286441, -1.85604731, 1.57079633)
                flag1=robot.add_waypoint(joint_radian)
                print('point1:',flag1)

                joint_radian=(0.71039368, -0.53203763,  1.36669062, -1.24286441, -0.86040264,1.5707963)
                flag2=robot.add_waypoint(joint_radian)
                print('point2:',flag2)

                joint_radian=(0.71039368, -0.63763321,  1.4856621 , -1.01829734, -0.86040264,1.57079633)
                flag3=robot.add_waypoint(joint_radian)
                print('point3:',flag3)

                joint_radian=(-0.28525098, -0.63763321,  1.4856621 , -1.01829734, -1.85604731,1.57079633)
                flag4=robot.add_waypoint(joint_radian)
                print("point4:",flag4)

                joint_radian=(-0.28525098, -0.78704025,  1.5382336 , -0.8163188 , -1.85604731, 1.57079633)
                flag5=robot.add_waypoint(joint_radian)
                print("point5:",flag5)

                joint_radian=(0.71039368, -0.78704025,  1.5382336 , -0.8163188 , -0.86040264,1.57079633)
                flag5=robot.add_waypoint(joint_radian)

                joint_radian=(0.71039368, -0.96986677,  1.52551268, -0.64621321, -0.86040264, 1.57079633)
                flag5=robot.add_waypoint(joint_radian)
                
                joint_radian=(-0.28525098, -0.96986677,  1.52551268, -0.64621321, -1.85604731, 1.57079633)
                flag5=robot.add_waypoint(joint_radian)

                joint_radian=(-0.28525098, -1.17565041,  1.44731776, -0.51862448, -1.85604731, 1.57079633)
                flag5=robot.add_waypoint(joint_radian)

                joint_radian=(0.71039368, -1.17565041,  1.44731776, -0.51862448, -0.86040264,1.57079633)
                flag5=robot.add_waypoint(joint_radian)

                joint_radian=(0.71039368, -1.39770205,  1.3012984 , -0.44259221, -0.86040264,1.57079633)
                flag5=robot.add_waypoint(joint_radian)

                joint_radian=(-0.28525098, -1.39770205,  1.3012984 , -0.44259221, -1.85604731,1.57079633)
                flag5=robot.add_waypoint(joint_radian)

                set_flag=robot.set_blend_radius(blend_radius=0.05)
                print("set_flag=:",set_flag)

                set_flag1=robot.move_track(RobotMoveTrackType.CARTESIAN_MOVEP)
                print("set_flag1=:",set_flag1)
                
                # print("robot.get_joint_maxacc()",robot.get_joint_maxacc())
                # print("robot.get_joint_maxvelc()",robot.get_joint_maxvelc())
                # print("robot.get_end_max_line_acc()",robot.get_end_max_line_acc())
                # print("robot.get_end_max_linevelc()",robot.get_end_max_line_velc())
                # print("robot.get_end_max_angle_acc()",robot.get_end_max_angle_acc())
                # print("robot.get_end_max_angle_velc()",robot.get_end_max_angle_velc())
                # print("robot.get_joint_status()",robot.get_joint_status())
                # print("get_tool_dynamics_param",robot.get_tool_dynamics_param())
                # print("get_dynidentify_results",robot.get_dynidentify_results())
                # print("get_current_waypoint",robot.get_current_waypoint())
                # print_json(robot.get_current_waypoint())

            # 断开服务器链接
            robot.disconnect()

    except RobotError, e:
        logger.error("{0} robot Event:{1}".format(robot.get_local_time(), e))

    finally:
        # 断开服务器链接
        if robot.connected:
            # 关闭机械臂
            robot.robot_shutdown()
            # 断开机械臂链接
            robot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()
        logger.info("{0} test completed.".format(Auboi5Robot.get_local_time()))

if __name__=="__main__":
    main(10)
    # print deg_to_rad((-3.3364,12.406,-81.09,-91.207,-86.08,0.164))