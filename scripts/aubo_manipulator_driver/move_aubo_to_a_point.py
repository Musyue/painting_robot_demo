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

            # 循环测试
            while test_count > 0:
                test_count -= 1

                joint_status = robot.get_joint_status()
                logger.info("joint_status={0}".format(joint_status))

                # 初始化全局配置文件
                robot.init_profile()

                # 设置关节最大加速度
                # robot.set_joint_maxacc((5.5, 5.5, 5.5, 5.5, 5.5, 5.5))
                #
                # # 设置关节最大加速度
                # robot.set_joint_maxvelc((1.5, .5, 1.5, 1.5, 1.5, 1.5))
                # 设置关节最大加速度
                robot.set_joint_maxacc((1.5, 1.5, 1.5, 1.5, 1.5, 1.5))

                # 设置关节最大加速度
                robot.set_joint_maxvelc((1.5, 1.5, 1.5, 1.5, 1.5, 1.5))
                # 设置机械臂末端最大线加速度(m/s)
                robot.set_end_max_line_acc(0.5)
                logger.info("-------go-----to-----start-------step--01")
                # 获取机械臂末端最大线加速度(m/s)
                # robot.set_end_max_line_velc(0.2)
                robot.set_end_max_line_velc(0.5)
                
                # joint_radian = deg_to_rad((-3.07,-2.094,80.07,64.0747,95.11,89.98))#((0,0,0,0,0,0))
                # joint_radian=(0.710393681906976, 
                #     -1.3977020494122865, 
                #     1.3012983964751612, 
                #     -0.4425922077023454, 
                #     -0.8604026448879205, 
                #     1.5707963267948966)
            #     dictdata={
            #         "aubo_data_num_0": [
            #         -0.2852509833270265, 
            #         -0.5320376301933496, 
            #         1.3666906155038931, 
            #         -1.2428644078925508, 
            #         -1.856047310121923, 
            #         1.5707963267948966
            #     ], 
            #     "aubo_data_num_1": [
            #         -0.2852509833270265, 
            #         -1.3977020494122865, 
            #         1.3012983964751612, 
            #         -0.4425922077023463, 
            #         -1.856047310121923, 
            #         1.5707963267948966
            #     ], 
            #     "aubo_data_num_10": [
            #         0.710393681906976, 
            #         -0.6376332132456559, 
            #         1.4856621047642653, 
            #         -1.018297335579872, 
            #         -0.8604026448879205, 
            #         1.5707963267948966
            #     ], 
            #     "aubo_data_num_11": [
            #         0.710393681906976, 
            #         -0.5320376301933489, 
            #         1.3666906155038934, 
            #         -1.24286440789255, 
            #         -0.8604026448879205, 
            #         1.5707963267948966
            #     ], 
            #     "aubo_data_num_12": [
            #         -0.2852509833270265, 
            #         -0.5320376301933498, 
            #         1.3666906155038934, 
            #         -1.2428644078925508, 
            #         -1.856047310121923, 
            #         1.5707963267948966
            #     ], 
            #     "aubo_data_num_2": [
            #         0.710393681906976, 
            #         -1.3977020494122865, 
            #         1.3012983964751612, 
            #         -0.4425922077023454, 
            #         -0.8604026448879205, 
            #         1.5707963267948966
            #     ], 
            #     "aubo_data_num_3": [
            #         0.710393681906976, 
            #         -1.1756504138805868, 
            #         1.447317763230986, 
            #         -0.5186244764782195, 
            #         -0.8604026448879205, 
            #         1.5707963267948966
            #     ], 
            #     "aubo_data_num_4": [
            #         -0.2852509833270265, 
            #         -1.1756504138805868, 
            #         1.447317763230986, 
            #         -0.5186244764782204, 
            #         -1.856047310121923, 
            #         1.5707963267948966
            #     ], 
            #     "aubo_data_num_5": [
            #         -0.2852509833270265, 
            #         -0.9698667703466137, 
            #         1.525512678114401, 
            #         -0.6462132051287792, 
            #         -1.856047310121923, 
            #         1.5707963267948966
            #     ], 
            #     "aubo_data_num_6": [
            #         0.710393681906976, 
            #         -0.9698667703466137, 
            #         1.525512678114401, 
            #         -0.6462132051287783, 
            #         -0.8604026448879205, 
            #         1.5707963267948966
            #     ], 
            #     "aubo_data_num_7": [
            #         0.710393681906976, 
            #         -0.787040253980912, 
            #         1.53823360029741, 
            #         -0.8163187993114711, 
            #         -0.8604026448879205, 
            #         1.5707963267948966
            #     ], 
            #     "aubo_data_num_8": [
            #         -0.2852509833270265, 
            #         -0.787040253980912, 
            #         1.53823360029741, 
            #         -0.8163187993114711, 
            #         -1.856047310121923, 
            #         1.5707963267948966
            #     ], 
            #     "aubo_data_num_9": [
            #         -0.2852509833270265, 
            #         -0.6376332132456559, 
            #         1.4856621047642644, 
            #         -1.0182973355798728, 
            #         -1.856047310121923, 
            #         1.5707963267948966
            #     ]
            # }

                # robot.set_blend_radius(0.01)
                # for i in range(len(dictdata)):
                #     joint_radian=tuple(dictdata["aubo_data_num_"+str(i)])
                #     logger.info("move joint to {0}".format(joint_radian))
                    # robot.add_waypoint(joint_radian)
                    # robot.move_joint(joint_radian)
                    # robot.move_line(joint_radian)
                # robot.remove_all_waypoint()
                # robot.move_joint((0,0,0,0,0,0))
                print("robot.get_joint_maxacc()",robot.get_joint_maxacc())
                print("robot.get_joint_maxvelc()",robot.get_joint_maxvelc())
                print("robot.get_end_max_line_acc()",robot.get_end_max_line_acc())
                print("robot.get_end_max_linevelc()",robot.get_end_max_line_velc())
                print("robot.get_end_max_angle_acc()",robot.get_end_max_angle_acc())
                print("robot.get_end_max_angle_velc()",robot.get_end_max_angle_velc())
                print("robot.get_joint_status()",robot.get_joint_status())
                print("get_tool_dynamics_param",robot.get_tool_dynamics_param())
                print("get_dynidentify_results",robot.get_dynidentify_results())

                print("get_current_waypoint",robot.get_current_waypoint())
                print_json(robot.get_current_waypoint())
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
    main(1)
    # print deg_to_rad((-3.3364,12.406,-81.09,-91.207,-86.08,0.164))