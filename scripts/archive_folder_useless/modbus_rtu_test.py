#!/usr/bin/env python
# -*- coding: utf_8 -*-
"""
id：1---->stand bar
id:2----->roation
id:3------>Upper and lower climbing pole
"""
import rospy
import serial
import time
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
import readchar
from math import *
import yaml
class Control3DOFROBOT():
    def __init__(self,PORT,BaudRate,Nodename):
        self.PORT=PORT
        self.BaudRate=BaudRate
        self.nodename=Nodename
        self.Openmodbus_ok_flag=0
        self.climb_close_loop_inital()
        self.configname="3dof_robot.yaml"
        self.canyamlconfig_path=0
    def climb_close_loop_inital(self,):
        self.Kp_climb = 0.1#rospy.get_param("climb_kp")
        self.Ki_climb = 0.1#rospy.get_param("climb_ki")
        self.Kd_climb = 0.1#rospy.get_param("climb_kd")
        self.current_time_climb=0
        self.last_time_climb=0
        self.last_error_climb=0
        self.sample_time_climb=0.00
        self.PTerm_climb=0
        self.ITerm_climb=0
        self.DTerm_climb=0
        self.int_error = 0.0
        self.windup_guard_climb = 0.1
        self.output_climb = 0.0
    def Opreating_Yaml(self):

        yaml_path = self.canyamlconfig_path+self.configname
        file_data = open(yaml_path)
        self.yamlDic = yaml.load(file_data)
        file_data.close()
    def Init_node(self):
        rospy.init_node(self.nodename)
    def Connect_3DOF_MODbus_RTU(self):
        try:
            # Connect to the slave
            master = modbus_rtu.RtuMaster(
                serial.Serial(port=self.PORT, baudrate=self.BaudRate, bytesize=8, parity='O', stopbits=1, xonxoff=0)
            )
            master.set_timeout(5.0)
            master.set_verbose(True)
            rospy.loginfo("connected")
            return master
        except: #modbus_tk.modbus.ModbusError as exc:
            #rospy.logerr("%s- Code=%d", exc, exc.get_exception_code())
            self.Openmodbus_ok_flag=1
            return False


    def Control_3DOF_Robot(self, master, control_id, velocity, outputPulse):  # position control
        """

        :param master:
        :param control_id: 1-stand,2-rotation,3-climber
        :param velocity: 0-2500
        :param outputPulse: High 32位
        :return:
        """
        rospy.loginfo(master.execute(control_id, cst.READ_HOLDING_REGISTERS, 0, 8))
        # print type(master.execute(4, cst.READ_HOLDING_REGISTERS, 0, 8))
        master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 1, output_value=6) #
        #rospy.loginfo(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 282, output_value=1))  # enable Climb Driver
        master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 290,
                                   output_value=outputPulse)  # High 16 10000 pulse 1 rpm,negtive up,positive up
        #rospy.loginfo(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 291, output_value=outputPulse))  # Low 16bit
        master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 97, output_value=velocity)  # internal velocity
        # rospy.loginfo(master.execute(4, cst.WRITE_SINGLE_REGISTER, 113, output_value=1000))  # internal velocity
        # rospy.loginfo(master.execute(4, cst.WRITE_SINGLE_REGISTER, 114, output_value=1000))  # internal velocity
        master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 324, output_value=1000)  # set fixed velocity
        #
        master.execute(control_id, cst.READ_HOLDING_REGISTERS, 212, 1)
        master.execute(control_id, cst.READ_HOLDING_REGISTERS, 214, 1)
        master.execute(control_id, cst.READ_HOLDING_REGISTERS, 218, 1)
        master.execute(control_id, cst.READ_HOLDING_REGISTERS, 220, 1)

        master.execute(control_id, cst.READ_HOLDING_REGISTERS, 212, 12)

    def Control_3DOF_Robot_Speed_Mode(self, master, control_id, velocity):  # velocity control
        """

        :param master:
        :param control_id: 1-stand,2-rotation,3-climber
        :param velocity: 0-2500
        :param outputPulse: High 32位
        :return:
        """
        rospy.loginfo(master.execute(control_id, cst.READ_HOLDING_REGISTERS, 0, 8))
        # print type(master.execute(4, cst.READ_HOLDING_REGISTERS, 0, 8))
        kk=master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 1, output_value=7) #
        rospy.loginfo(kk)
        #rospy.loginfo(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 282, output_value=1))  # enable Climb Driver
        master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 324,
                                   output_value=velocity)  # High 16 10000 pulse 1 rpm,negtive up,positive up
        #rospy.loginfo(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 291, output_value=outputPulse))  # Low 16bit
        # master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 97, output_value=velocity)  # internal velocity
        rospy.loginfo(master.execute(4, cst.WRITE_SINGLE_REGISTER, 113, output_value=1000))  # internal velocity
        rospy.loginfo(master.execute(4, cst.WRITE_SINGLE_REGISTER, 114, output_value=1000))  # internal velocity
        # master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 324, output_value=1000)  # set fixed velocity
        #
        master.execute(control_id, cst.READ_HOLDING_REGISTERS, 212, 1)
        master.execute(control_id, cst.READ_HOLDING_REGISTERS, 214, 1)
        master.execute(control_id, cst.READ_HOLDING_REGISTERS, 218, 1)
        master.execute(control_id, cst.READ_HOLDING_REGISTERS, 220, 1)

        master.execute(control_id, cst.READ_HOLDING_REGISTERS, 212, 12)


    def Holding_Robot(self, master, velocity, outputDistance, control_id=1):  # position control
        """

        :param master:
        :param velocity:
        :param outputPulse:Distance unit:m pos up,neg Down
        :param control_id:
        :return:
        """
        outputPulse = outputDistance*444.4
        self.Control_3DOF_Robot(master, control_id, velocity, -1.0*outputPulse)


    def Rotation_Robot(self, master, velocity, outputDegree, control_id=2):  # position control
        """

        :param master:
        :param velocity: 0-2500
        :param outputDegree: 0-360Degree,Positive disclockwise,Negtive clockwise
        :param control_id:
        :return:
        """
        rospy.loginfo("0-360 Degree,Positive disclockwise,Negtive clockwise")
        
        outputPulse = outputDegree*9.17
        self.Control_3DOF_Robot(master, control_id, velocity, -1.0*outputPulse)


    def Climbing_Robot(self, master, velocity, outputDistance, control_id=3):  # position control
        """

        :param master:
        :param velocity: 0-2500
        :param outputDistance: 0-3m
        :param control_id:
        :return:
        """

        rospy.loginfo("------climb robot,neg down,pos up")
        outputPulse = outputDistance *42.5
        self.Control_3DOF_Robot(master, control_id, velocity, -1.0*outputPulse)
    def Climbing_Robot_close_loop_control(self, master, velocity, DesireDistance, feedback_distance,control_id=3):  # position control
        """
        Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        :param master:
        :param velocity: 0-2500
        :param DesireDistance: 0-3m
        :param control_id:
        :return:
        """
        Distance_error=DesireDistance-feedback_distance
        rospy.loginfo("------climb robot,neg down,pos up")
        self.current_time_climb=time.time()
        deltatime=self.current_time_climb-self.last_time_climb

        # outputPulse = DesireDistance *42.5
        
        deltaerror=Distance_error-self.last_error_climb
        if(deltatime>=self.sample_time_climb):
            self.PTerm_climb=self.Kp_climb*Distance_error
            self.ITerm_climb+=Distance_error*deltatime
            if (self.ITerm_climb < -self.windup_guard_climb):
                self.ITerm_climb = -self.windup_guard_climb
            elif (self.ITerm_climb > self.windup_guard_climb):
                self.ITerm_climb = self.windup_guard_climb
            self.DTerm_climb=0.0
            if deltatime>0:
                self.DTerm_climb=deltaerror/deltatime
            self.last_time_climb=self.current_time_climb
            self.last_error_climb=Distance_error
            self.output_climb=self.PTerm_climb + (self.Ki_climb *self.ITerm_climb) + (self.Kd_climb * self.DTerm_climb)
            outputPulse = self.output_climb *42.5
            self.Control_3DOF_Robot(master, control_id, velocity, -1.0*outputPulse)

    def Climbing_Robot_close_loop_velocity_control(self, master, velocity, DesireDistance, feedback_distance,control_id=3):  # position control
        """
        Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        :param master:
        :param velocity: 0-2500
        :param DesireDistance: 0-3m
        :param control_id:
        :return:
        """
        Distance_error=DesireDistance-feedback_distance
        rospy.loginfo("------climb robot,neg down,pos up")
        self.current_time_climb=time.time()
        deltatime=self.current_time_climb-self.last_time_climb

        # outputPulse = DesireDistance *42.5
        
        deltaerror=Distance_error-self.last_error_climb
        if(deltatime>=self.sample_time_climb):
            self.PTerm_climb=self.Kp_climb*Distance_error
            self.ITerm_climb+=Distance_error*deltatime

            if (self.ITerm_climb < -self.windup_guard_climb):
                self.ITerm_climb = -self.windup_guard_climb
            elif (self.ITerm_climb > self.windup_guard_climb):
                self.ITerm_climb = self.windup_guard_climb

            self.DTerm_climb=0.0
            if deltatime>0:
                self.DTerm_climb=deltaerror/deltatime
            self.last_time_climb=self.current_time_climb
            self.last_error_climb=Distance_error
            self.output_climb=self.PTerm_climb + (self.Ki_climb *self.ITerm_climb) + (self.Kd_climb * self.DTerm_climb)
            outputvelocity = self.output_climb *42.5
            self.Control_3DOF_Robot_Speed_Mode(master, control_id, outputvelocity)


    def Read_3DOF_Controller_Buffe(self, master):
        """

        :param master:
        :return:
        """
        rospy.loginfo("Driver Warnning nums Meaning Table:")
        rospy.loginfo("0: No Warnning")
        rospy.loginfo("3: Over Flow")
        rospy.loginfo("4: Over heat")
        rospy.loginfo("6: Encoder Warnning")
        rospy.loginfo("13: EEPROM WRITING&READING Unusal")
        rospy.loginfo("8: Over Load")
        rospy.loginfo("11: Over speed")
        rospy.loginfo("2: Over Voltage")
        rospy.loginfo("1: Lack Voltage")
        rospy.loginfo("9: Position Error Large")
        rospy.loginfo(master.execute(1, cst.READ_HOLDING_REGISTERS, 212, 2))
        rospy.loginfo("Holding Robot driver warnning nums")
        rospy.loginfo(master.execute(1, cst.READ_HOLDING_REGISTERS, 202, 2))
        rospy.loginfo("Rotation Robot command position counts")
        rospy.loginfo(master.execute(2, cst.READ_HOLDING_REGISTERS, 212, 2))
        rospy.loginfo("Rotation Robot driver warnning nums")
        rospy.loginfo(master.execute(2, cst.READ_HOLDING_REGISTERS, 202, 2))
        rospy.loginfo("Climbing Robot command position counts")
        rospy.loginfo(master.execute(3, cst.READ_HOLDING_REGISTERS, 212, 2))
        rospy.loginfo("Climbing Robot driver warnning nums")
        rospy.loginfo(master.execute(3, cst.READ_HOLDING_REGISTERS, 202, 2))


    def Emergency_Stop_All(self, master):
            rospy.loginfo(master.execute(1, cst.WRITE_SINGLE_REGISTER, 282, output_value=0))
            rospy.loginfo(master.execute(2, cst.WRITE_SINGLE_REGISTER, 282, output_value=0))
            rospy.loginfo(master.execute(3, cst.WRITE_SINGLE_REGISTER, 282, output_value=0))
    def Open_Stop_Enable_Driver(self, master, control_id,stop_open_flag):
        master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 282, output_value=stop_open_flag)  # enable
        return control_id,stop_open_flag
def main():
    nodename="climb_3dof_node"
    climb_port = rospy.get_param("/renov_up_level/climb_port")
    rospy.loginfo("%s is %s", rospy.resolve_name('climb_port'), climb_port)

    climb_port_baudrate = rospy.get_param("/renov_up_level/climb_port_baudrate")
    rospy.loginfo("%s is %s", rospy.resolve_name('climb_port_baudrate'), climb_port_baudrate)


    c3dof=Control3DOFROBOT(climb_port,climb_port_baudrate,nodename)
    # open_climb_flag=0
    # open_rotation_flag=0
    # open_hold_flag=0
    Master=c3dof.Connect_3DOF_MODbus_RTU()
    c3dof.Init_node()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        c3dof.Control_3DOF_Robot_Speed_Mode(Master,3,100)
        rate.sleep()
    c3dof.Control_3DOF_Robot_Speed_Mode(Master,3,0)
if __name__=="__main__":
    main()
