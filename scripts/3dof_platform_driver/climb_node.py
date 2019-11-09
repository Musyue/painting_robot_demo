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


class Control3DOFROBOT():
    def __init__(self,PORT,BaudRate,Nodename):
        self.PORT=PORT
        self.BaudRate=BaudRate
        self.nodename=Nodename
        self.Openmodbus_ok_flag=0
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
        rospy.loginfo(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 1, output_value=6)) #
        #rospy.loginfo(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 282, output_value=1))  # enable Climb Driver
        rospy.loginfo(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 290,
                                   output_value=outputPulse))  # High 16 10000 pulse 1 rpm,negtive up,positive up
        #rospy.loginfo(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 291, output_value=outputPulse))  # Low 16bit
        rospy.loginfo(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 97, output_value=velocity))  # internal velocity
        # rospy.loginfo(master.execute(4, cst.WRITE_SINGLE_REGISTER, 113, output_value=1000))  # internal velocity
        # rospy.loginfo(master.execute(4, cst.WRITE_SINGLE_REGISTER, 114, output_value=1000))  # internal velocity
        rospy.loginfo(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 324, output_value=1000))  # set fixed velocity
        #
        rospy.loginfo(master.execute(control_id, cst.READ_HOLDING_REGISTERS, 212, 1))
        rospy.loginfo(master.execute(control_id, cst.READ_HOLDING_REGISTERS, 214, 1))
        rospy.loginfo(master.execute(control_id, cst.READ_HOLDING_REGISTERS, 218, 1))
        rospy.loginfo(master.execute(control_id, cst.READ_HOLDING_REGISTERS, 220, 1))

        rospy.loginfo(master.execute(control_id, cst.READ_HOLDING_REGISTERS, 212, 12))


    def Holding_Robot(self, master, velocity, outputDistance, control_id=1):  # position control
        """

        :param master:
        :param velocity:
        :param outputPulse:5.5cm -20 Negtive up,Positive Down
        :param control_id:
        :return:
        """
        outputPulse = outputDistance/3.6
        self.Control_3DOF_Robot(master, control_id, velocity, outputPulse)


    def Rotation_Robot(self, master, velocity, outputDegree, control_id=2):  # position control
        """

        :param master:
        :param velocity: 0-2500
        :param outputDegree: 0-360Degree,Positive clockwise,Negtive disclockwise
        :param control_id:
        :return:
        """
        rospy.loginfo("outputDegree: 0-360 Degree,Positive clockwise,Negtive disclockwise")
        outputPulse = outputDegree / 6.5
        self.Control_3DOF_Robot(master, control_id, velocity, outputPulse)


    def Climbing_Robot(self, master, velocity, outputDistance, control_id=3):  # position control
        """

        :param master:
        :param velocity: 0-2500
        :param outputDistance: 0-300cm
        :param control_id:
        :return:
        """

        rospy.loginfo("outputDegree: 0-360 Degree,Positive down,Negtive up")
        outputPulse = outputDistance /5.6
        self.Control_3DOF_Robot(master, control_id, velocity, outputPulse)


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
        rospy.loginfo(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 282, output_value=stop_open_flag))  # enable
        return control_id,stop_open_flag
def main():
    nodename="climb_3dof_node"
    climb_port = rospy.get_param("climb_port")
    rospy.loginfo("%s is %s", rospy.resolve_name('climb_port'), climb_port)

    climb_port_baudrate = rospy.get_param("climb_port_baudrate")
    rospy.loginfo("%s is %s", rospy.resolve_name('climb_port_baudrate'), climb_port_baudrate)


    c3dof=Control3DOFROBOT(climb_port,climb_port_baudrate,nodename)
    open_climb_flag=0
    open_rotation_flag=0
    open_hold_flag=0
    Master=c3dof.Connect_3DOF_MODbus_RTU()
    c3dof.Init_node()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        climb_port_ok_flag = rospy.get_param("climb_port_ok_flag")
        rospy.loginfo("%s is %s", rospy.resolve_name('climb_port_ok_flag'), climb_port_ok_flag)

        close_all_3dof_climb_driver_flag = rospy.get_param("close_all_3dof_climb_driver_flag")
        rospy.loginfo("%s is %s", rospy.resolve_name('close_all_3dof_climb_driver_flag'), close_all_3dof_climb_driver_flag)

        enable_control_stand_bar = rospy.get_param("enable_control_stand_bar")
        rospy.loginfo("%s is %s", rospy.resolve_name('enable_control_stand_bar'), enable_control_stand_bar)

        enable_control_rotation = rospy.get_param("enable_control_rotation")
        rospy.loginfo("%s is %s", rospy.resolve_name('enable_control_rotation'), enable_control_rotation)

        enable_climb_control = rospy.get_param("enable_climb_control")
        rospy.loginfo("%s is %s", rospy.resolve_name('enable_climb_control'), enable_climb_control)

        velocity_control_stand_bar = rospy.get_param("velocity_control_stand_bar")
        rospy.loginfo("%s is %s", rospy.resolve_name('velocity_control_stand_bar'), velocity_control_stand_bar)

        velocity_control_rotation = rospy.get_param("velocity_control_rotation")
        rospy.loginfo("%s is %s", rospy.resolve_name('velocity_control_rotation'), velocity_control_rotation)

        velocity_climb_control = rospy.get_param("velocity_climb_control")
        rospy.loginfo("%s is %s", rospy.resolve_name('velocity_climb_control'), velocity_climb_control)

        distance_control_stand_bar = rospy.get_param("distance_control_stand_bar")
        rospy.loginfo("%s is %s", rospy.resolve_name('distance_control_stand_bar'), distance_control_stand_bar)

        rad_control_rotation = rospy.get_param("rad_control_rotation")
        rospy.loginfo("%s is %s", rospy.resolve_name('rad_control_rotation'), rad_control_rotation)
        
        distance_climb_control = rospy.get_param("distance_climb_control")
        rospy.loginfo("%s is %s", rospy.resolve_name('distance_climb_control'), distance_climb_control)

        if c3dof.Openmodbus_ok_flag!=1 and climb_port_ok_flag==1:

            if close_all_3dof_climb_driver_flag==1:
                c3dof.Emergency_Stop_All(Master)
                open_hold_flag=0
                open_rotation_flag=0
                open_rotation_flag=0

            if enable_control_stand_bar==1:
                c3dof.Open_Stop_Enable_Driver(Master,1,1)
                rospy.set_param('enable_control_stand_bar',0)
                open_hold_flag=1
            if enable_control_stand_bar==2:
                c3dof.Open_Stop_Enable_Driver(Master,1,0)
                rospy.set_param('enable_control_stand_bar',0)
                open_hold_flag=0

            if enable_control_rotation==1:
                c3dof.Open_Stop_Enable_Driver(Master,2,1)
                rospy.set_param('enable_control_rotation',0)
                open_rotation_flag=1
                
            if enable_control_rotation==2:
                c3dof.Open_Stop_Enable_Driver(Master,2,0)
                rospy.set_param('enable_control_rotation',0)
                open_rotation_flag=0
            if enable_climb_control==1:
                c3dof.Open_Stop_Enable_Driver(Master,3,1)
                rospy.set_param('enable_climb_control',0)
                open_climb_flag=1
            if enable_climb_control==2:
                c3dof.Open_Stop_Enable_Driver(Master,3,0)
                rospy.set_param('enable_climb_control',0)
                open_climb_flag=0
            #set velocity
            if open_hold_flag==1:
                c3dof.Holding_Robot(Master,velocity_control_stand_bar,distance_control_stand_bar)
                open_hold_flag=0
            if open_climb_flag==1:
                c3dof.Climbing_Robot(Master,velocity_climb_control,distance_climb_control)
                open_climb_flag=0
            if open_rotation_flag==1:
                c3dof.Rotation_Robot(Master,velocity_control_rotation,rad_control_rotation) 
                open_rotation_flag=0       
        else:
            Master=c3dof.Connect_3DOF_MODbus_RTU()
            time.sleep(3)
            rospy.logerr("Open climb modbus error---I will reconnect after 3 seconds----")
        rate.sleep()

if __name__=="__main__":
    main()
