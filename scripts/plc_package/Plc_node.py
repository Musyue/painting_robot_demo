
#!/usr/bin/env python
# -*- coding: utf_8 -*-
import rospy
import sys
import binascii
import inspect
import serial
import commands
import time
from src.painting_robot_demo.scripts.plc_package.CRC_16_Check import *
from src.painting_robot_demo.scripts.plc_package.plc_command import *
from textwrap import wrap
from std_msgs.msg import String,UInt64MultiArray

class PLCPKG:
    def __init__(self,nodename):
        self.readstringlength=25

        self.crc16=RobotCRC16()
        self.nodename=nodename
        self.plccmd=PLCDriverCommands()
        self.ReadInfopublish=rospy.Publisher("/read_plc_register", String, queue_size=10)

    def Init_node(self):
        rospy.init_node(self.nodename)

    def Send_message_to_port(self,ser,message):
        """
        :param message: String "010600000001480A" or "01 06 00 00 00 01 48 0A"
        :return: int list
        """
        message_bytes = message.replace(" ",'').decode('hex')
        # print str(hex(message_bytes))
        ser.write(message_bytes)
        ser.flushInput()
        ser.flushOutput()
        time.sleep(0.01)
        strt = ser.read(self.readstringlength).encode('hex')
        readbuffer=[int(int(i, 16)) for i in wrap(strt, 2)]
        return readbuffer,strt
    def Get_crc_16_str(self,cmd):
        return self.crc16.Combining_CRC_and_info(cmd)

def main():
    plcpkg=PLCPKG("plc_pkg_node")
    open_serial_port_again_flag=0
    plcpkg.Init_node()

    plc_port = rospy.get_param("plc_port")
    rospy.loginfo("%s is %s", rospy.resolve_name('plc_port'), plc_port)

    # fetch the utterance parameter from our parent namespace
    plc_port_baudrate = rospy.get_param('plc_port_baudrate')
    rospy.loginfo("%s is %s", rospy.resolve_name('plc_port_baudrate'), plc_port_baudrate)


    try:
        ser = serial.Serial(port=plc_port, baudrate=plc_port_baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=1,timeout=0.3, xonxoff=0,rtscts=False,dsrdtr=False)
    except:
        rospy.logerr("Please check PLC Usb port----")
        open_serial_port_again_flag=1

    count=0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        plc_port_ok_flag = rospy.get_param("plc_port_ok_flag")
        rospy.loginfo("%s is %s", rospy.resolve_name('plc_port_ok_flag'), plc_port_ok_flag)

        read_line_encode = rospy.get_param("read_line_encode")
        rospy.loginfo("%s is %s", rospy.resolve_name('read_line_encode'), read_line_encode)

        # fetch the utterance parameter from our parent namespace
        read_limit_switch_status = rospy.get_param('read_limit_switch_status')
        rospy.loginfo("%s is %s", rospy.resolve_name('read_limit_switch_status'), read_limit_switch_status)

        read_echos_status = rospy.get_param("read_echos_status")
        rospy.loginfo("%s is %s", rospy.resolve_name('read_echos_status'), read_echos_status)


        write_front_light_open_forever = rospy.get_param("write_front_light_open_forever")
        rospy.loginfo("%s is %s", rospy.resolve_name('write_front_light_open_forever'), write_front_light_open_forever)

        # fetch the utterance parameter from our parent namespace
        write_front_light_fast_blink = rospy.get_param('write_front_light_fast_blink')
        rospy.loginfo("%s is %s", rospy.resolve_name('write_front_light_fast_blink'), write_front_light_fast_blink)

        write_front_light_slow_blink = rospy.get_param("write_front_light_slow_blink")
        rospy.loginfo("%s is %s", rospy.resolve_name('write_front_light_slow_blink'), write_front_light_slow_blink)



        write_front_lingth_close = rospy.get_param("write_front_lingth_close")
        rospy.loginfo("%s is %s", rospy.resolve_name('write_front_lingth_close'), write_front_lingth_close)


        write_flex_pole_motor_close = rospy.get_param("write_flex_pole_motor_close")
        rospy.loginfo("%s is %s", rospy.resolve_name('write_flex_pole_motor_close'), write_flex_pole_motor_close)

        # fetch the utterance parameter from our parent namespace
        write_flex_pole_motor_up = rospy.get_param('write_flex_pole_motor_up')
        rospy.loginfo("%s is %s", rospy.resolve_name('write_flex_pole_motor_up'), write_flex_pole_motor_up)

        write_flex_pole_motor_down = rospy.get_param("write_flex_pole_motor_down")
        rospy.loginfo("%s is %s", rospy.resolve_name('write_flex_pole_motor_down'), write_flex_pole_motor_down)


        write_mobile_platform_brake_close = rospy.get_param("write_mobile_platform_brake_close")
        rospy.loginfo("%s is %s", rospy.resolve_name('write_mobile_platform_brake_close'), write_mobile_platform_brake_close)

        # fetch the utterance parameter from our parent namespace
        write_mobile_platform_brake_open = rospy.get_param('write_mobile_platform_brake_open')
        rospy.loginfo("%s is %s", rospy.resolve_name('write_mobile_platform_brake_open'), write_mobile_platform_brake_open)

        write_electric_switch_painting_close = rospy.get_param("write_electric_switch_painting_close")
        rospy.loginfo("%s is %s", rospy.resolve_name('write_electric_switch_painting_close'), write_electric_switch_painting_close)

        write_electric_switch_painting_open = rospy.get_param("write_electric_switch_painting_open")
        rospy.loginfo("%s is %s", rospy.resolve_name('write_electric_switch_painting_open'), write_electric_switch_painting_open)


        if open_serial_port_again_flag!=1 and plc_port_ok_flag==1:
            # read line encode data
            read_line_encode_data,line_encode_str=plcpkg.Send_message_to_port(ser,plcpkg.crc16.Combining_CRC_and_info(plcpkg.plccmd.READ_LINE_ENCODE))
            if len(read_line_encode_data)!=0:
                rospy.set_param('read_line_encode', read_line_encode_data)
            # read  read_limit_switch_status
            read_limit_switch_status_data,read_limit_switch_status_str=plcpkg.Send_message_to_port(ser,plcpkg.crc16.Combining_CRC_and_info(plcpkg.plccmd.READ_LIMIT_SWITCH_STATUS))
            if len(read_limit_switch_status_data)!=0:
                rospy.set_param('read_limit_switch_status', read_limit_switch_status_data)
                rospy.set_param('top_limit_switch_status', read_limit_switch_status_data[2])#here wait for affirm
                rospy.set_param('mid_limit_switch_status', read_limit_switch_status_data[1])
                rospy.set_param('bottom_limit_switch_status', read_limit_switch_status_data[0])
            
            # read line encode data
            read_echos_status_data,read_echos_status_str=plcpkg.Send_message_to_port(ser,plcpkg.crc16.Combining_CRC_and_info(plcpkg.plccmd.READ_ECHOS_STATUS))
            if len(read_echos_status_data)!=0:
                rospy.set_param('read_echos_status', read_echos_status_data)
            
            # write_front_light_open_forever
            #light
            if write_front_light_open_forever==1:
                plcpkg.Send_message_to_port(ser,plcpkg.crc16.Combining_CRC_and_info(plcpkg.plccmd.WRITE_FRONT_LIGHT_OPEN_FOREVER))
            if write_front_light_fast_blink==2:
                plcpkg.Send_message_to_port(ser,plcpkg.crc16.Combining_CRC_and_info(plcpkg.plccmd.WRITE_FRONT_LIGHT_FAST_BLINK))
            if write_front_light_slow_blink==3:
                plcpkg.Send_message_to_port(ser,plcpkg.crc16.Combining_CRC_and_info(plcpkg.plccmd.WRITE_FRONT_LIGHT_SLOW_BLINK))
            if write_front_lingth_close==4:
                plcpkg.Send_message_to_port(ser,plcpkg.crc16.Combining_CRC_and_info(plcpkg.plccmd.WRITE_FRONT_LIGHT_CLOSE))         
            #control flex pole
            if write_flex_pole_motor_up==1:
                plcpkg.Send_message_to_port(ser,plcpkg.crc16.Combining_CRC_and_info(plcpkg.plccmd.WRITE_FLEX_POLE_MOTOR_UP))
            if write_flex_pole_motor_down==2:
                plcpkg.Send_message_to_port(ser,plcpkg.crc16.Combining_CRC_and_info(plcpkg.plccmd.WRITE_FLEX_POLE_MOTOR_DOWN))
            if write_flex_pole_motor_close==3:
                plcpkg.Send_message_to_port(ser,plcpkg.crc16.Combining_CRC_and_info(plcpkg.plccmd.WRITE_FLEX_POLE_MOTOR_CLOSE))                     
            #brake mobile platform
            if write_mobile_platform_brake_close==1:
                plcpkg.Send_message_to_port(ser,plcpkg.crc16.Combining_CRC_and_info(plcpkg.plccmd.WRITE_MOBILE_PLATFORM_BRAKE_CLOSE))
            if write_mobile_platform_brake_open==2:
                plcpkg.Send_message_to_port(ser,plcpkg.crc16.Combining_CRC_and_info(plcpkg.plccmd.WRITE_MOBILE_PLATFORM_BRAKE_OPEN))
            #electric sitwch
            if write_electric_switch_painting_close==2:
                plcpkg.Send_message_to_port(ser,plcpkg.crc16.Combining_CRC_and_info(plcpkg.plccmd.WRITE_ELECTRIC_SWITCH_PAINTING_CLOSE))
            if write_electric_switch_painting_open==1:
                plcpkg.Send_message_to_port(ser,plcpkg.crc16.Combining_CRC_and_info(plcpkg.plccmd.WRITE_ELECTRIC_SWITCH_PAINTING_OPEN))         
                
        else:
            try:
                ser = serial.Serial(port=plc_port, baudrate=plc_port_baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=1,timeout=0.3, xonxoff=0,rtscts=False,dsrdtr=False)
            except:
                rospy.logerr("Please check PLC Usb port----,I will reconnect after three seconds-----")
                open_serial_port_again_flag=1
                time.sleep(3)
            
        rate.sleep()
if __name__ == "__main__":
    main()