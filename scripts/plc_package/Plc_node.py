
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
        if open_serial_port_again_flag!=1:
            pass
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