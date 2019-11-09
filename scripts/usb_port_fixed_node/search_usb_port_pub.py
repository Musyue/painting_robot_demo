#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from std_msgs.msg import String
import serial
from time import sleep
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
from textwrap import wrap
class SerarchUSB():
    def __init__(self,nodename):
        self.nodename=nodename
        self.iostatebuff=[]
        self.readstringlength=25
    def Init_node(self):
        rospy.init_node(self.nodename)
    def search_modbus_port(self,port,Baudrate):
        """
        id : 1---->stand bar
        id : 2----->rotation
        id : 3------>Upper and lower climbing pole

        """
        try:
            #Connect to the slave
            master = modbus_rtu.RtuMaster(
                serial.Serial(port=port, baudrate=Baudrate, bytesize=8, parity='O', stopbits=1, xonxoff=0)
            )
            master.set_timeout(5.0)
            master.set_verbose(True)
            buf1=master.execute(1, cst.READ_HOLDING_REGISTERS, 0, 8)
            buf2=master.execute(2, cst.READ_HOLDING_REGISTERS, 0, 8)
            buf3=master.execute(3, cst.READ_HOLDING_REGISTERS, 0, 8)

            rospy.loginfo(buf1)
            rospy.loginfo(buf2)
            rospy.loginfo(buf3)
            if len(buf1)!=0:
                return True
        except modbus_tk.modbus.ModbusError as exc:
            rospy.logerr("%s- Code=%d", exc, exc.get_exception_code())
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
    def search_plc_port(self,port,Baudrate,Message):
        ser = serial.Serial(port, Baudrate, timeout=0.5)  #/dev/ttyUSB0
        readbuffer,strt=self.Send_message_to_port(ser,Message)
        if len(readbuffer)!=0:
            if readbuffer[-1]==1:
                return True
            else:
                return False
def main():
    iob=SerarchUSB("search_port_node")
    iob.Init_node()

    port_list=[]
    imu_port = rospy.get_param("imu_port")
    rospy.loginfo("%s is %s", rospy.resolve_name('imu_port'), imu_port)
    port_list.append(imu_port)
    # fetch the utterance parameter from our parent namespace
    imu_port_baudrate = rospy.get_param('imu_port_baudrate')
    rospy.loginfo("%s is %s", rospy.resolve_name('imu_port_baudrate'), imu_port_baudrate)

    plc_port = rospy.get_param("plc_port")
    # print(type(plc_port))
    rospy.loginfo("%s is %s", rospy.resolve_name('plc_port'), plc_port)
    port_list.append(plc_port)
    # fetch the utterance parameter from our parent namespace
    plc_port_baudrate = rospy.get_param('plc_port_baudrate')
    rospy.loginfo("%s is %s", rospy.resolve_name('plc_port_baudrate'), plc_port_baudrate)

    climb_port = rospy.get_param("climb_port")
    rospy.loginfo("%s is %s", rospy.resolve_name('climb_port'), climb_port)
    port_list.append(climb_port)
    # fetch the utterance parameter from our parent namespace
    climb_port_baudrate = rospy.get_param('climb_port_baudrate')
    rospy.loginfo("%s is %s", rospy.resolve_name('climb_port_baudrate'), climb_port_baudrate)
    # try:
    #     serial_imu = serial.Serial(imu_port, imu_port_baudrate, timeout=0.5)  #/dev/ttyUSB0
    #     if serial_imu.isOpen() :
    #         rospy.loginfo("open imu port success")
    #     else :
    #         rospy.logerr("open imu port failed")
    # except:
    #     serial = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.5)  #/dev/ttyUSB0
    #     if serial.isOpen() :
    #         print("open port success")
    #     else :
    #         print("open port failed")
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo("%s is %s", rospy.resolve_name('imu_port'), imu_port)
        rospy.loginfo("%s is %s", rospy.resolve_name('imu_port_baudrate'), imu_port_baudrate)
        rospy.loginfo("%s is %s", rospy.resolve_name('plc_port'), plc_port)
        rospy.loginfo("%s is %s", rospy.resolve_name('plc_port_baudrate'), plc_port_baudrate)
        rospy.loginfo("%s is %s", rospy.resolve_name('climb_port'), climb_port)
        rospy.loginfo("%s is %s", rospy.resolve_name('climb_port_baudrate'), climb_port_baudrate)
        rate.sleep()
if __name__ == '__main__':
    main()