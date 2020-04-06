#!/usr/bin/env python
# coding=utf-8

import rospy
import math
from std_msgs.msg import String
def deg_to_rad(tuplelist):
    dd=[]
    for i in tuplelist:
        dd.append(i*math.pi/180)
    return tuple(dd)
def talker():
    pub = rospy.Publisher('/aubo_ros_script/movej', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    move_point_degree=deg_to_rad([-16.34,-37.01,78.92,-71.36,-106.303,90.01])
    while not rospy.is_shutdown():
        hello_str="movej"+str(move_point_degree)
        # hello_str = "movej(-0.28525098, -0.53203763,  1.36669062, -1.24286441, -1.85604731, 1.57079633)"#(-0.28525098, -0.53203763,  1.36669062, -1.24286441, -1.85604731, 1.57079633)(-0.28525098, -0.53203763,  1.36669062, -1.24286441, -1.85604731, 1.57079633)"
        # hello_str="movel(0.71039368, -0.53203763,  1.36669062, -1.24286441, -0.86040264, 1.57079633)(0.01039368, -0.03203763,  1.36669062, -1.24286441, -0.86040264, 1.57079633)"
        rospy.loginfo(hello_str)
        pub.publish(hello_str)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
