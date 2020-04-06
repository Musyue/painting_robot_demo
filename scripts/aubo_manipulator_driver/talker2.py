#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('/aubo_ros_script/movel', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        
        #hello_str = "movej(-0.28525098, -0.53203763,  1.36669062, -1.24286441, -1.85604731, 1.57079633)(-0.28525098, -0.53203763,  1.36669062, -1.24286441, -1.85604731, 1.57079633)(-0.28525098, -0.53203763,  1.36669062, -1.24286441, -1.85604731, 1.57079633)"
        hello_str="movel(0.71039368, -0.53203763,  1.36669062, -1.24286441, -0.86040264, 1.57079633)(0.01039368, -0.03203763,  1.36669062, -1.24286441, -0.86040264, 1.57079633)"
        rospy.loginfo(hello_str)
        pub.publish(hello_str)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
