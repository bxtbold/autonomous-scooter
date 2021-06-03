#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import time

def talker():
    twist = raw_input('Enter: ')
    pub = rospy.Publisher('/cmd_vel', String, queue_size=10)
    #pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('cmd_vel_node', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    

    while not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass