#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker(velocity):
    pub = rospy.Publisher('cmd_vel', String, queue_size=10)
    rospy.init_node('cmd_vel_node', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo(velocity)
        pub.publish(velocity)
        rate.sleep()

if __name__ == '__main__':
    velocity = raw_input('Enter a number: ')
    try:
        talker(velocity)
    except rospy.ROSInterruptException:
        pass