#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

def talker():
    linear = raw_input('Linear: ')
    angular = raw_input('Angular: ')
    old_velocity = 0
    #pub = rospy.Publisher('/cmd_vel', String, queue_size=10)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('cmd_vel_node', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    
    twist = Twist()
    twist.linear.x = float(linear)
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = float(angular)

    while not rospy.is_shutdown():
        current_time = time.time()
        trigger = 0
    
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass