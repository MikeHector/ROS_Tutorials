#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def just_go_fwd():
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('just_go_fwd', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        command = Twist()
        command.linear.x = 0.1
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = 0.0
        
        rospy.loginfo(command)
        pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    try:
        just_go_fwd()
    except rospy.ROSInterruptException:
        pass
