#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def publisher_node():
    # set up publisher node
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    
    # create twist object and set ros rate
    twist=Twist()
    freq = 10
    rate = rospy.Rate(freq)

    # move robot forward 1m
    twist.linear.x=0.1
    twist.angular.z=0.0
    linear_dist = 1.0
    linear_time = math.ceil((linear_dist/abs(twist.linear.x))*freq)

    count = 0
    while not rospy.is_shutdown() and count < linear_time:
        count += 1
        cmd_pub.publish(twist)
        rate.sleep()
    
    # rotate robot 360 degrees clockwise
    twist.linear.x=0.0
    twist.angular.z=-0.1
    rot_dist = 2*math.pi
    rot_time = math.ceil((rot_dist/abs(twist.angular.z))*freq)

    count = 0
    while not rospy.is_shutdown() and count < rot_time:
        count += 1
        cmd_pub.publish(twist)
        rate.sleep()
    
    twist.angular.z=0.0
    cmd_pub.publish(twist)

def main():
    try:
        rospy.init_node('motor')
        publisher_node()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
