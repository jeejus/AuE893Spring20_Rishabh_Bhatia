#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897

def move_circle():
    rospy.init_node('robot_circle', anonymous=True)
    # Create a publisher which can "talk" to Turtlesim and tell it to move
    vel_circle = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)

    # Create a Twist message and add linear x and angular z values
    circle_cmd = Twist()
    circle_cmd.linear.x = 2.0
    circle_cmd.angular.z = 2.0


    # Save current time and set publish rate at 10 Hz
    t0 = rospy.Time.now()
    rate = rospy.Rate(10)

    # For the next 5 seconds publish cmd_vel move commands to Turtlesim
    while rospy.Time.now() < t0 + rospy.Duration.from_sec(3):
        vel_circle.publish(circle_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_circle()

    except rospy.ROSInterruptException:
        pass