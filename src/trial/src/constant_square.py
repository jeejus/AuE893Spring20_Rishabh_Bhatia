#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897

def move_square():
    rospy.init_node('robot_square', anonymous=True)
    # Create a publisher which can "talk" to Turtlesim and tell it to move
    vel_square_2 = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)

    # Create a Twist message and add linear x and angular z values
    square2_cmd = Twist()
    


    # Save current time and set publish rate at 10 Hz
    for i in range(4):
        t0 = rospy.Time.now()
        rate = rospy.Rate(10)

        while rospy.Time.now() < t0 + rospy.Duration.from_sec(10):
            square2_cmd.linear.x = 0.2
            vel_square_2.publish(square2_cmd)
            rate.sleep()
        square2_cmd.linear.x = 0.0
        vel_square_2.publish(square2_cmd)


        t1 = rospy.Time.now().to_sec()
        current_angle = 0
        while (current_angle < (PI/2)):
            square2_cmd.angular.z = 0.2
            vel_square_2.publish(square2_cmd)
            t2 = rospy.Time.now().to_sec()
            current_angle = 0.2*(t2-t1)
            rate.sleep()
        square2_cmd.angular.z = 0.0
        vel_square_2.publish(square2_cmd)
    

if __name__ == '__main__':
    try:
        move_square()

    except rospy.ROSInterruptException:
        pass