#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
PI = 3.1415926535897

class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)
        

    def move2goal(self):
        """Moves the turtle to the goal."""
        goal_pose = Pose() 
        goal_pose.x = 5
        goal_pose.y = 5

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 0.03

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)
        
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        current_angle = self.pose.theta
        while (current_angle > .05) or (current_angle < (-0.05)):
            vel_msg.angular.z = 0.2
            self.velocity_publisher.publish(vel_msg)
            current_angle = self.pose.theta
            self.rate.sleep()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)



        # user inputs
        forward_speed= input("Input forward speed (max speed 0.9)= ")  
        angular_speed= input("Input angular speed (max value 0.2)= ")

        for i in range(4):
            t0 = rospy.Time.now().to_sec()
        
            while rospy.Time.now().to_sec() < t0 + (3/forward_speed):
                print(type(rospy.Duration.from_sec(3/forward_speed)))
                vel_msg.linear.x = forward_speed
                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()
            vel_msg.linear.x = 0.0
            self.velocity_publisher.publish(vel_msg)


            t1 = rospy.Time.now().to_sec()
            current_angle = 0
            while (current_angle < (PI/2)):
                vel_msg.angular.z = angular_speed
                self.velocity_publisher.publish(vel_msg)
                t2 = rospy.Time.now().to_sec()
                current_angle = angular_speed*(t2-t1)
                self.rate.sleep()
            vel_msg.angular.z = 0.0
            self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass