#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point,PoseStamped
from math import atan2, sqrt

class TurtleFollower:
    def __init__(self):
        rospy.init_node('turtle_follower')
        rospy.Subscriber('/green_ball_pose', PoseStamped, self.ball_position_callback)
        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(6)
        # self.ball_position = Point()
        self.turtle_position = Point()

    def get_distance(self, p1, p2):
        return sqrt((p1.pose.position.x - p2.x)**2 + (p1.pose.position.y - p2.y)**2)
    
    def transform_to_turtlesim_coordinates(self,x, y):
        # Reverse y-axis
        image_width = 150
        image_height = 150
        turtlesim_width = 10
        turtlesim_height = 10
        y_turtlesim = image_height - y

        # Translate origin to center of image
        x_turtlesim = x - image_width / 2
        y_turtlesim -= image_height / 2

        # Scale coordinates to fit turtlesim frame
        scale_factor_x = turtlesim_width / image_width
        scale_factor_y = turtlesim_height / image_height
        x_turtlesim *= scale_factor_x
        y_turtlesim *= scale_factor_y

        # Clip coordinates to turtlesim frame
        # x_turtlesim = max(0, min(x_turtlesim, turtlesim_width - 1))
        # y_turtlesim = max(0, min(y_turtlesim, turtlesim_height - 1))

        return x_turtlesim, y_turtlesim

    def transform_coordinates(self,x_lemniscate, y_lemniscate, image_width, image_height):
        # Reverse y-axis and translate origin to center of image
        # y_turtlesim = image_height - y_lemniscate
        # x_turtlesim = x_lemniscate - image_width / 2
        # y_turtlesim -= image_height / 2

        # Scale coordinates to fit turtlesim frame
        # scale_factor = 20  # Adjust based on your setup
        # x_turtlesim /= scale_factor
        # y_turtlesim /= scale_factor

        x_turtlesim = 0
        y_turtlesim = 0

        return x_turtlesim, y_turtlesim

    def ball_position_callback(self, msg):
        self.ball_position = msg
        print(self.ball_position.pose.position.x,self.ball_position.pose.position.x)
        # self.ball_position.pose.position.x,self.ball_position.pose.position.y = self.transform_to_turtlesim_coordinates(self.ball_position.pose.position.x,self.ball_position.pose.position.y)
        desired_velocity = Twist()
        angle = atan2(self.ball_position.pose.position.y - self.turtle_position.y, self.ball_position.pose.position.x - self.turtle_position.x)
        distance = self.get_distance(self.ball_position, self.turtle_position)
        print("distance: ",distance)
        print("angle: ",angle)

        
        # x_vel = (tx - self.turtle_position.x ) * distance
        # y_vel = (ty - self.turtle_position.y) *distance

        if distance > 0.4:
            desired_velocity.linear.x = 0.5 *distance
            desired_velocity.angular.z = 4.0 * angle
        else:
            desired_velocity.linear.x = 0.0
            desired_velocity.angular.z = 0.0

        # if angle > 0.5:
        #     desired_velocity.linear.x = 0.0
        #     desired_velocity.angular.z = 4.0 * angle
            
        # if angle < 0.5 and distance > 1:
        #     desired_velocity.linear.x = 0.2
        #     desired_velocity.angular.z = 0.0

        # else:
        #     desired_velocity.linear.x = 0.0
        #     desired_velocity.angular.z = 0.0

        # desired_velocity.linear.x = x_vel
        # desired_velocity.angular.z = y_vel
            
        self.vel_pub.publish(desired_velocity)
        print(desired_velocity)

    
    # def follow_ball(self):
    #     while not rospy.is_shutdown():
           
    #         self.rate.sleep()

if __name__ == '__main__':
    turtle_follower = TurtleFollower()
    while not rospy.is_shutdown():
        rospy.spin()

   
