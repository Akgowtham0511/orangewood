import rospy
from geometry_msgs.msg import PoseStamped, Twist
from turtlesim.msg import Pose
from math import atan2, sqrt

class TurtleController:
    def __init__(self):
        rospy.init_node('turtle_controller')
        rospy.Subscriber('/green_ball_pose', PoseStamped, self.green_ball_callback)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose, self.update_pose)
        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.image_width = 500  
        self.image_height = 500  
        self.turtlesim_width = 11 # width of the turtlsim frame from what observed by moving using the teleop key
        self.turtlesim_height = 11  #heigh of the turtlesim frame
        self.pose_x = 5.5
        self.pose_y = 5.5
        self.pose_theta = 0

    def update_pose(self, data):
        self.pose = data
        self.pose_x = round(self.pose.x, 4)
        self.pose_y = round(self.pose.y, 4)
        self.pose_theta = data.theta

    def green_ball_callback(self, msg):
        # Extract x, y coordinates from PoseStamped message
        green_ball_x = msg.pose.position.x
        green_ball_y = msg.pose.position.y
        print('green_ball_callback',green_ball_x,green_ball_y)
        # Scale down the coordinates to match turtlesim's range
        # scaled_x = green_ball_x * (self.turtlesim_width / self.image_width)
        # scaled_y = green_ball_y * (self.turtlesim_height / self.image_height)
        # Control the turtle to move towards the scaled coordinates
        self.control_turtle_function(green_ball_x, green_ball_y)

    def control_turtle_function(self, target_x, target_y):
        print('target_x,target_y',target_x,target_y)
        linear_speed = 1.0  
        target_angle = atan2(target_y-self.pose_y, target_x-self.pose_x)
        twist_msg = Twist()
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = 3.9 * (target_angle-self.pose_theta)
        self.vel_pub.publish(twist_msg)

if __name__ == '__main__':
    turtle_controller = TurtleController()
    while not rospy.is_shutdown():
        rospy.spin()
    
   
