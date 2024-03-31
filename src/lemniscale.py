import rospy
from geometry_msgs.msg import PoseStamped, Twist
from math import atan2, sqrt,cos,sin

class TurtleController:
    def __init__(self):
        rospy.init_node('turtle_controller')
        self.ball_position_sub = rospy.Subscriber('/turtle/move/pose', PoseStamped, self.ball_position_callback)
        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.turtle_position = PoseStamped()
        self.rate = rospy.Rate(10)

        # Lemniscate parameters
        self.a = 5  # Distance from the center to the foci
        self.angular_speed = 0.1  # Angular speed for tracing the lemniscate

    def transform_coordinates(self,x_lemniscate, y_lemniscate, image_width, image_height):
        # Reverse y-axis and translate origin to center of image
        y_turtlesim = image_height - y_lemniscate
        x_turtlesim = x_lemniscate - image_width / 2
        y_turtlesim -= image_height / 2

        # Scale coordinates to fit turtlesim frame
        scale_factor = 2  # Adjust based on your setup
        x_turtlesim /= scale_factor
        y_turtlesim /= scale_factor

        return x_turtlesim, y_turtlesim


    def ball_position_callback(self, msg):
        self.turtle_position = msg

        tx,ty = self.transform_coordinates(self.turtle_position.pose.position.x,self.turtle_position.pose.position.y,150,150)
        # Calculate desired position on lemniscate curve
        t = rospy.get_time()
        x = self.a * (2 * cos(self.angular_speed * t)) / (1 + sin(self.angular_speed * t)**2)
        y = self.a * (2 * sin(self.angular_speed * t) * cos(self.angular_speed * t)) / (1 + sin(self.angular_speed * t)**2)
        
        # Calculate angle and distance to desired position
        angle = atan2(y - ty, x - tx)
        distance = sqrt((x - tx)**2 + (y - ty)**2)
        
        # Control the turtle's movement
        desired_velocity = Twist()
        if distance > 0.1:
            desired_velocity.linear.x = 0.5 * distance
            desired_velocity.angular.z = 2.0 * angle
        else:
            desired_velocity.linear.x = 0.0
            desired_velocity.angular.z = 0.0
        
        # Publish velocity command
        print(desired_velocity)
        self.vel_pub.publish(desired_velocity)

    def get_distance(self, p1, p2):
        return sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    
if __name__ == '__main__':
    try:
        turtle_controller = TurtleController()
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
