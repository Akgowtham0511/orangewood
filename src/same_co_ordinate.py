import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np

def transform_coordinates2(x_lemniscate, y_lemniscate, image_width, image_height):
    # Reverse y-axis and translate origin to center of image
    y_turtlesim = image_height - y_lemniscate
    x_turtlesim = x_lemniscate - image_width / 2
    y_turtlesim -= image_height / 2

    return x_turtlesim, y_turtlesim


def transform_coordinates2(x_image, y_image, image_width, image_height):
    # Reverse y-axis
    y_turtlesim = image_height - y_image
    turtlesim_width = 11
    turtlesim_height = 11

    # Translate origin to center of turtlesim
    x_turtlesim = x_image - (image_width / 2)
    y_turtlesim -= (image_height / 2)

    # Scale coordinates to fit turtlesim frame
    scale_factor_x = turtlesim_width / image_width
    scale_factor_y = turtlesim_height / image_height
    x_turtlesim *= scale_factor_x
    y_turtlesim *= scale_factor_y

    return x_turtlesim, y_turtlesim

def transform_coordinates(x_image, y_image, image_width, image_height):
    # Clamp coordinates to ensure they stay within target range
    x_target = x_image / image_width * 10 # trying to normaliza it by subracting it to the min and max of cal values
    y_target = y_image / image_height * 10
    print('x_target, y_target',x_target, y_target)
    return x_target, y_target

def detect_green_ball(frame):
    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define range of green color in HSV
    lower_green = np.array([40, 40, 40])
    upper_green = np.array([70, 255, 255])
    
    # Threshold the HSV image to get only green colors
    mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # If contours are found
    if contours:
        # Get the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        # Calculate centroid of the largest contour
        M = cv2.moments(largest_contour)
        print(frame.shape)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = frame.shape[0] - int(M["m01"] / M["m00"]) 
            print(cx,cy)
            # Transform coordinates to the desired coordinate system
            transformed_x, transformed_y = transform_coordinates(cx, cy, frame.shape[1], frame.shape[0])
            print('transformed_x, transformed_y',transformed_x, transformed_y)
            return transformed_x, transformed_y
            #return cx,cy
    
    # If no contours found or centroid calculation fails, return None
    return None, None

def image_callback(msg):
    try:
        frame = CvBridge().imgmsg_to_cv2(msg, "passthrough")
        green_ball_x, green_ball_y = detect_green_ball(frame)
        print("in function image_callback:",green_ball_x, green_ball_y)
        if green_ball_x is not None and green_ball_y is not None:
            publish_green_ball_coordinates(green_ball_x, green_ball_y)
    except CvBridgeError as e:
        print(e)

def publish_green_ball_coordinates(x, y):
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    print('publish_green_ball_coordinates',pose_msg.pose.position.x,pose_msg.pose.position.y)
    pub.publish(pose_msg)

if __name__ == '__main__':
    rospy.init_node('green_ball_tracker')
    pub = rospy.Publisher('/green_ball_pose', PoseStamped, queue_size=10)
    rospy.Subscriber("/image/ball_animation", Image, image_callback)
    rospy.spin()
