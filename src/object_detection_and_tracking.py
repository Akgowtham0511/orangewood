import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
from geometry_msgs.msg import PoseStamped

class obj_detection():
    def __init__(self):
        rospy.init_node("object_detection_tracking")
        rospy.Subscriber("/image/ball_animation",Image,callback=self.image_cb)
        self.pose_pub = rospy.Publisher("/turtle/move/pose",PoseStamped,queue_size=10)
        self.bridge = CvBridge()
        self.ball_position = PoseStamped()
        
    def detect_green_ball(self,image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([70, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green) 
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea) 
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                transformed_x = cx 
                transformed_y = self.frame_shape - cy 
                return (transformed_x, transformed_y)
        return None

    def image_cb(self,data):
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        self.frame_shape = img.shape[0]
        ball_position = self.detect_green_ball(img)
        x,y = ball_position
        if ball_position:
            cv2.circle(img, ball_position, 12, (0, 0, 255), -1)

        self.ball_position.header.frame_id = "camera"
        self.ball_position.pose.position.x = x
        self.ball_position.pose.position.y = y
        self.pose_pub.publish(self.ball_position)

        print(self.ball_position)

        # cv2.imshow('Green Ball Detection', img)
        # cv2.waitKey(1)

        # Break the loop if 'q' is pressed
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break
       
        # try:
        # # Convert ROS image message to OpenCV image
        #     cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        
        # # Process the OpenCV image (e.g., display it)
        #     cv2.imshow("Received Image", cv_image)
        #     cv2.waitKey(1)  # Wait for a short time to update the display
        
        # except CvBridgeError as e:
        #     rospy.logerr("CvBridge Error: {0}".format(e))

if __name__ == "__main__":
    obj_detection_obj = obj_detection()
    while not rospy.is_shutdown():
        rospy.spin()