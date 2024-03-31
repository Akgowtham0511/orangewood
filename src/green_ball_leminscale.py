import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
# Function to calculate lemniscate curve coordinates
rospy.init_node("leminscent_image_publisher")
bridge = CvBridge()
image_msg = Image()
image_pub = rospy.Publisher("/image/ball_animation",Image,queue_size=10)
def lemniscate(a, t):
    x = np.cos(t) / (1 + np.sin(t)**2)**0.5 * a
    y = np.sin(t) * np.cos(t) / (1 + np.sin(t)**2)**0.5 * a
    return x, y

# Function to draw the lemniscate curve on the frame
def draw_lemniscate(frame, a, color):
    center = (frame.shape[1] // 2, frame.shape[0] // 2)
    t = np.linspace(0, 2*np.pi, 100)
    x, y = lemniscate(a, t)
    x += center[0]
    y += center[1]
    points = np.array([x, y], dtype=np.int32).T
    cv2.polylines(frame, [points], isClosed=False, color=color, thickness=2)

# Function to update ball position along the lemniscate curve
def update_ball_position(a, t, ball_radius):
    x, y = lemniscate(a, t)
    x += frame.shape[1] // 2
    y += frame.shape[0] // 2
    return int(x), int(y)

# Define video properties
frame_size = (500, 500)
fps = 30
out = cv2.VideoWriter('lemniscate_video.avi', cv2.VideoWriter_fourcc(*'DIVX'), fps, frame_size)

# Parameters for lemniscate curve
a = 150  # distance from the center to the foci
t = 0    # parameter for lemniscate curve
delta_t = 0.05  # increment for parameter t

# Ball properties
ball_radius = 10
ball_color = (0, 255, 0)  # Green color for the ball

# Main loop for generating video frames
while True:
    frame = np.zeros((frame_size[1], frame_size[0], 3), dtype=np.uint8)

    # Draw lemniscate curve
    draw_lemniscate(frame, a, (255, 255, 255))

    # Update ball position
    ball_position = update_ball_position(a, t, ball_radius)

    # Draw ball
    cv2.circle(frame, ball_position, ball_radius, ball_color, -1)

    # Show frame
    if frame is not None:
            frame_ros = np.uint8(frame)
    image_message = bridge.cv2_to_imgmsg(frame_ros, encoding="passthrough")
    cv2.imshow('Lemniscate Video', frame)
    # image_message.header.frame_id = "camera"
    # image_message.height = 150
    # image_message.width = 150
    # image_message.data = list(frame_ros)
    # image_pub.publish(image_message)

    ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    ros_image.header.frame_id = "camera"

    # Publish ROS image message
    image_pub.publish(ros_image)

    out.write(frame)

    # Update parameter t
    t += delta_t
    if t >= 2*np.pi:
        t = 0

    # Break the loop when 'q' is pressed
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

# Release video writer and close all windows
out.release()
cv2.destroyAllWindows()
