import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import time
a = 150  # a = distance from the center of the foci of the lemnicate
t = 0    # parameter for lemniscate curve
delta_t = 0.05  # increment for parameter t


ball_radius = 10
ball_color = (0, 255, 0)
frame_size = (500, 500)
fps = 30
class lemniscate():
    def __init__(self):
        rospy.init_node("leminscent_image_publisher")
        self.image_pub = rospy.Publisher("/image/ball_animation",Image,queue_size=10)
        self.bridge = CvBridge()
        self.image_msg = Image()
        self.out = cv2.VideoWriter('lemniscate_video.mp4', cv2.VideoWriter_fourcc(*'DIVX'), fps, frame_size)

    def lemniscate(self,a, t):
        x = np.cos(t) / (1 + np.sin(t)**2)**0.5 * a
        y = np.sin(t) * np.cos(t) / (1 + np.sin(t)**2)**0.5 * a
        return x, y
    
    def draw_lemniscate(self,frame, a, color):
        center = (frame.shape[1] // 2, frame.shape[0] // 2)
        t = np.linspace(0, 2*np.pi, 100)
        x, y = self.lemniscate(a, t)
        x += center[0]
        y += center[1]
        points = np.array([x, y], dtype=np.int32).T
        cv2.polylines(frame, [points], isClosed=False, color=color, thickness=2)

    def update_ball_position(self,a, t, ball_radius):
        x, y = self.lemniscate(a, t)
        x += 250.0
        y += 250.0
        return int(x), int(y)
    
    def main(self):
        t = 0
        while True:
            frame = np.zeros((frame_size[1], frame_size[0], 3), dtype=np.uint8)
            # Draw lemniscate curve
            self.draw_lemniscate(frame, a, (255, 255, 255))
             
            # Update ball position
            ball_position = self.update_ball_position(a, t, ball_radius)
            print(ball_position)

            # Draw ball
            cv2.circle(frame, ball_position, ball_radius, ball_color, -1)

            # Show frame
            # if frame is not None:
            #         frame_ros = np.uint8(frame)
            # image_message = self.bridge.cv2_to_imgmsg(frame_ros, encoding="passthrough")
            cv2.imshow('Lemniscate Video', frame)
            # image_message.header.frame_id = "camera"
            # image_message.height = 150
            # image_message.width = 150
            # image_message.data = list(frame_ros)
            # image_pub.publish(image_message)
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
            ros_image.header.frame_id = "camera"
            # Publish ROS image message
            self.image_pub.publish(ros_image)
            self.out.write(frame)
            # Update parameter t
            t += delta_t
            if t >= 2*np.pi:
                t = 0

            # Break the loop when 'q' is pressed
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break

            time.sleep(0.1)
        self.out.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    image_obj = lemniscate()
    image_obj.main()
    rospy.spin()



            