# Task Description

## Task 1: Motion Generation
Generate a video feed of a green ball continuously tracing a lemniscate curve using OpenCV. The frame size should be 500 x 500 pixels. Consider the centre of the frame to be the lemniscate origin, and the foci to be 150 pixels apart and publish the image in a ros topic name /image/ball_animation.

### Instructions:
1. Ensure you have OpenCV installed (`pip3 install opencv-python`).
2. The `task1.py` script to generate the video feed which publishes the video feed on a ros topic.

## Task 2: Ball Tracking
Create a node that subscribes to the generated video feed and uses OpenCV to continuously compute the (x, y) center coordinates of the green ball o the lemniscate curve. Publish these coordinates to the topic '/turtle/move_pose' using the PoseStamped message type.

### Instructions:
1. The `task2.py` script to track the green ball and publish its position on a ros topic.

## Task 3: Turtle Control
Create a control node (script) which subscribes to the topic '/turtle/move_pose' and makes the turtle in turtlesim follow the waypoints provided by the ball tracking node. Ensure that the turtle's motion follows a lemniscate curve exactly as shown in the video feed.

### Instructions:
1. Make sure you have turtlesim installed (`sudo apt-get install ros-$ROS_DISTRO-turtlesim`).
2. The `task3_cpp.cpp` script contains the code for turtlesim follow the lemniscate curve using cmd_vel.

#### Running all at once
1. The `lemniscate.launch` contains the launch file to run all the code at once on a single command terminal.
2. Run `catkin_make`
3.`roslaunch orangewood lemniscate.launch`

# Running the Code

1. Clone this repository to your ROS workspace:

```bash
git clone <repository_url>