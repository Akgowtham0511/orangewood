#include <cstdlib> 
#include <iostream> 
#include "ros/ros.h" 
#include "geometry_msgs/Twist.h" 
#include "geometry_msgs/Pose2D.h" 
#include "geometry_msgs/PoseStamped.h"
#include "turtlesim/Pose.h"
#include "math.h"
#include <cmath>
using namespace std;

float turtle_theta = 0;
float current_turtlesim_x = 5.5; //As the turtlrsim always starts from the center
float current_turtlesim_y = 5.5; //As the turtlrsim always starts from the center
ros::Publisher velocityPub;
void updatePose(const geometry_msgs::PoseStamped &currentPose) {
  float target_x = currentPose.pose.position.x;
  float target_y = currentPose.pose.position.y;

  geometry_msgs::Twist twist_msg;
  float linear_speed = 1;
  double target_angle = atan2(target_y-current_turtlesim_y, target_x-current_turtlesim_x);

  twist_msg.linear.x = linear_speed;
  twist_msg.angular.z = 3.9 * (target_angle-turtle_theta);
  velocityPub.publish(twist_msg);
  std::cout<<"Current goal is: "<<target_x<<" "<<target_y<<endl;
}

void turtelPose(const turtlesim::Pose &turtle_pose)
{
  current_turtlesim_x = turtle_pose.x;
  current_turtlesim_y = turtle_pose.y;
  turtle_theta = turtle_pose.theta;
}
 
int main(int argc, char **argv) {
ros::init(argc, argv, "cmd_vel");
ros::NodeHandle node;
velocityPub =
    node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 0);

ros::Subscriber currentPoseSub =
    node.subscribe("/green_ball_pose", 0, updatePose);

ros::Subscriber currentturtlesimPose =
    node.subscribe("/turtle1/pose", 0, turtelPose);
 
ros::Rate loop_rate(10); 
 
  while (ros::ok()) {
    ros::spinOnce();
    //setVelocity();
    //velocityPub.publish(velCommand);
    loop_rate.sleep();
  }
 
  return 0;
}