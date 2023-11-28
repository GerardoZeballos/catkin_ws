#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Pose.h>

ros::Publisher second_turtle_pub;
turtlesim::Pose turtle1_pose;

// Callback function of the subscriber
void velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  geometry_msgs::Twist opposite_vel;
  opposite_vel.linear.x = -msg->linear.x;
  opposite_vel.angular.z = -msg->angular.z;

  second_turtle_pub.publish(opposite_vel);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_second_turtle");
  ros::NodeHandle nh;

  // Spawning a new turtle
  ros::service::waitForService("/spawn");
  ros::ServiceClient spawnClient = nh.serviceClient<turtlesim::Spawn>("/spawn");
  turtlesim::Spawn srv;
  srv.request.x = 5.5;
  srv.request.y = 5.5;
  srv.request.name = "second_turtle";
  spawnClient.call(srv);


  srv.request.x = turtle1_pose.x;
  srv.request.y = turtle1_pose.y;
  srv.request.theta = turtle1_pose.theta;
  srv.request.name = "second_turtle";

  // Publisher of the opposite velocities for the second turtle
  second_turtle_pub = nh.advertise<geometry_msgs::Twist>("second_turtle/cmd_vel", 10);

  // Subscriber for the first-turtle-velocities topic
  ros::Subscriber sub = nh.subscribe("turtle1/cmd_vel", 10, velCallback);

  ros::spin();

  return 0;
}