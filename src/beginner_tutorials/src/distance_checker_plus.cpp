#include <ros/ros.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Pose.h>
#include <math.h>
#include <evaluation_ros/ComputeDistance.h>

ros::ServiceClient spawnClient;
ros::ServiceServer distanceServer;

float turtle1_x, turtle1_y, turtle2_x, turtle2_y;

bool spawnTurtle()
{
  turtlesim::Spawn srv;
  srv.request.x = 5.0;
  srv.request.y = 5.0;
  srv.request.theta = 0.0;
  srv.request.name = "turtle2";

  if (spawnClient.call(srv))
  {
    ROS_INFO("Turtle2 spawned successfully");
    return true;
  }
  else
  {
    ROS_ERROR("Failed to spawn Turtle2");
    return false;
  }
}

bool computeDistance(evaluation_ros::ComputeDistance::Request &req,
                     evaluation_ros::ComputeDistance::Response &res)
{
  float distance = sqrt(pow(turtle1_x - turtle2_x, 2) + pow(turtle1_y - turtle2_y, 2));
  res.distance = distance;
  return true;
}

void turtle1PoseCallback(const turtlesim::Pose::ConstPtr &pose)
{
  turtle1_x = pose->x;
  turtle1_y = pose->y;
}

void turtle2PoseCallback(const turtlesim::Pose::ConstPtr &pose)
{
  turtle2_x = pose->x;
  turtle2_y = pose->y;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "distance_checker_plus_node");
  ros::NodeHandle nh;

  ros::service::waitForService("spawn");
  spawnClient = nh.serviceClient<turtlesim::Spawn>("spawn");

  ros::Subscriber sub1 = nh.subscribe("/turtle1/pose", 1000, turtle1PoseCallback);
  ros::Subscriber sub2 = nh.subscribe("/turtle2/pose", 1000, turtle2PoseCallback);

  spawnTurtle();

  distanceServer = nh.advertiseService("calculate_distance", computeDistance);

  ros::spin();

  return 0;
}