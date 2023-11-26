#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/MyCustomMessageType.h"


// Implement the subscriber callback function
void subscriberCallback(const beginner_tutorials::MyCustomMessageType& msg)
{
  // Extract the data from the message
  bool is_valid = msg.isValid;
  uint restarts_remaining = msg.restartsRemaining;
  std::string taskDescription = msg.taskDescription;
  // and so on for the other fields.

  // Print the elements of the vector
  std::vector<float>  distances_vector;
  distances_vector.assign(msg.distancesToOtherRobots.begin(), msg.distancesToOtherRobots.end());

  int vector_size = distances_vector.size();
  ROS_INFO_STREAM("*********[SUBSCRIBER CPP NODE]*************");
  ROS_INFO("The is_valid parameter is: %s", is_valid ? "true" : "false");
  ROS_INFO("The restarts_remaining parameter is: [%i]", restarts_remaining);
  ROS_INFO("The taskDescription parameter is: %s", taskDescription.c_str());
  for (int i=0; i<vector_size; i++)
  {
    ROS_INFO_STREAM("[SUBSCRIBER CPP NODE] distancesToOtherRobots[" << i << "] = " << msg.distancesToOtherRobots.at(i));
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_custom_msg");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("great_custom_topic", 1000, subscriberCallback);

  ros::spin();

  return 0;
}
