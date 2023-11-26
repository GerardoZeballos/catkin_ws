#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/MyCustomMessageType.h"

#include <sstream>
ros::Publisher chatter_pub;

// Implement the timer callback function
void timerCallback(const ros::TimerEvent&)
{
  // Initialise an empty message of the custom type
  beginner_tutorials::MyCustomMessageType msg;

  // Fill in the fields of the message
  msg.isValid = true;
  msg.restartsRemaining = 42;
  msg.distanceToStart = 4.2;
  msg.distanceToEnd = 42.42;
  msg.taskDescription = "Staying alive";

  // "Push back" data into the vector field
  msg.distancesToOtherRobots.push_back(1.1);
  msg.distancesToOtherRobots.push_back(2.3);
  msg.distancesToOtherRobots.push_back(2.7);

  // Publish the message
  chatter_pub.publish(msg);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker_custom_msg");

  ros::NodeHandle n;

  chatter_pub = n.advertise<beginner_tutorials::MyCustomMessageType>("great_custom_topic", 1000);
  ros::Timer timer = n.createTimer(ros::Duration(1.0), timerCallback);

  // Spin to keep the node alive and process callbacks
  ros::spin();

  return 0;
}
