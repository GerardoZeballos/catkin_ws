#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <stdlib.h>
#include <time.h>

class SmartTurtle {
public:
  SmartTurtle() : nh_(), linear_velocity_(1.0), angular_velocity_(0.0), hit_wall_(false) {
    pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    sub_ = nh_.subscribe("turtle1/pose", 1000, &SmartTurtle::poseCallback, this);
    srand(time(0));
  }

  void poseCallback(const turtlesim::Pose& pose) {
    // Check if the turtle has hit a wall.
    if (isHittingWall(pose)) {
      // If hitting the wall, choose a random direction to turn away.
      angular_velocity_ = 2 * double(rand()) / double(RAND_MAX) - 1;
      hit_wall_ = true;
    } else {
      // If not hitting the wall, move forward with a fixed linear velocity.
      linear_velocity_ = 1.0;
      hit_wall_ = false;
    }

    // Publish the velocity command.
    publishVelocity();
  }

  void publishVelocity() {
    geometry_msgs::Twist msg;
    msg.linear.x = linear_velocity_;
    msg.angular.z = angular_velocity_;

    pub_.publish(msg);

    // Log the details.
    ROS_INFO_STREAM("Publishing velocity command:"
                    << " linear=" << msg.linear.x
                    << " angular=" << msg.angular.z);
  }

  bool isHittingWall(const turtlesim::Pose& pose) {
    // Set a threshold for the distance from the walls to consider hitting the wall.
    double wall_threshold = 0.2;

    // Check if the turtle is too close to any of the walls.
    return (pose.x < wall_threshold || pose.x > 10 - wall_threshold ||
            pose.y < wall_threshold || pose.y > 10 - wall_threshold);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  double linear_velocity_;
  double angular_velocity_;
  bool hit_wall_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "smart_turtle");
  SmartTurtle node;
  ros::spin();
  return 0;
}