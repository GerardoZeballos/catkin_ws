#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <stdlib.h>
#include <time.h>

class SafeZoneVelPublisher {
public:
  SafeZoneVelPublisher() : nh_(), linear_velocity_(1.0), angular_velocity_(0.0) {
    pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    sub_ = nh_.subscribe("turtle1/pose", 1000, &SafeZoneVelPublisher::poseCallback, this);
    srand(time(0));
  }

  void poseCallback(const turtlesim::Pose& pose) {
    // Check if the turtle is in the safe zone (5x5 dimension square around the center).
    if (isInSafeZone(pose.x, pose.y)) {
      // Inside the safe zone, publish a fixed linear velocity and random angular velocity.
      linear_velocity_ = 1.0;
      angular_velocity_ = double(rand()) / double(RAND_MAX);
    } else {
      // Outside the safe zone, publish random linear and angular velocities.
      linear_velocity_ = double(rand()) / double(RAND_MAX);
      angular_velocity_ = 2 * double(rand()) / double(RAND_MAX) - 1;
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

  bool isInSafeZone(double x, double y) {
    // Define the dimensions of the safe zone (5x5 square around the center).
    double safeZoneSize = 5.0;
    double minX = 5.0 - safeZoneSize / 2.0;
    double maxX = 5.0 + safeZoneSize / 2.0;
    double minY = 5.0 - safeZoneSize / 2.0;
    double maxY = 5.0 + safeZoneSize / 2.0;

    // Check if the turtle is inside the safe zone.
    return (x >= minX && x <= maxX && y >= minY && y <= maxY);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  double linear_velocity_;
  double angular_velocity_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "safe_zone_vel_publisher");
  SafeZoneVelPublisher node;
  ros::spin();
  return 0;
}
