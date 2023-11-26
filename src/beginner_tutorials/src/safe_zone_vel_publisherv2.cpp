#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <iomanip>
#include <stdlib.h>     // For rand() and RAND_MAX

class SafeZoneVelPublisher {
public:
    SafeZoneVelPublisher() {
        // Initialize ROS node handle
        nh_ = ros::NodeHandle("~");

        // Initialize publishers and subscribers
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
        pose_sub_ = nh_.subscribe("/turtle1/pose", 1, &SafeZoneVelPublisher::poseCallback, this);

        // Set initial velocities
        linear_vel_ = 1.0;
        angular_vel_ = 0.0;

        // Set the safe zone boundaries
        safe_zone_x_min_ = 7.5; // center_x - square_side/2
        safe_zone_x_max_ = 12.5; // center_x + square_side/2
        safe_zone_y_min_ = 7.5; // center_y - square_side/2
        safe_zone_y_max_ = 12.5; // center_y + square_side/2

        // Seed for random number generation
        srand(time(NULL));

        // Start publishing velocities
        publishVelocities();
    }

    void poseCallback(const turtlesim::Pose::ConstPtr& pose) {
        // Check if turtle is inside the safe zone
        if (pose->x >= safe_zone_x_min_ && pose->x <= safe_zone_x_max_ &&
            pose->y >= safe_zone_y_min_ && pose->y <= safe_zone_y_max_) {
            // Inside safe zone, set fixed linear velocity and random angular velocity
            linear_vel_ = 1.0;
            angular_vel_ = 2*double(rand())/double(RAND_MAX) - 1; // You can modify this if you want a random angular velocity as well
        } else {
            // Outside safe zone, set random linear and angular velocities
            linear_vel_ = getRandomVelocity();
            angular_vel_ = getRandomVelocity();
        }
    }

    void publishVelocities() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            geometry_msgs::Twist vel_msg;
            vel_msg.linear.x = linear_vel_;
            vel_msg.angular.z = angular_vel_;
            vel_pub_.publish(vel_msg);
            ros::spinOnce();
            rate.sleep();
        }
    }

    double getRandomVelocity() {
        // Generate a random velocity between -1.0 and 1.0
        return 2.0 * rand() / RAND_MAX - 1.0;
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber pose_sub_;
    double linear_vel_;
    double angular_vel_;
    double safe_zone_x_min_;
    double safe_zone_x_max_;
    double safe_zone_y_min_;
    double safe_zone_y_max_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "safe_zone_vel_publisher");
    SafeZoneVelPublisher safe_zone_vel_publisher;
    ros::spin();
    return 0;
}
