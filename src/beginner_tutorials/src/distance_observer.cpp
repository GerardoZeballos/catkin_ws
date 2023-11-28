#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <beginner_tutorials/ComputeDistance.h>

double turtle1_x, turtle1_y, turtle2_x, turtle2_y;

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
    ros::init(argc, argv, "distance_observer_node");
    ros::NodeHandle nh;

    ros::Subscriber sub1 = nh.subscribe("turtle1/pose", 1000, turtle1PoseCallback);
    ros::Subscriber sub2 = nh.subscribe("turtle2/pose", 1000, turtle2PoseCallback);

    ros::service::waitForService("calculate_distance");

    ros::Rate loop_rate(2); // 0.5 Hz

    while (ros::ok())
    {
        beginner_tutorials::ComputeDistance srv;
        srv.request.turtle1_x = turtle1_x;
        srv.request.turtle1_y = turtle1_y;
        srv.request.turtle2_x = turtle2_x;
        srv.request.turtle2_y = turtle2_y;

        // Call the distance computation service
        if (ros::service::call("calculate_distance", srv))
        {
            ROS_INFO("Distance between turtles: %.2f", srv.response.distance);
        }
        else
        {
            ROS_ERROR("Failed to call service calculate_distance");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}