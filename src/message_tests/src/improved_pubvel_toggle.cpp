#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <message_tests/Changerate.h>
#include <message_tests/StopStart.h>

bool forward = true;
double newfrequency;
bool ratechanged = false;
bool turtle_On = true;
double current_speed = 1.0;

bool toggleForward(
    std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &resp){
        forward = !forward;
        ROS_INFO_STREAM("Now sending "<<(forward?
                "forward":"rotate")<< " commands.");
    return true;
}

bool changeRate(
    message_tests::Changerate::Request &req,
    message_tests::Changerate::Response &resp){

        ROS_INFO_STREAM("Changing rate to "<<req.newrate);

        newfrequency = req.newrate;
        ratechanged = true;
        return true;
}

bool StopandRun(
    message_tests::StopStart::Request &req,
    message_tests::StopStart::Response &resp){
        turtle_On = req.run;
        ROS_INFO_STREAM("Turtle is "<<(turtle_On?
                "running":"stopped"));
        return true;
}

int main(int argc, char **argv){
    ros::init(argc,argv,"improved_pubvel_toggle");
    ros::NodeHandle nh;
        
    ros::ServiceServer server_toggle_forward = 
        nh.advertiseService("toggle_forward",&toggleForward);

    ros::ServiceServer server_change_rate =
        nh.advertiseService("change_rate",&changeRate);

    ros::ServiceServer server_toggle_turtle =
        nh.advertiseService("StopRun",&StopandRun);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(
        "turtle1/cmd_vel", 1000);
    
    ros::Rate rate(2);
    while(ros::ok()){
        geometry_msgs::Twist msg;
        if (turtle_On) {
            msg.linear.x = forward ? current_speed : 0.0;
            msg.angular.z = forward ? 0.0 : current_speed;
        } else {
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
        }

        pub.publish(msg);
        ros::spinOnce();
        if (ratechanged) {
            rate = ros::Rate(newfrequency);
            ratechanged = false;
        }
        rate.sleep();
    }
}
