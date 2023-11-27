#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <message_tests/Changerate.h>
#include <message_tests/StopStart.h>
#include <message_tests/ChangeSpeed.h>

bool forward = true;
double newfrequency;
bool ratechanged = false;
bool turtle_On = true;
double current_speed = 1.0;

ros::ServiceClient toggle_forward;
ros::ServiceClient Speed;

ros::Publisher My_turtle;

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

bool changeSpeed(
    message_tests::ChangeSpeed::Request &req,
    message_tests::ChangeSpeed::Response &resp){
        current_speed = req.new_speed;
        ROS_INFO_STREAM("Changing speed to "<<current_speed);
        resp.success = true; 
        return true;
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    My_turtle.publish(*msg);
}


int main(int argc, char **argv){
    ros::init(argc,argv,"improved_pubvel_toggle");
    ros::NodeHandle nh;

    toggle_forward = nh.serviceClient<std_srvs::Empty>("/toggle_forward");
    Speed = nh.serviceClient<message_tests::ChangeSpeed>("/change_speed");
        
    ros::ServiceServer server = 
        nh.advertiseService("toggle_forward",&toggleForward);

    ros::ServiceServer server0 =
        nh.advertiseService("change_rate",&changeRate);

    ros::ServiceServer server1 =
        nh.advertiseService("StopRun",&StopandRun);

    ros::ServiceServer server2 =
        nh.advertiseService("Change_speed",&changeSpeed);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>( "turtle1/cmd_vel", 1000);

    My_turtle = nh.advertise<geometry_msgs::Twist>("Leo/cmd_vel", 1000);
    
    
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
