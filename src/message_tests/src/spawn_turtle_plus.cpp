//This program spawns a new turtlesim turtle by calling
// the appropriate service.
#include <ros/ros.h>
//The srv class for the service.
#include <turtlesim/Spawn.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <message_tests/Changerate.h>
#include <message_tests/StopStart.h>
#include <message_tests/ChangeSpeed.h>

ros::ServiceClient toggle_forward;
ros::ServiceClient Speed;
ros::Publisher My_turtle;


void toggleForward() {
    std_srvs::Empty empty;
    if (toggle_forward.call(empty)) {
        ROS_INFO("Toggled forward");
    } else {
        ROS_ERROR("Failed to toggle forward");
    }
}

void setInitialSpeed() {
    message_tests::ChangeSpeed change_speed;
    change_speed.request.new_speed = 5.0;
    if (Speed.call(change_speed)) {
        ROS_INFO("Set initial speed to 5.0");
    } else {
        ROS_ERROR("Failed to set initial speed");
    }
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    My_turtle.publish(*msg);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "spawn_turtle");
    ros::NodeHandle nh;

    ros::ServiceClient toggle_Forward= nh.serviceClient<std_srvs::Empty>("/toggle_forward");  
    ros::ServiceClient Speed= nh.serviceClient<std_srvs::Empty>("/Change_speed");
    toggleForward();
    setInitialSpeed();

    ros::Subscriber sub_cmd_vel = nh.subscribe("turtle1/cmd_vel", 1000, cmdVelCallback);
    ros::Publisher My_turtle = nh.advertise<geometry_msgs::Twist>( "Myturtle/cmd_vel", 1000);

    ros::ServiceClient spawnClient= nh.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn::Request req;
    turtlesim::Spawn::Response resp;

    req.x = 2;
    req.y = 3;
    req.theta = M_PI/2;
    req.name = "Leo";

    ros::service::waitForService("spawn", ros::Duration(5));
    bool success = spawnClient.call(req,resp);

    if(success){
	ROS_INFO_STREAM("Spawned a turtle named "
			<< resp.name);
    }else{
	ROS_ERROR_STREAM("Failed to spawn.");
    }
}
