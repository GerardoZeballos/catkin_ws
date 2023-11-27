#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <message_tests/ChangeSpeed.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

ros::ServiceClient toggle_forward_client;
ros::ServiceClient change_speed_client;

ros::Publisher pub_my_turtle;

void toggleForward() {
    std_srvs::Empty empty;
    if (toggle_forward_client.call(empty)) {
        ROS_INFO("Toggled forward");
    } else {
        ROS_ERROR("Failed to toggle forward");
    }
}

void setInitialSpeed() {
    message_tests::ChangeSpeed change_speed;
    change_speed.request.new_speed = 5.0;
    if (change_speed_client.call(change_speed)) {
        ROS_INFO("Set initial speed to 5.0");
    } else {
        ROS_ERROR("Failed to set initial speed");
    }
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    pub_my_turtle.publish(*msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "spawn_turtle_plus");
    ros::NodeHandle nh;

    //Create a client object for the spawn service. This
//needs to know the data type of the service and its name.
    ros::ServiceClient spawnClient
		= nh.serviceClient<turtlesim::Spawn>("spawn");

//Create the request and response objects.
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

    // Service clients for toggle_forward and change_speed
    toggle_forward_client = nh.serviceClient<std_srvs::Empty>("/toggle_forward");
    change_speed_client = nh.serviceClient<message_tests::ChangeSpeed>("/change_speed");

    toggleForward();
    setInitialSpeed();

    // Subscribe to "turtle1/cmd_vel" topic
    ros::Subscriber sub_cmd_vel = nh.subscribe("turtle1/cmd_vel", 1000, cmdVelCallback);

    // Publisher for "MyTurtle/cmd_vel"
    pub_my_turtle = nh.advertise<geometry_msgs::Twist>("MyTurtle/cmd_vel", 1000);

    ros::spin();

    return 0;
}