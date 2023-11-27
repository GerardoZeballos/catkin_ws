#include <ros/ros.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <message_tests/ChangeSpeed.h>

bool forward = true;
double current_speed = 1.0;

bool toggleForward(
	std_srvs::Empty::Request &req,
	std_srvs::Empty::Response &resp){
        forward = !forward;
        ROS_INFO_STREAM("Now sending "<<(forward?
                "forward":"rotate")<< " commands.");
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

int main(int argc, char **argv){

    ros::init(argc, argv, "spawn_turtle_plus");
    ros::NodeHandle nh;

    ros::ServiceClient spawnClient
		= nh.serviceClient<turtlesim::Spawn>("spawn");

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

    ros::ServiceServer server = nh.advertiseService("toggle_forward",&toggleForward);
     ros::ServiceServer server2 = nh.advertiseService("Change_speed",&changeSpeed);
               
    ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1000);
    ros::Publisher pub2=nh.advertise<geometry_msgs::Twist>("MyTurtle/cmd_vel",1000);
    
    
    ros::Rate rate(2);
	
	while(ros::ok()){
		geometry_msgs::Twist msg;
             msg.linear.x = forward ? current_speed : 0.0;
             msg.angular.z = forward ? 0.0 : current_speed;
		pub.publish(msg);
		ros::spinOnce();
		rate.sleep();
	}
}
