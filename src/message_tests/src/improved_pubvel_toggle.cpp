//this program toggles between rotation and translation
//commands,based on calls to a service.
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <message_tests/Changerate.h>

bool forward = true;
double newfrequency;
bool ratechanged = false;
bool StopRun = true;
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

bool Startstop(
    message_tests::ToggleTurtle::Request &req,
    message_tests::ToggleTurtle::Response &resp){
        StopRun = req.run_turtle;
        ROS_INFO_STREAM("Turtle is "<<(StopRun?
                "running":"stopped"));
        return true;
}


int main(int argc, char **argv){
        ros::init(argc,argv,"pubvel_toggle_rate");
	ros::NodeHandle nh;
        
	ros::ServiceServer server = 
		nh.advertiseService("toggle_forward",&toggleForward);
                
        ros::ServiceServer server0 =
                nh.advertiseService("change_rate",&changeRate);
        ros::ServiceServer server1 =
                nh.advertiseService("change_rate",&Startstop);
                
        ros::Publisher pub=nh.advertise<geometry_msgs::Twist>(
		"turtle1/cmd_vel",1000);
    
        ros::Rate rate(2);
	 while(ros::ok()){
        geometry_msgs::Twist msg;
        if (Startstop) {
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
