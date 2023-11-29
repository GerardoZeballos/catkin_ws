#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include "urdf_tutorial/changescale.h"

double deltaPan;
double deltaTilt;
double scale;
const double degree2rad = M_PI/180;

bool changescale(urdf_tutorial::changescale::Request  &req,
         urdf_tutorial::changescale::Response &res)
{
  scale = req.s;
  ROS_INFO("Scale changed to = %f", req.s);
  return true;
}

void valuesCallback(const geometry_msgs::Twist& msg)
{
  ROS_INFO("Teleoperated delta values (in degrees): [%f, %f]", msg.linear.x * scale, msg.angular.z * scale);
  
  deltaPan = msg.linear.x * degree2rad * scale;
  deltaTilt = msg.angular.z * degree2rad * scale;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_publisher");
  ros::NodeHandle n;

  //The node subscribes to the values given by the keys in the keyboard
  ros::Subscriber sub = n.subscribe("teleop_values", 1, valuesCallback);
  
  //The node advertises the joint values of the pan-tilt
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  
  //The node provides a service to change the scale factor of the teleoperated motions
  ros::ServiceServer service = n.advertiseService("urdf_tutorial/change_scale", changescale);
  
  ros::Rate loop_rate(30);

  // message declarations
  sensor_msgs::JointState joint_state;
  joint_state.name.resize(9);
  joint_state.position.resize(9);
  double pan = 0.0;
  double tilt = 0.0;
  
  deltaPan = 0.0;
  deltaTilt = 0.0;
  scale =0.5;

  joint_state.position[0] = 0;
  joint_state.position[1] = 0;
  joint_state.position[2] = 0;
  joint_state.position[3] = 0;
  joint_state.position[4] = 0;
  joint_state.position[5] = 0;
  joint_state.position[6] = 0; 
  joint_state.position[7] = 0; 
  joint_state.position[8] = 0; 

  while (ros::ok())
  {
      //listen to the teleop_keys
      ros::spinOnce();
            
      if (pan+deltaPan < 90*degree2rad && pan+deltaPan > -90*degree2rad) pan = pan + deltaPan;
      if (tilt+deltaTilt < 45*degree2rad && tilt+deltaTilt > -45*degree2rad ) tilt = tilt + deltaTilt;

      //update joint_state
      joint_state.position[0] = 0;
      joint_state.position[1] = 0;
      joint_state.position[2] = 0;
      joint_state.position[3] = pan;
      joint_state.position[4] = 0;
      joint_state.position[5] = tilt;
      joint_state.position[6] = 0; 
      joint_state.position[7] = 0; 
      joint_state.position[8] = 0;   
      
      joint_state.header.stamp = ros::Time::now();
      joint_state.name[0] ="shoulder_pan_joint";
      joint_state.name[1] ="shoulder_pitch_joint";
      joint_state.name[2] ="elbow_roll_joint";
      joint_state.name[3] ="elbow_pitch_joint";
      joint_state.name[4] ="wrist_roll_joint";
      joint_state.name[5] ="wrist_pitch_joint";
      joint_state.name[6] ="gripper_roll_joint";
      joint_state.name[7] ="finger_joint1";
      joint_state.name[8] ="finger_joint2";

      //send the joint state 
      joint_pub.publish(joint_state);

      deltaPan=0;
      deltaTilt=0; 
      
      loop_rate.sleep();
  }
  return 0;
}



