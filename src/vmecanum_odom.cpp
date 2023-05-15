#include <ros/ros.h>
#include <control_msgs/JointControllerState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <string>
#include <math.h>
//#include "mecanumrover.h"


float wheel_speed[4] = { 0 };
float wheel_circumference=0.4775f;
float w_rate=wheel_circumference/(2*M_PI);
void state0_callback(const control_msgs::JointControllerState& state_msg)
{
  wheel_speed[0] = state_msg.process_value*w_rate;
}
void state1_callback(const control_msgs::JointControllerState& state_msg)
{
  wheel_speed[1] = state_msg.process_value*w_rate;
}
void state2_callback(const control_msgs::JointControllerState& state_msg)
{
  wheel_speed[2] = state_msg.process_value*w_rate;
}
void state3_callback(const control_msgs::JointControllerState& state_msg)
{
  wheel_speed[3] = state_msg.process_value*w_rate;
}

float publish_rate = 20;

int main(int argc, char** argv)
{
  float ROVER_D=(0.266/2),ROVER_HB=(0.230/2);
  int F_L=0,F_R=1,R_L=2,R_R=3;

  ros::init(argc, argv, "s4_omni_odom");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  // publish
  ros::Publisher odom_pub = n.advertise<geometry_msgs::Twist>("rover_odo", 1);
  ros::Publisher wheel_speeds_pub = n.advertise<std_msgs::Float32MultiArray>("/rover_debug/wheel_speeds", 10);

  // Subscribe
  ros::Subscriber odometry0 = n.subscribe("wheel0/state", 1, state0_callback);
  ros::Subscriber odometry1 = n.subscribe("wheel1/state", 1, state1_callback);
  ros::Subscriber odometry2 = n.subscribe("wheel2/state", 1, state2_callback);
  ros::Subscriber odometry3 = n.subscribe("wheel3/state", 1, state3_callback);



  pn.getParam("rover_d", ROVER_D);
  pn.getParam("rover_hb", ROVER_HB);
  pn.getParam("wheel_circumference", wheel_circumference);
  pn.getParam("f_l", F_L);
  pn.getParam("f_r", F_R);
  pn.getParam("r_l", R_L);
  pn.getParam("r_r", R_R);

  w_rate=wheel_circumference/(2*M_PI);

  float dt = 1.0 / publish_rate;
  ros::Rate loop_rate(publish_rate);

  while (ros::ok())
  {

    geometry_msgs::Twist rover_odo;

    rover_odo.linear.x  = ((wheel_speed[F_R] + wheel_speed[R_R] + -1.0*(wheel_speed[F_L] + wheel_speed[R_L]))/4.0);
    rover_odo.linear.y  = ((wheel_speed[F_R] + -1.0*wheel_speed[R_R] + wheel_speed[F_L] + -1.0*wheel_speed[R_L]))/4.0;

    //V-Stone
    double dCenter2Wheel = ROVER_D + ROVER_HB;
    rover_odo.angular.z = (wheel_speed[F_L] + wheel_speed[R_L] + wheel_speed[R_R] + wheel_speed[F_R])/(4.0 * dCenter2Wheel);

    //iTB
    // double dCenter2Wheel = ROVER_D + ROVER_HB;
    // double wheel_radius = 0.37375;
    // double angular_velocity = (wheel_radius / 4) * (1 / (dCenter2Wheel)) * (-wheel_speed[F_L] + wheel_speed[F_R] - wheel_speed[R_R] + wheel_speed[R_L]);
    // rover_odo.angular.z = angular_velocity;

    odom_pub.publish(rover_odo);
    std_msgs::Float32MultiArray wheel_speeds;
    wheel_speeds.data.resize(4);
    wheel_speeds.data = {wheel_speed[F_R], wheel_speed[R_R], wheel_speed[F_L], wheel_speed[R_L]};
    wheel_speeds_pub.publish(wheel_speeds);


    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
