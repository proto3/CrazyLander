#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include <std_msgs/Float32MultiArray.h>
#include <sstream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <PID.h>

PID pid_x(0.1,0.1,0), pid_y(0.1,0.1,0), pid_z(0.1,0.1,0);
ros::Publisher command_pub;

void arucoPosCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{

    sensor_msgs::Joy joy_msg;
	joy_msg.axes.resize(5);

    joy_msg.axes[0] = 0.0;


	ROS_INFO("msg %lf / %lf / %lf / %lf ", msg->data[0], msg->data[1], msg->data[2], msg->data[3]);

	Eigen::Vector3f pos_cam(msg->data[0], msg->data[1], msg->data[2]);
	Eigen::Vector3f cons(0.0, 0.0, 1.0);

	Eigen::Vector3f dir_cam = cons - pos_cam;

	Eigen::AngleAxis<float> rot(msg->data[3] * 180 / M_PI, Eigen::Vector3f(0,0,1));

	Eigen::Vector3f dir_crazy = rot * dir_cam;


    pid_x.process(dir_crazy[0], joy_msg.axes[3]);
    pid_y.process(dir_crazy[1], joy_msg.axes[4]);
	pid_z.process(dir_crazy[2], joy_msg.axes[1]);

    command_pub.publish(joy_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "crazy_pid_node");
    ros::NodeHandle n;

    command_pub = n.advertise<sensor_msgs::Joy>("/autopilot_joy", 1);
    ros::Subscriber aruco_pos_sub = n.subscribe("aruco_pos", 1, arucoPosCallback);
    ros::Rate loop_rate(10);

    ros::spin();
    return 0;
}
