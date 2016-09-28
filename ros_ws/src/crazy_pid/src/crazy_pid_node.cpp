#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include <std_msgs/Float32MultiArray.h>
#include <sstream>

#include <cmath>
#include <PID.h>

PID pid_x(0.1,0.1,0), pid_y(0.1,0.1,0), pid_z(0.1,0.1,0);
ros::Publisher command_pub;

void arucoPosCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{

    sensor_msgs::Joy joy_msg;
	joy_msg.axes.resize(5);

    joy_msg.axes[0] = 0.0;

    float cons_x = 0.0;
    float cons_y = 0.0;
    float cons_z = 100.0;


    float dir_z = cons_z - msg->data[2];

    pid_z.process(dir_z, joy_msg.axes[1]);

    float dir_x_cam, dir_y_cam;
    dir_x_cam = cons_x - msg->data[0];
    dir_y_cam = cons_y - msg->data[1];
 

    if(std::abs(msg->data[3]) < 0.001)
    {
        //TODO display error
        return;
    }

    float alpha = std::atan(msg->data[4] / msg->data[3]);
    float dir_x, dir_y;
    dir_x = dir_x_cam * std::cos(alpha);
    dir_y = dir_y_cam * std::sin(alpha);

    pid_x.process(dir_x, joy_msg.axes[3]);
    pid_y.process(dir_y, joy_msg.axes[4]);

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
