#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>

cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
cv::aruco::DetectorParameters detectorParams;
std::vector<int> ids;
std::vector<std::vector<cv::Point2f> > corners, rejected;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("Image received.");

    try
    {
        cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::aruco::detectMarkers(img, dictionary, corners, ids, detectorParams, rejected);
        cv::aruco::drawDetectedMarkers(img, corners, ids);
        if(ids.size() > 0)
        {
            //publish marker position
            //std_msgs::String msg;
            //position_pub.publish(msg);
        }
        cv::imshow("aruco", img);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    ros::init(argc, argv, "aruco_detector");
    ros::NodeHandle n;

    cv::namedWindow("aruco");
    cv::startWindowThread();
    image_transport::ImageTransport it(n);
//    ros::Publisher position_pub = n.advertise<std_msgs::String>("aruco_position", 1);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
    ros::spin();
    cv::destroyWindow("aruco");
}
