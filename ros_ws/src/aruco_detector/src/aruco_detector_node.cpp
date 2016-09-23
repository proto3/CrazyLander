#include "ros/ros.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <geometry_msgs/Vector3.h>

#include <vector>

int main(int argc, char* argv[])
{

	ros::init(argc, argv, "aruco_detector");
    ros::NodeHandle n;
	ros::Publisher vector_pub = n.advertise<geometry_msgs::Vector3>("aruco_pos", 1000);

    cv::Mat img, imageCopy;
    cv::VideoCapture *camera = new cv::VideoCapture(1);
	cv::Mat cameraMatrix;
	cv::Mat cameraCoeff;

	double cam_tab[9] = {7.4094821037933866e+02, 0., 3.1950000000000000e+02,
                        0., 7.4094821037933866e+02, 2.3950000000000000e+02,
                        0., 0., 1.};
	double coeff[5] = {1.2666795176764284e-01, -1.0966937220820303e+00, 0., 0., 2.8112317830722389e+00};
	cameraMatrix = cv::Mat(3, 3, CV_64FC1, cam_tab);
	cameraCoeff = cv::Mat(1,5, CV_64FC1, coeff);


    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters detectorParams;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners, rejected;
	geometry_msgs::Vector3 aruco_pos;


	 std::vector< cv::Vec3d > rvecs, tvecs;


    while(true)
    {
        camera->read(img);
		img.copyTo(imageCopy);
        cv::aruco::detectMarkers(img, dictionary, corners, ids, detectorParams, rejected);

		if(ids.size() > 0){
		//ROS_INFO("img %d", ids.size());
		//ROS_INFO("corners %f", corners[0][0].x);
 			cv::aruco::estimatePoseSingleMarkers(corners, 0.04, cameraMatrix, cameraCoeff, rvecs, tvecs);
			for(int i=0; i<ids.size(); i++)
				cv::aruco::drawAxis(imageCopy, cameraMatrix, cameraCoeff, rvecs[i], tvecs[i], 0.1);
			ROS_INFO("tvecs %lf / %lf / %lf ", tvecs[0][0], tvecs[0][1], tvecs[0][2]);
			//cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
//      	  cv::aruco::drawDetectedMarkers(img, corners, ids, rvecs, tvecs);
		}
		aruco_pos.x = 65;
		aruco_pos.y = 8;
		aruco_pos.z = 7;
		vector_pub.publish(aruco_pos);
        cv::imshow("crazylander", imageCopy);
        if(cv::waitKey(30) >= 0)
            break;
    }

    return 0;
}
