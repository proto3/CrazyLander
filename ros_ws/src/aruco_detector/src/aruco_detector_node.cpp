#include "ros/ros.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <cmath>

void getEulerAngles(cv::Mat &rotCamerMatrix,cv::Vec3d &eulerAngles){

    cv::Mat cameraMatrix,rotMatrix,transVect,rotMatrixX,rotMatrixY,rotMatrixZ;
    double* _r = rotCamerMatrix.ptr<double>();
    double projMatrix[12] = {_r[0],_r[1],_r[2],0,
                          _r[3],_r[4],_r[5],0,
                          _r[6],_r[7],_r[8],0};

    decomposeProjectionMatrix( cv::Mat(3,4,CV_64FC1,projMatrix),
                               cameraMatrix,
                               rotMatrix,
                               transVect,
                               rotMatrixX,
                               rotMatrixY,
                               rotMatrixZ,
                               eulerAngles);
}

int main(int argc, char* argv[])
{

	ros::init(argc, argv, "aruco_detector");
    ros::NodeHandle n;
	//ros::Publisher vector_pub = n.advertise<std_msgs::Float32MultiArray>("aruco_pos", 1000);
	ros::Publisher vector_pub = n.advertise<std_msgs::Float32MultiArray>("aruco_pos", 1000);

    cv::Mat img, imageCopy;
    cv::VideoCapture *camera = new cv::VideoCapture(1);
	camera->set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	camera->set(CV_CAP_PROP_FRAME_HEIGHT, 720);
//	camera->set(CV_CAP_PROP_FRAME_WIDTH, 640);
//	camera->set(CV_CAP_PROP_FRAME_HEIGHT, 480);
//	cv::Mat cameraMatrix;
//v	cv::Mat cameraCoeff;

	cv::Mat camMatrix, distCoeffs;

	std::string calibrationFile = "camera_calibration_720p.xml";
	cv::FileStorage fs(calibrationFile, cv::FileStorage::READ);
	if(!fs.isOpened())
	{
		throw std::logic_error("Unable to open \"" + calibrationFile + "\".");
	}
	else
	{
		fs["camera_matrix"] >> camMatrix;
		fs["distortion_coefficients"] >> distCoeffs;
	}

    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters detectorParams;
    std::vector<int> ids;
	int pos;
    std::vector<std::vector<cv::Point2f> > corners, rejected;
	//std_msgs::Float32MultiArray aruco_pos;
	geometry_msgs::Pose aru_pose;

	 std::vector< cv::Vec3d > rvecs, tvecs;

    while(true)
    {
        camera->read(img);
		img.copyTo(imageCopy);
        cv::aruco::detectMarkers(img, dictionary, corners, ids, detectorParams, rejected);
		pos = find(ids.begin(), ids.end(), 18) - ids.begin();
		if(pos < ids.size()){
 			cv::aruco::estimatePoseSingleMarkers(corners, 0.03, camMatrix, distCoeffs, rvecs, tvecs);
			

			cv::Vec3d rvec, tvec;
			rvec = rvecs[pos];
			tvec = tvecs[pos];
			//for(int i=0; i<ids.size(); i++)
			cv::aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec, 0.1);
			//ROS_INFO("tvecs %lf / %lf / %lf ", tvec[0], tvec[1], tvec[2]);
			//ROS_INFO("rvecs %lf / %lf / %lf ", rvec[0], rvec[1], rvec[2]);
			cv::Mat rmat;
			cv::Rodrigues(rvec, rmat); 
			cv::Vec3d eulerAngles;
			getEulerAngles(rmat,eulerAngles);

			std_msgs::Float32MultiArray aru_pose;
			aru_pose.data.clear();

			aru_pose.data.push_back(tvec[0]);
			aru_pose.data.push_back(tvec[1]);
			aru_pose.data.push_back(tvec[2]);
			aru_pose.data.push_back(eulerAngles[2]);
			vector_pub.publish(aru_pose);


            //cv::Vec3d eulerAngles;
			//getEulerAngles(rmat,eulerAngles);
			ROS_INFO("rvecs %lf / %lf / %lf ", eulerAngles[0], eulerAngles[1], eulerAngles[2]);

			cv::line(imageCopy, cv::Point(100,100), cv::Point(100+std::cos(eulerAngles[0]*3.14/180)*100, 100+std::sin(eulerAngles[0]*3.14/180)*100), cv::Scalar(0,255,0), 3);
			cv::line(imageCopy, cv::Point(100,100), cv::Point(100+std::cos(eulerAngles[1]*3.14/180)*100, 100+std::sin(eulerAngles[1]*3.14/180)*100), cv::Scalar(0,0,255), 3);
			cv::line(imageCopy, cv::Point(100,100), cv::Point(100+std::cos(eulerAngles[2]*3.14/180)*100, 100+std::sin(eulerAngles[2]*3.14/180)*100), cv::Scalar(255,0,0), 3);
			//cv::line(imageCopy, cv::Point(100,100), cv::Point(100+eulerAngles[0]*100, 100+result[1]*100), cv::Scalar(0,255,0), 3);

			/*result = drone_referential * Eigen::Vector3f(0,1,0);		
			//cv::line(imageCopy, cv::Point(100,100), cv::Point(100+std::cos(rvec[0])*100, 100+std::sin(rvec[0])*100), cv::Scalar(0,255,0), 3);
			cv::line(imageCopy, cv::Point(100,100), cv::Point(100+result[0]*100, 100+result[1]*100), cv::Scalar(0,0,255), 3);

			result = drone_referential * Eigen::Vector3f(0,0,1);		
			//cv::line(imageCopy, cv::Point(100,100), cv::Point(100+std::cos(rvec[0])*100, 100+std::sin(rvec[0])*100), cv::Scalar(0,255,0), 3);
			cv::line(imageCopy, cv::Point(100,100), cv::Point(100+result[0]*100, 100+result[1]*100), cv::Scalar(255,0,0), 3);
*/
		}
		

		
        cv::imshow("crazylander", imageCopy);
        if(cv::waitKey(30) >= 0)
            break;
    }

    return 0;
}
