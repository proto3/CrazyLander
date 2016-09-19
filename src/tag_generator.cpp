#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>

int main()
{
    cv::Mat markerImage;

	//cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);    
	cv::Ptr<cv::aruco::Dictionary> dictionary=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    cv::aruco::drawMarker(dictionary, 18, 500, markerImage, 1);

    cv::imwrite( "marker.jpg", markerImage);

    cv::imshow("marker", markerImage);
    cv::waitKey(0);
    return 0;
}
