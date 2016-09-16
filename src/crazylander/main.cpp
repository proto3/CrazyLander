#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>

int main()
{
    cv::Mat markerImage;
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    while(true)
    {
        cv::aruco::drawMarker(dictionary, 18, 500, markerImage, 1);
        cv::imshow("marker", markerImage);

        if(cv::waitKey(30) >= 0)
            exit(0);

        usleep(200);
    }
}
