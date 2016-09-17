#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>

int main()
{
    cv::Mat img;
    cv::VideoCapture *camera = new cv::VideoCapture(0);

    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters detectorParams;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;

    while(true)
    {
        camera->read(img);

        cv::aruco::detectMarkers(img, dictionary, corners, ids, detectorParams, rejected);
        cv::aruco::drawDetectedMarkers(img, corners, ids);

        cv::imshow("crazylander", img);
        if(cv::waitKey(30) >= 0)
            break;
    }

    return 0;
}
