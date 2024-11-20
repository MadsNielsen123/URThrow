#include "cameraFunctions.h"

int main()
{

    cv::Mat img = takePicture();
    
    std::vector<cv::Point2f> coords = findObject(img, HOUGH, TARGET);
    std::cout << "coordinates are\n";
    for (int i = 0; i < coords.size(); i++)
    {
        std::cout << "x: " << coords[i].x << ", y: " << coords[i].y << '\n';
    }

    cv::resize(img, img, img.size() / 2, cv::INTER_LINEAR);
    cv::imshow("image", img);
    cv::waitKey(0);
    
}
