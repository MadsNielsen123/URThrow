#ifndef CAMERAFUNCTIONS_H
#define CAMERAFUNCTIONS_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <pylon/PylonIncludes.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>




//method
#define COLORSEG 0
#define HOUGH 1

//object
#define BALL 0
#define TARGET 1

cv::Mat takePicture();
std::vector<cv::Point2f> findObject(cv::Mat& image, int object = 0, int method = 1,
    int minRadiusBall = 14, int maxRadiusBall = 17, int minRadiusCup = 32, int maxRadiusCup = 40, int targetHeight = 10);

#endif // CAMERAFUNCTIONS_H
