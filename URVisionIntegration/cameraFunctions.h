#ifndef CAMERAFUNCTIONS_H
#define CAMERAFUNCTIONS_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <pylon/PylonIncludes.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>

// --------------------- EIGEN MATHS ---------------------------------
//sudo apt-get install libeigen3-dev
//https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
#include <Eigen/Core>       //Basic Linear Algebra (Matrix+vectors)
#include <Eigen/Geometry>   //Basic Transformation, 2D, 3D rotations
#include <Eigen/LU>         //Inverse ect

cv::Mat takePicture();
std::vector<Eigen::Vector3d> findBalls(cv::Mat& image);
std::vector<Eigen::Vector3d> findCups(cv::Mat& image);

#endif // CAMERAFUNCTIONS_H
