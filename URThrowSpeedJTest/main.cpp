﻿#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <chrono>
#include <thread>

#include "ur5.h"
#include <cameraFunctions.h>

Eigen::Vector3d calSpeed(Eigen::Vector3d throwCords, Eigen::Vector3d targetCords)
{
    double range = std::sqrt(pow(targetCords.x()-throwCords.x(), 2) + pow(targetCords.y()-throwCords.y(), 2));
    double g = 9.81584;
    double v = std::sqrt((g*pow(range, 2)) / (throwCords.z()-targetCords.z()+range));
    double v_to_vx_angle = std::atan((targetCords.y()-throwCords.y()) / (targetCords.x()-throwCords.x()));
    double tcp_speed_x = v * std::sqrt(2)/2 * std::cos(v_to_vx_angle);
    double tcp_speed_y = v * std::sqrt(2)/2 * std::sin(v_to_vx_angle);
    double tcp_speed_z = v * std::sqrt(2)/2;

    Eigen::Vector3d velocities = {tcp_speed_x, tcp_speed_y, tcp_speed_z};
    return velocities;
}

int main()
{

    UR5 UR;

//    //Find coordinates of balls and cups
//    cv::Mat picture = takePicture();
//    cv::imshow("Picture", picture);
//    //cv::waitKey(0);
//    std::vector<Eigen::Vector3d> ballCords = findBalls(picture);


//    if(ballCords.empty()) //Exit if no ball found
//    {
//        std::cout << "NO BALL" << std::endl;
//        return 1;
//    }



//    UR.moveL(ballCords[0].x(), ballCords[0].y(), 0.05, 0, 0); //Move over ball

//    //Grab ball
//    UR.moveL(ballCords[0], 0, 0);
//    UR.gripper_gripBall();

//    std::vector<Eigen::Vector3d> cupsCords = findCups(picture);
//    if(cupsCords.empty()) //Exit if no ball found
//    {
//        std::cout << "NO CUP" << std::endl;
//        return 1;
//    }

    // ---------------- Shoot at first cup -------------


    Eigen::Vector3d targetCords = {0.71, 0.40, 0.11};    //Practice cords
    //Eigen::Vector3d targetCords = cupsCords[0];       //First Cup coords
    Eigen::Vector3d throwCords = { 0.35, 0.25, 0.3};
    Eigen::Vector3d startAccCords = {0.1, 0.2, 0.15};    //These coordinates doesn't matter much. (Only if joint 5 is moving too much)


    //Calculate speed from distance to cup
    Eigen::Vector3d  throwSpeed = calSpeed(throwCords, targetCords);
    std::cout << throwSpeed << std::endl;

    //Throw ball
    UR.throwFixed(throwCords, throwSpeed, startAccCords);
    return 0;
}


