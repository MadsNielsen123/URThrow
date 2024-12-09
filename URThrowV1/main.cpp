#include <iostream>
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

    //Find coordinates of balls and cups
    cv::Mat picture = takePicture();
    std::vector<Eigen::Vector3d> ballCords = findBalls(picture);
    std::vector<Eigen::Vector3d> cupsCords = findCups(picture);
    cv::imshow("Balls and cups", picture);

    if(ballCords.empty()) //Exit if no ball found
    {
        std::cout << "NO BALL" << std::endl;
        return 1;
    }

    if(cupsCords.empty()) //Exit if no ball found
    {
        std::cout << "NO CUP" << std::endl;
        return 1;
    }

    //Throw all balls
    for(int i = 0; i< std::max(ballCords.size(), cupsCords.size()); ++i)
    {
        UR.moveL(ballCords[i].x(), ballCords[i].y(), 0.05, 0, 0); //Move over ball

        //Grab ball
        UR.moveL(ballCords[i], 0, 0);
        UR.gripper_gripBall();

        Eigen::Vector3d targetCords = cupsCords[i];//First Cup coords
        int centerDirection = 1;
        if(cupsCords[i].y()>0.25)
            centerDirection = -1;

        Eigen::Vector3d throwCords = { 0.40, cupsCords[i].y()+0.05*centerDirection, 0.3};
        Eigen::Vector3d startAccCords = {0.15, 0.2, 0.15};    //These coordinates doesn't matter much. (Only if joint 5 is moving too much)

        //Calculate speed from distance to cup
        Eigen::Vector3d  throwSpeed = calSpeed(throwCords, targetCords);
        std::cout << "Distance = " << cupsCords[i].x()-throwCords.x() << std::endl;
        throwSpeed.x() *= 1.20;
        throwSpeed.y() *= 1.20;
        UR.throwFixed(throwCords, throwSpeed, startAccCords);

        sleep(1);
    }

    //Eigen::Vector3d targetCords = {0.71, 0.30, 0.11};    //Practice cords

    return 0;
}


