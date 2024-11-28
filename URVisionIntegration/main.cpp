#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <ur5.h>
#include <fstream>
#include <chrono> // For timing
#include <thread>
#include <cameraFunctions.h>

std::vector<double> calSpeed(const double& x_s, const double& y_s, const double& z_s, const double& x_f, const double& y_f, const double& z_f){
    double range = std::sqrt(pow(x_f-x_s, 2) + pow(y_f-y_s, 2));
    double g = 9.82;

    double v = std::sqrt((g*pow(range, 2)) / (x_s-z_f+range));

    double v_to_vx_angle = std::atan((y_f-y_s) / (x_f-x_s));

    double tcp_speed_x = v * std::sqrt(2)/2 * std::cos(v_to_vx_angle);
    double tcp_speed_y = v * std::sqrt(2)/2 * std::sin(v_to_vx_angle);
    double tcp_speed_z = v * std::sqrt(2)/2;

    std::vector<double> velocities = {v, tcp_speed_x, tcp_speed_y, tcp_speed_z};;

    return velocities;
}

int main()
{

    UR5 UR;
//    cv::Mat img = takePicture();
//    std::vector<cv::Point2f> coordsBall = findObject(img, BALL);
//    std::vector<cv::Point2f> coordsCups = findObject(img, TARGET);
//    UR.moveL(coordsBall[0].x, coordsBall[0].y, 0, 0, 0);
//    UR.gripper_gripBall();
//    UR.moveL(coordsCups[0].x, coordsCups[0].y, 0.15, 0, 0);
//    UR.moveL(coordsCups[1].x, coordsCups[1].y, 0.15, 0, 0);
//    UR.moveL(coordsCups[2].x, coordsCups[2].y, 0.15, 0, 0);
//    UR.gripper_releaseBall(4);


    Eigen::Vector3d throwCordsW;
    throwCordsW << 0.35, 0.2, 0.45;

    double angle = UR.D2R(45);
    double throwSpeed = 1; // 1m/s

    Eigen::Vector3d throwSpeedVec;
    throwSpeedVec << cos(angle)*throwSpeed, 0, sin(angle)*throwSpeed;

    Eigen::Vector3d startCordsW;
    startCordsW << 0.1, 0.2, 0.1;

    UR.throwFixed(throwCordsW, throwSpeedVec, startCordsW);



    //UR.moveL(throwCordsW(0), throwCordsW(1), throwCordsW(2),0);
//    UR.moveL(0, 0, 0, 0);

//    cv::Mat img = takePicture();
//    std::vector<double> ballCoordinates = findObject(img, HOUGH, BALL);
//    std::cout << "x: " << ballCoordinates[0]*0.01 << " y:" << ballCoordinates[1]*0.01 << std::endl;

//    UR.moveL(ballCoordinates[0]*0.01, ballCoordinates[1]*0.01, 0.2, 0);
//    UR.moveL(ballCoordinates[0]*0.01, ballCoordinates[1]*0.01, 0, 0);
//    UR.gripper_grip();
//    UR.moveL(0, 0, 0.2, 0);
//    UR.gripper_release(20);

//    cv::waitKey(0);

    //X,Y,Z,TCPAngle
//    UR.moveL(0, 0, 0, 0);
//    UR.gripper_grip();
//    UR.moveL(0.2, 0.2, 0, -45);
//    UR.gripper_release(80);

    return 0;
}
