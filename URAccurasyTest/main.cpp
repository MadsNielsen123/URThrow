#include <iostream>
#include <cmath>
#include <vector>
#include <string>

// ------------------- UR RTDE ------------------------------------------
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

// --------------------- EIGEN MATHS ------------------------------------
//sudo apt-get install libeigen3-dev
//https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
#include <Eigen/Core>       //Basic Linear Algebra (Matrix+vectors)
#include <Eigen/Geometry>   //Basic Transformation, 2D, 3D rotations
#include <Eigen/LU>         //Inverse ect


double A2R(double angle)
{
    return angle*(M_PI/180);
}

int main()
{
    std::string ip = "192.168.1.54"; //UR
    //std::string ip = "192.168.56.101"; //UR sim
    ur_rtde::RTDEControlInterface rtde_control(ip);
    rtde_control.setWatchdog(0.0043);
    ur_rtde::RTDEIOInterface rtde_IO(ip);
    ur_rtde::RTDEReceiveInterface rtde_recv(ip);

    rtde_control.moveJ({A2R(-64),A2R(-90),A2R(-100),A2R(-45),A2R(90),0}); //Home Pos

    // Define points
    double x1 = -0.07983, y1 = -0.46546, z1 = 0.160;
    double x2 = 0.07224, y2 = -0.83567, z2 = 0.160;

    // Compute ux (vector from point 1 to point 2)
    Eigen::Vector3d ux(x2 - x1, y2 - y1, z2 - z1);

    // Normalize ux
    ux.normalize();

    // Define uz as the unit vector along the z-axis
    Eigen::Vector3d uz(0, 0, 1);

    // Compute uy as the cross product of uz and ux
    Eigen::Vector3d uy = uz.cross(ux);

    // Define Q (the origin point for the transformation matrix)
    Eigen::Vector3d Q(x1, y1, z1);

    // Construct the 4x4 transformation matrix T_BH
    Eigen::Matrix4d T_BH;
    T_BH << ux(0), uy(0), uz(0), Q(0),
           ux(1), uy(1), uz(1), Q(1),
           ux(2), uy(2), uz(2), Q(2),
           0,     0,     0,     1;

    Eigen::Matrix4d T_BH_INV = T_BH.inverse();

    std::cout << T_BH << std::endl;

    /*
    for(int y = 0; y < 10; ++y)
    {
        for(int x = 0; x < 10; ++x)
        {
            Eigen::Vector4d P_H(0.05*x-0.025, 0.05*y-0.025, 0, 1);  //Point int world coordinates
            Eigen::Vector4d P_B = T_BH * P_H;          //Point in Robot coordinates
            rtde_control.moveL({P_B(0),P_B(1),P_B(2), 0, A2R(-180),0});
        }
    }
    */

    return 0;
}