#include <iostream>
#include <cmath>
#include <vector>

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

double calcMinAcc(double speed, std::vector<double> throwPos,  std::vector<double> throwEndPos)
{
       double x = throwEndPos[0] - throwPos[0];
       double z = throwEndPos[2] - throwPos[2];
       double d = sqrt((x*x)+(z*z));
       return (speed*speed)/(2*d);
}

int main()
{
    ur_rtde::RTDEControlInterface rtde_control("192.168.0.26");
    ur_rtde::RTDEIOInterface rtde_IO("192.168.0.26");
    ur_rtde::RTDEReceiveInterface rtde_recv("192.168.0.26");
    std::cout << "Connected to the robot!" << std::endl;

    rtde_control.moveJ({A2R(-64),A2R(-90),A2R(-100),A2R(-45),A2R(90),0}); //Home Pos
    rtde_control.moveL({-0.017, -0.577, 0.16, 0, 0, 0}); // Pos1


    // Define points
    double x1 = -0.017, y1 = -0.577, z1 = 0.160;
    double x2 = 0.0257, y2 = -0.6803, z2 = 0.160;

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

    // Define the points in H-space
    Eigen::Vector4d P1_H(0.0, 0.0, 0, 1);
    Eigen::Vector4d P2_H(0.2, 0.0, 0, 1);
    Eigen::Vector4d P3_H(0.2, 0.2, 0, 1);
    Eigen::Vector4d P4_H(0.0, 0.2, 0, 1);

    // Apply the transformation to the points
    Eigen::Vector4d P1_B = T_BH * P1_H;
    Eigen::Vector4d P2_B = T_BH * P2_H;
    Eigen::Vector4d P3_B = T_BH * P3_H;
    Eigen::Vector4d P4_B = T_BH * P4_H;


    rtde_control.moveL({P1_B(0),P1_B(1),P1_B(2), 0, 0 ,0});
    rtde_control.moveL({P1_B(0),P1_B(1),P1_B(2), M_PI, 0 , 0});

//    //Draw square
//    for(int i = 0; i<3; ++i)
//    {
//        rtde_control.moveL({P1_B(0),P1_B(1),P1_B(2), A2R(25), A2R(-180),0});
//        rtde_control.moveL({P2_B(0),P2_B(1),P2_B(2), A2R(25), A2R(-180),0});
//        rtde_control.moveL({P3_B(0),P3_B(1),P3_B(2), A2R(25), A2R(-180),0});
//        rtde_control.moveL({P4_B(0),P4_B(1),P4_B(2), A2R(25), A2R(-180),0});
//    }

//    //Throw from current position

//    rtde_control.moveL({P4_B(0),P4_B(1),P4_B(2), A2R(25), A2R(-180),0}); //Throw Pos
//    std::vector<double> throwPos = rtde_recv.getActualTCPPose();
//    Eigen::Vector4d QThrowPosB(throwPos[0], throwPos[1],throwPos[2], 0); //Get x, y, z
//    Eigen::Vector4d QThrowPosH = T_BH_INV * QThrowPosB; //Convert to Home-frame

//    //Add add 20x20 cm to throw point in home-frame
//    QThrowPosH(0)+= 0.2;
//    QThrowPosH(2) += 0.2;

//    //Convert back to Base-frame
//    Eigen::Vector4d QThrowEndPosB = T_BH * QThrowPosH;
//    std::vector<double> throwEndPos = {QThrowEndPosB(0), QThrowEndPosB(1), QThrowEndPosB(2), throwPos[3], throwPos[4], throwPos[5]};

//    rtde_control.moveL(throwEndPos, 1, calcMinAcc(1, throwPos, throwEndPos), true); //Throw the 20x20cm with 1 m/s. Acceleration required calculated from speed and distance (throwPos&EndPos)
//    while(!rtde_control.isSteady())
//    {
//        //Check when x pos > than 50% of xthrowPath x startspos -> Release
//    }





    std::cout << "Done" << std::endl;

    return 0;
}
