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


// Function to compute centroids of a set of points
Eigen::Vector3d computeCentroid(const std::vector<Eigen::Vector3d> &points) {
    Eigen::Vector3d centroid(0.0, 0.0, 0.0);
    for (const auto &p : points) {
        centroid += p;
    }

    centroid /= points.size();
    return centroid;
}

// Function to compute the transformation matrix (rotation and translation)
void computeTransformation(const std::vector<Eigen::Vector3d> &pointsBase,
                           const std::vector<Eigen::Vector3d> &pointsWorld,
                           Eigen::Matrix3d &R, Eigen::Vector3d &T) {

    // Step 1: Compute centroids of the points in both frames
    Eigen::Vector3d centroidBase = computeCentroid(pointsBase);
    Eigen::Vector3d centroidWorld = computeCentroid(pointsWorld);

    // Step 2: Subtract centroids from the points to get zero-mean points
    std::vector<Eigen::Vector3d> qBase, qWorld;
    for (size_t i = 0; i < pointsBase.size(); ++i) {
        qBase.push_back(pointsBase[i] - centroidBase);
        qWorld.push_back(pointsWorld[i] - centroidWorld);
    }

    // Step 3: Compute matrix H
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < qBase.size(); ++i) {
        H += qBase[i] * qWorld[i].transpose();
    }

    // Step 4: Perform Singular Value Decomposition (SVD) of H
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    /*

    std::cout << "U:\n" << U << std::endl;
    std::cout << "V:\n" << V << std::endl;
    std::cout << "U^T * U:\n" << U.transpose() * U << std::endl;
    std::cout << "V^T * V:\n" << V.transpose() * V << std::endl;
    std::cout << "Determinant of R: " << R.determinant() << std::endl;
    std::cout.precision(15);
    std::cout << "R = V * U^T:\n" << (V * U.transpose()) << std::endl;
    std::cout << "R = V * U:\n" << (V * U) << std::endl;
    */


    // Step 5: Compute rotation matrix R
    R = V * U.transpose();

    if (R.determinant() < 0) {
        R.col(2) *= -1;  // Adjust to ensure proper rotation
    }

    // Step 6: Compute translation vector T
    T = centroidWorld - R * centroidBase;
}

int main()
{
    std::string ip = "192.168.1.54"; //UR
    //std::string ip = "192.168.56.101"; //UR sim
    ur_rtde::RTDEControlInterface rtde_control(ip);
    rtde_control.setWatchdog(0.0043);
    ur_rtde::RTDEIOInterface rtde_IO(ip);
    ur_rtde::RTDEReceiveInterface rtde_recv(ip);

    //rtde_control.moveJ({A2R(-64),A2R(-90),A2R(-100),A2R(-45),A2R(90),0}); //Home Pos

    // Define points in the robot base frame
        std::vector<Eigen::Vector3d> pointsBase = {
            {-0.08, -0.46547, -0.0115},
            {0.0723, -0.8358, -0.0115},
            {0.33678, -0.29458, -0.0115}
        };

        // Define corresponding points in the world (table) frame
        std::vector<Eigen::Vector3d> pointsWorld = {
            {0.0, 0.0, -0.160},
            {0.4, 0.0, -0.160},
            {0.0, 0.45, -0.160}
        };

        // Rotation matrix R and translation vector T
        Eigen::Matrix3d R;
        Eigen::Vector3d T;

        // Compute transformation
        computeTransformation(pointsBase, pointsWorld, R, T);

    // Construct the 4x4 transformation matrix T_BH
    Eigen::Matrix4d T_BH;
    T_BH << R(0,0), R(0,1), R(0,2), T(0),
            R(1,0), R(1,1), R(1,2), T(1),
            R(2,0), R(2,1), R(2,2), T(2),
            0,      0,      0,      1;

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
