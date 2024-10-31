#ifndef UR5_H
#define UR5_H
// ------------------- UR RTDE ------------
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

// --------------------- EIGEN MATHS ---------------------------------
//sudo apt-get install libeigen3-dev
//https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
#include <Eigen/Core>       //Basic Linear Algebra (Matrix+vectors)
#include <Eigen/Geometry>   //Basic Transformation, 2D, 3D rotations
#include <Eigen/LU>         //Inverse ect


class UR5
{
public:
    UR5();
    double A2R(double degrees) const;
    double R2A(double radians) const;
    void moveL(double wX, double wY, double wZ, double tcpAngle, bool asynchonous = false);

private:
    //std::string mIP = "192.168.1.54"; //UR
    std::string mIP = "192.168.0.26"; //UR sim

    ur_rtde::RTDEControlInterface mRTDE_ctrl;
    ur_rtde::RTDEIOInterface mRTDE_IO;
    ur_rtde::RTDEReceiveInterface mRTDE_recv;

    //4x4 transformation matrixes
    Eigen::Matrix4d mT_BW, mT_BW_INV;
    Eigen::Matrix4d mT_TFTCP, mT_TFTCP_INV;
    Eigen::Matrix4d getT_World2TCP(double degrees) const;

    Eigen::Vector3d getAngleBetweenFrames(Eigen::Matrix4d transformationMatrix) const;
};

#endif // UR5_H
