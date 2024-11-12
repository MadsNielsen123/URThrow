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

// --------------------- Gripper TCP ---------------------------------
#include <QCoreApplication>
#include <QTcpSocket>
#include <QtDebug>


class UR5
{
public:
    UR5();

    double D2R(double degrees) const;
    double R2D(double radians) const;
    Eigen::Vector3d world2baseCords(Eigen::Vector3d worldCords) const;
    Eigen::Vector3d base2worldCords(Eigen::Vector3d baseCords) const;

    //Jacobean
    Eigen::Matrix<double, 6, 6> getJacobean(std::vector<double> jointPos) const;

    void moveL(double wX, double wY, double wZ, double tcpAngle, bool asynchonous = false);
    void gripper_grip();
    void gripper_release(unsigned int mm = 10);

    //Parameters: worldThrowPos, throwSpeed (from that pos) & T (Time to accelerate) ... *Method backtracks throw*
    void throwFixed(Eigen::Vector3d throwCordsW, Eigen::Vector3d throwSpeed, Eigen::Vector3d startCordsW);

private:
    // ---------- UR Connection/Control -----------
    //std::string mIP = "192.168.1.54"; //UR
    std::string mIP = "192.168.0.26"; //UR sim

    ur_rtde::RTDEControlInterface mRTDE_ctrl;
    ur_rtde::RTDEIOInterface mRTDE_IO;
    ur_rtde::RTDEReceiveInterface mRTDE_recv;

    // ---------- 4x4 Transformation matrices -----------
    Eigen::Matrix4d mT_BW, mT_BW_INV;
    Eigen::Matrix4d mT_TFTCP, mT_TFTCP_INV;
    Eigen::Matrix4d getT_World2TCP(double degrees) const;



    // ---------- Gripper Connection/Control -----------
    QTcpSocket mGripperSocket;
    bool mGripped = false;
};

#endif // UR5_H
