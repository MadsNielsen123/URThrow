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
    Eigen::Vector3d world2baseCords(Eigen::Vector3d worldCords, double tilt, double rotation) const;

    //Jacobean
    Eigen::Matrix<double, 6, 6> getJacobean(std::vector<double> jointPos) const;

    void moveL(double cordX, double cordY, double cordZ, double tcpAngleZ, double tcpAngleX, bool asynchonous = false);
    void moveL(Eigen::Vector3d cords, double tcpAngleZ, double tcpAngleX, bool asynchonous = false);
    void gripper_home();
    void moveJ(std::vector<double> jointPos);
    void gripper_gripBall();
    void gripper_releaseBall(double mm = 4);

    //Parameters: worldThrowPos, throwSpeed (from that pos) & T (Time to accelerate) ... *Method backtracks throw*
    void throwFixed(Eigen::Vector3d throwCordsW, Eigen::Vector3d throwSpeedW, Eigen::Vector3d startCordsW);

private:
    // ---------- UR Connection/Control -----------
    //std::string mIP = "192.168.1.54"; //UR
    std::string mIP = "192.168.56.101"; //UR sim

    ur_rtde::RTDEControlInterface mRTDE_ctrl;
    ur_rtde::RTDEIOInterface mRTDE_IO;
    ur_rtde::RTDEReceiveInterface mRTDE_recv;

    // ---------- Transformation/Rotation matrices -----------
    Eigen::Matrix4d mT_BW, mT_BW_INV;
    Eigen::Matrix3d mR_BW;
    Eigen::Matrix4d mT_TFTCP, mT_TFTCP_INV;
    Eigen::Matrix4d getT_World2TCP(double degreesZ, double degreesX) const;



    // ---------- Gripper Connection/Control -----------
    QTcpSocket mGripperSocket;
    bool mGripped = false;

    //Throw internal methods:
    Eigen::VectorXd getThrowJointSpeeds(std::vector<double> jThrowPos, Eigen::Vector3d throwSpeedW);
    std::pair<std::vector<double>, std::vector<double>> getBaseStartThrowPosition(Eigen::Vector3d throwCordsW, Eigen::Vector3d startCordsW, Eigen::Vector3d speedVec);

};

#endif // UR5_H
