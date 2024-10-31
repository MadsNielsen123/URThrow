#include "ur5.h"
#include <cmath>

UR5::UR5() : mRTDE_ctrl(mIP), mRTDE_IO(mIP), mRTDE_recv(mIP)
{
    // mRTDE_ctrl.setWatchdog(0.0043); //Acknowledge time for the robot (Frequency)

    mT_BW << 0.3843,   0.932,  -0.0001, -0.1151,
           -0.9232,   0.3843,  0.0005, -0.3732,
            0.0005,  -0.0001,  1,      -0.0217,
            0,        0,       0,       1;

    mT_BW_INV = mT_BW.inverse();

    mT_TFTCP << 1,  0,  0, 0,
                0,  1,  1, 0,
                0,  0,  1, 0.1817,
                0,  0,  0, 1;

    mT_TFTCP_INV = mT_TFTCP.inverse();

    /*
     * OLD CODE (SAVE FOR NOW)
    // ---------------------------------- INIT -------------------------------------

    //Calculate angle from rotation matrix ect.
    Eigen::Vector3d x(1, 0, 0);
    Eigen::Vector3d ux(mT_BH(0,0), mT_BH(1,0), mT_BH(2,0));
    double initAngle = acos(x.dot(ux));

    std::vector<double> currentJointPos, targetJointPos;

    //Move to home position
    Eigen::Vector4d P_H(0.0, 0.2, 0.2, 1);  //Point int world coordinates
    Eigen::Vector4d P_B = mT_BH * P_H;          //Point in Robot coordinates
    mRTDE_ctrl.moveL({P_B(0), P_B(1), P_B(2), 0, A2R(-180), 0});

    // --------------------------------- Make new orientation for TCP --------------------

    //Add angle-90degrees to tool-flange-joint & move
    double targetAngle = A2R(0); //x decrees
    currentJointPos = mRTDE_recv.getActualQ();
    targetJointPos = currentJointPos;
    targetJointPos[5] = targetJointPos[5]+initAngle-M_PI/2+targetAngle;
    mRTDE_ctrl.moveJ(targetJointPos);

    //Use tcpOri to orientate tcp
    Eigen::Vector3d tcpOri;
    tcpOri(0) = mRTDE_recv.getActualTCPPose()[3];
    tcpOri(1) = mRTDE_recv.getActualTCPPose()[4];
    tcpOri(2) = mRTDE_recv.getActualTCPPose()[5];
    */

}

double UR5::A2R(double degrees) const
{
    return degrees*(M_PI/180);
}
double UR5::R2A(double radians) const
{
    return (radians*180)/M_PI;
}

Eigen::Matrix4d UR5::getT_World2TCP(double degrees) const
{
    Eigen::Matrix4d mT_WTCP = Eigen::Matrix4d::Zero();
    Eigen::Matrix3d RotX180, RotZ;

    RotX180 << 1,  0,  0,
               0, -1,  0,
               0,  0, -1;

    RotZ << cos(A2R(degrees)), -sin(A2R(degrees)), 0,
            sin(A2R(degrees)),  cos(A2R(degrees)), 0,
            0,                  0,                 1;

    //Fixed angle (rot TCP-frame around stationary W-frame) -> Rotate  180° around x, then ~° around z
    mT_WTCP.block<3, 3>(0, 0) = RotZ*RotX180; // Set top-left corner to 3x3 matrix
    mT_WTCP(3, 3) = 1;

    return mT_WTCP;
}

void UR5::moveL(double wX, double wY, double wZ, double tcpAngle, bool asynchonous)
{
    Eigen::Vector4d P_H(wX, wY, wZ, 1);         //Point int world coordinates
    Eigen::Vector4d P_B = mT_BW*mT_TFTCP * P_H; //Point in Robot coordinates

    Eigen::Vector3d rotXYZ = getAngleBetweenFrames(mT_BW * getT_World2TCP(tcpAngle));

    std::cout << rotXYZ << std::endl;

    //mRTDE_ctrl.moveL({P_B(0), P_B(1), P_B(2), rotXYZ(0), rotXYZ(1), rotXYZ(2)}, 0.25, 1.2, asynchonous);
    mRTDE_ctrl.moveL({P_B(0), P_B(1), P_B(2), 0, A2R(-180), 0}, 0.25, 1.2, asynchonous); //Default TCP orientation for now
}

template <typename T>
T clamp(T value, T min, T max) {
    return (value < min) ? min : (value > max ? max : value);
}


Eigen::Vector3d UR5::getAngleBetweenFrames(Eigen::Matrix4d transformationMatrix) const
{
    Eigen::Vector3d rotXYZ;
    Eigen::Vector4d ux, uy, uz; //Homogenous
    ux << 1,0,0,1;
    uy << 0,1,0,1;
    uz << 0,0,1,1;

    rotXYZ(0) = acos(clamp(ux.dot(transformationMatrix.col(0)), -1.0, 1.0));
    rotXYZ(1) = acos(clamp(uy.dot(transformationMatrix.col(1)), -1.0, 1.0));
    rotXYZ(2) = acos(clamp(uz.dot(transformationMatrix.col(2)), -1.0, 1.0));

    return rotXYZ;
}





















