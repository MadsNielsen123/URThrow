#include "ur5.h"
#include <cmath>
#include <chrono>
#include <thread>

UR5::UR5() : mRTDE_ctrl(mIP), mRTDE_IO(mIP), mRTDE_recv(mIP)
{
    // mRTDE_ctrl.setWatchdog(0.0043); //Acknowledge time for the robot (Frequency)

    mT_BW << 0.3843,   0.932,  -0.0001, -0.1151,
           -0.9232,   0.3843,  0.0005, -0.3732,
            0.0005,  -0.0001,  1,      -0.0217,
            0,        0,       0,       1;



    mT_TFTCP << 1,  0,  0, 0,
                0,  1,  0, 0,
                0,  0,  1, 0.1817,
                0,  0,  0, 1;

    mT_BW_INV = mT_BW.inverse();
    mT_TFTCP_INV = mT_TFTCP.inverse();

    //Initialize/Home gripper
    mGripperSocket.connectToHost("192.168.1.20", 1000);
    if(!mGripperSocket.waitForConnected(5000))
    {
        qDebug() << "Connection Failed: " << mGripperSocket.errorString();
    }

    QByteArray response;
    QString command = "home()\n";
    mGripperSocket.write(command.toUtf8()); //Send

    if(!mGripperSocket.waitForBytesWritten(3000)) //If hasn't been send in 3s -> failed
    {
        qDebug() << "Failed to initialize gripper: " << mGripperSocket.errorString();
    }

    if(!mGripperSocket.waitForReadyRead(3000))
            qDebug() << "Command not received: " << mGripperSocket.errorString();

    response = mGripperSocket.readLine(); //Command ACK

    if(!mGripperSocket.waitForReadyRead(5000))
        qDebug() << "Command not done " << mGripperSocket.errorString();

    response = mGripperSocket.readLine(); //Command Finished

    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 0.5-second delay
}

double UR5::D2R(double degrees) const
{
    return degrees*(M_PI/180);
}
double UR5::R2D(double radians) const
{
    return (radians*180)/M_PI;
}

Eigen::Matrix4d UR5::getT_World2TCP(double degrees) const
{
    Eigen::Matrix4d T_WTCP = Eigen::Matrix4d::Zero();
    Eigen::Matrix3d RotX180, RotZ;

    //Rotatate 180 around x
    RotX180 << 1,  0,  0,
               0, -1,  0,
               0,  0, -1;

    //Rotate ~Degrees around Z
    RotZ << cos(D2R(degrees)), -sin(D2R(degrees)), 0,
            sin(D2R(degrees)),  cos(D2R(degrees)), 0,
            0,                  0,                 1;

    //Fixed angle (rot TCP-frame around stationary W-frame)
    T_WTCP.block<3, 3>(0, 0) = RotZ*RotX180; // Set top-left corner to 3x3 matrix
    T_WTCP(3, 3) = 1;

    return T_WTCP;
}

void UR5::moveL(double wX, double wY, double wZ, double tcpAngle, bool asynchonous)
{
    //Translation
    Eigen::Vector4d P_H(wX, wY, wZ, 1);         //Point int world coordinates
    Eigen::Vector4d P_B = mT_BW*mT_TFTCP * P_H; //Point in Robot coordinates

    //Orientation
    Eigen::Matrix4d T_BTCP = mT_BW*getT_World2TCP(tcpAngle-90);
    Eigen::AngleAxisd angleAxis(T_BTCP.block<3,3>(0,0));
    Eigen::Vector3d axis = angleAxis.axis();

    //MoveL
    mRTDE_ctrl.moveL({P_B(0), P_B(1), P_B(2), axis(0)*angleAxis.angle(), axis(1)*angleAxis.angle(), axis(2)*angleAxis.angle()}, 0.25, 1.2, asynchonous);

}

void UR5::gripper_grip()
{
    mGripped = true;
    QByteArray response;
    QString command = "grip()\n";

    mGripperSocket.write(command.toUtf8()); //Send command

    if(!mGripperSocket.waitForBytesWritten(3000)) //If hasn't been send in 3s -> failed
        qDebug() << "Failed to send command, exiting: " << mGripperSocket.errorString();

    if(!mGripperSocket.waitForReadyRead(3000))
        qDebug() << "Command not received: " << mGripperSocket.errorString();

    response = mGripperSocket.readLine(); //Command ACK

    if(!mGripperSocket.waitForReadyRead(5000))
        qDebug() << "Command not done " << mGripperSocket.errorString();

    response = mGripperSocket.readLine(); //Command Finished
}

void UR5::gripper_release(unsigned int mm)
{
    if(!mGripped)
        return;
    mGripped = !mGripped;

    if(mm > 55)
        mm = 55;

    QByteArray response;
    QString command = "release("+QString::number(mm)+",250)\n"; //Default Quick Speed 250
    mGripperSocket.write(command.toUtf8()); //Send command

    if(!mGripperSocket.waitForBytesWritten(3000)) //If hasn't been send in 3s -> failed
        qDebug() << "Failed to send command, exiting: " << mGripperSocket.errorString();

    if(!mGripperSocket.waitForReadyRead(3000))
        qDebug() << "Command not received: " << mGripperSocket.errorString();

    response = mGripperSocket.readLine(); //Command ACK

    if(!mGripperSocket.waitForReadyRead(5000))
        qDebug() << "Command not done " << mGripperSocket.errorString();

    response = mGripperSocket.readLine(); //Command Finished
}


















