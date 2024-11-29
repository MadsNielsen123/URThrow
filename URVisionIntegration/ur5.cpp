#include "ur5.h"
#include <cmath>
#include <chrono>
#include <thread>
#include <algorithm>

UR5::UR5() : mRTDE_ctrl(mIP), mRTDE_IO(mIP), mRTDE_recv(mIP)
{
    // mRTDE_ctrl.setWatchdog(0.0043); //Acknowledge time for the robot (Frequency)

    //Precice SVD (Not othogonal though)
    mT_BW << 0.3843,   0.932,  -0.0001, -0.1151,
           -0.9232,   0.3843,  0.0005, -0.3732,
            0.0005,  -0.0001,  1,      -0.0217,
            0,        0,       0,       1;

    //Less precice (But othogonal)
    mT_BW  <<  0.3827,	 0.9239,	-0.0001,	-0.1151,
              -0.9239,	 0.3827,	 0.0005,	-0.3732,
               0.0005,	-0.0001,	 1.0000,	-0.0217,
               0,         0,	     0,	     1.0000;

    mR_BW = mT_BW.topLeftCorner(3, 3);


    mT_TFTCP << 1,  0,  0, 0,
                0,  1,  0, 0,
                0,  0,  1, 0.1817,
                0,  0,  0, 1;

    mT_BW_INV = mT_BW.inverse();
    mT_TFTCP_INV = mT_TFTCP.inverse();



//    //Initialize gripper
//    mGripperSocket.connectToHost("192.168.1.20", 1000);
//    if(!mGripperSocket.waitForConnected(5000))
//    {
//        qDebug() << "Connection Failed: " << mGripperSocket.errorString();
//    }

//    gripper_home();

    moveJ({D2R(-90.92), D2R(-89.66), D2R(-134.22), D2R(-46.09), D2R(90.01), D2R( -23.47)}); //Start in default location (world 0,0,0)
}

double UR5::D2R(double degrees) const
{
    return degrees*(M_PI/180);
}
double UR5::R2D(double radians) const
{
    return (radians*180)/M_PI;
}

Eigen::Vector3d UR5::world2baseCords(Eigen::Vector3d worldCords) const
{
    // Convert 3D coordinates to 4D homogeneous coordinates
    Eigen::Vector4d worldCordsHomo;
    worldCordsHomo << worldCords, 1.0;

    // Apply the transformations
    Eigen::Vector4d baseCordsHomo = mT_BW * mT_TFTCP * worldCordsHomo;

    // Convert back to 3D by dividing by the homogeneous coordinate
    return baseCordsHomo.head<3>() / baseCordsHomo.w();
}

Eigen::Vector3d UR5::base2worldCords(Eigen::Vector3d baseCords) const
{
    Eigen::Vector4d baseCordsHomo;
    baseCordsHomo << baseCords, 1.0;
    Eigen::Vector4d worldCordsHomo = mT_BW * mT_TFTCP * baseCordsHomo;

    return worldCordsHomo.head<3>() / worldCordsHomo.w();
}

Eigen::Matrix<double, 6, 6> UR5::getJacobean(std::vector<double> jointPos) const
{
    double q1 = jointPos[0];
    double q2 = jointPos[1];
    double q3 = jointPos[2];
    double q4 = jointPos[3];
    double q5 = jointPos[4];
    double q6 = jointPos[5];

   Eigen::Matrix<double, 6, 6> J;

   J << (273*cos(q1))/2500 + (823*cos(q1)*cos(q5))/10000 + (17*cos(q2)*sin(q1))/40 - (947*cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/10000 + (947*sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))/10000 - (823*sin(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))))/10000 - (49*sin(q1)*sin(q2)*sin(q3))/125 + (49*cos(q2)*cos(q3)*sin(q1))/125, (17*cos(q1)*sin(q2))/40 + (947*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/10000 - (947*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/10000 + (823*sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))))/10000 + (49*cos(q1)*cos(q2)*sin(q3))/125 + (49*cos(q1)*cos(q3)*sin(q2))/125, (947*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/10000 - (947*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/10000 + (823*sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))))/10000 + (49*cos(q1)*cos(q2)*sin(q3))/125 + (49*cos(q1)*cos(q3)*sin(q2))/125, (947*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/10000 - (947*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/10000 + (823*sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))))/10000, - (823*sin(q1)*sin(q5))/10000 - (823*cos(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))))/10000,                                                                                                                                                     0,
        (273*sin(q1))/2500 - (17*cos(q1)*cos(q2))/40 + (823*cos(q5)*sin(q1))/10000 + (947*cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/10000 + (947*sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/10000 - (823*sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))))/10000 - (49*cos(q1)*cos(q2)*cos(q3))/125 + (49*cos(q1)*sin(q2)*sin(q3))/125, (17*sin(q1)*sin(q2))/40 - (947*cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))/10000 - (947*sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/10000 + (823*sin(q5)*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))))/10000 + (49*cos(q2)*sin(q1)*sin(q3))/125 + (49*cos(q3)*sin(q1)*sin(q2))/125, (823*sin(q5)*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))))/10000 - (947*sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/10000 - (947*cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))/10000 + (49*cos(q2)*sin(q1)*sin(q3))/125 + (49*cos(q3)*sin(q1)*sin(q2))/125, (823*sin(q5)*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))))/10000 - (947*sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/10000 - (947*cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))/10000,   (823*cos(q1)*sin(q5))/10000 + (823*cos(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))))/10000,                                                                                                                                                     0,
                                                                                                                                                                                                                                                                                                                                                                                                                                                             0,                                                                                         (49*sin(q2)*sin(q3))/125 - (49*cos(q2)*cos(q3))/125 - (17*cos(q2))/40 - (823*sin(q5)*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))))/10000 + (947*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))/10000 + (947*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))/10000,                                                                                 (49*sin(q2)*sin(q3))/125 - (49*cos(q2)*cos(q3))/125 - (823*sin(q5)*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))))/10000 + (947*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))/10000 + (947*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))/10000,                                                                 (947*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))/10000 - (823*sin(q5)*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))))/10000 + (947*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))/10000,                                                                -(823*cos(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))))/10000,                                                                                                                                                     0,
                                                                                                                                                                                                                                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                                                                                                                                                                             sin(q1),                                                                                                                                                                                                                                                                                                                                                                   sin(q1),                                                                                                                                                                                                                                                                                             sin(q1),                                                       cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)), cos(q5)*sin(q1) - sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))),
                                                                                                                                                                                                                                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                                                                                                                                                                            -cos(q1),                                                                                                                                                                                                                                                                                                                                                                  -cos(q1),                                                                                                                                                                                                                                                                                            -cos(q1),                                                       cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)), sin(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - cos(q1)*cos(q5),
                                                                                                                                                                                                                                                                                                                                                                                                                                                             1,                                                                                                                                                                                                                                                                                                                                                                                                   0,                                                                                                                                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                                                                   0,                                                                                       sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)),                                                  -sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)));

   J = J.unaryExpr([&](double val) { return std::abs(val) < (1e-15) ? 0.0 : val; }); //Round the small values (presition errors) down to 0

   return J;
}

Eigen::Matrix4d UR5::getT_World2TCP(double degreesZ, double degreesX) const
{
    Eigen::Matrix4d T_WTCP = Eigen::Matrix4d::Zero();
    Eigen::Matrix3d RotX180, RotX, RotZ;

    //Rotatate 180 around x
    RotX180 << 1,  0,  0,
               0, -1,  0,
               0,  0, -1;

    //Rotate ~Degrees around X
    RotX << 1,                  0,                   0,
               0, cos(D2R(degreesX)), -sin(D2R(degreesX)),
               0, sin(D2R(degreesX)),  cos(D2R(degreesX));

    //Rotate ~Degrees around Z
    RotZ << cos(D2R(degreesZ)), -sin(D2R(degreesZ)), 0,
            sin(D2R(degreesZ)),  cos(D2R(degreesZ)), 0,
            0,                  0,                 1;

    //Fixed angle (rot TCP-frame around stationary W-frame)
    T_WTCP.block<3, 3>(0, 0) = RotZ*RotX*RotX180; // Set top-left corner to 3x3 matrix
    T_WTCP(3, 3) = 1;
    return T_WTCP;
}

void UR5::moveJ(std::vector<double> jointPos)
{
    mRTDE_ctrl.moveJ(jointPos);
}

void UR5::moveL(double cordX, double cordY, double cordZ, double tcpAngleZ, double tcpAngleX, bool asynchonous)
{
    //Translation
    Eigen::Vector4d P_H(cordX, cordY, cordZ, 1); //Homogenous Point int world coordinates
    Eigen::Vector4d P_B = mT_BW*mT_TFTCP * P_H;              //Homogenous Point in Robot coordinates

    //Orientation
    Eigen::Matrix4d T_BTCP = mT_BW*getT_World2TCP(tcpAngleZ-90, tcpAngleX);
    Eigen::AngleAxisd angleAxis(T_BTCP.block<3,3>(0,0));
    Eigen::Vector3d axis = angleAxis.axis();

    //MoveL
    mRTDE_ctrl.moveL({P_B(0), P_B(1), P_B(2), axis(0)*angleAxis.angle(), axis(1)*angleAxis.angle(), axis(2)*angleAxis.angle()}, 0.25, 1.2, asynchonous);
}

void UR5::moveL(Eigen::Vector3d cords, double tcpAngleZ, double tcpAngleX, bool asynchonous)
{
    //Translation
    Eigen::Vector4d P_H(cords.x(), cords.y(), cords.z(), 1); //Homogenous Point int world coordinates
    Eigen::Vector4d P_B = mT_BW*mT_TFTCP * P_H;              //Homogenous Point in Robot coordinates

    //Orientation
    Eigen::Matrix4d T_BTCP = mT_BW*getT_World2TCP(tcpAngleZ-90, tcpAngleX);
    Eigen::AngleAxisd angleAxis(T_BTCP.block<3,3>(0,0));
    Eigen::Vector3d axis = angleAxis.axis();

    //MoveL
    mRTDE_ctrl.moveL({P_B(0), P_B(1), P_B(2), axis(0)*angleAxis.angle(), axis(1)*angleAxis.angle(), axis(2)*angleAxis.angle()}, 0.25, 1.2, asynchonous);

}

void UR5::gripper_home()
{
    mGripped = false;

    QByteArray response;
    QString command = "home()\n"; //5N  , 45mm size

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

void UR5::gripper_gripBall()
{
    if(mGripped)
        return;
    mGripped = true;

    QByteArray response;
    QString command = "grip()\n"; //5N  , 45mm size

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

void UR5::gripper_releaseBall(double mm)
{
    if(!mGripped)
        return;
    mGripped = false;

    if(mm > 50)
        mm = 50;

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

Eigen::VectorXd UR5::getThrowJointSpeeds(std::vector<double> jThrowPos, Eigen::Vector3d throwSpeedW)
{
    // Extract positional part of the Jacobian (3x6)
    Eigen::Matrix<double, 3, 6> positionalJacobian = getJacobean(jThrowPos).topRows(3);

    // Compute the pseudoinverse of the positional part
    Eigen::MatrixXd pseudoinverse = positionalJacobian.transpose() * (positionalJacobian * positionalJacobian.transpose()).inverse();

    // Compute joint speeds
    return pseudoinverse * (mR_BW * throwSpeedW);
}

std::pair<std::vector<double>, std::vector<double>> UR5::getBaseStartThrowPosition(Eigen::Vector3d throwCordsW, Eigen::Vector3d startCordsW)
{
    std::vector<double> bThrowPos(6), bStartPos(6);

    //Convert to base cordiantes -translation-
    Eigen::Vector3d throwCordsB = world2baseCords(throwCordsW);
    Eigen::Vector3d startCordsB = world2baseCords(startCordsW);
    bThrowPos[0] = throwCordsB(0);
    bThrowPos[1] = throwCordsB(1);
    bThrowPos[2] = throwCordsB(2);
    bStartPos[0] = startCordsB(0);
    bStartPos[1] = startCordsB(1);
    bStartPos[2] = startCordsB(2);

    //Convert to base cordiantes -orientation-
    Eigen::Vector3d pathVector = (throwCordsW-startCordsW);
    double x = pathVector.x();
    double y = pathVector.y();
    double magnitude = std::sqrt(x * x + y * y);
    double cosTheta = x / magnitude;
    cosTheta = std::max(-1.0, std::min(1.0, cosTheta)); // Clamp the value to the range [-1, 1] to handle numerical precision issues

    // Compute the angle
    double angle = std::acos(cosTheta);
    double angleDegrees = angle * (180.0 / M_PI);

    Eigen::Matrix4d T_BTCP;

    if(y>0)
        T_BTCP = mT_BW*getT_World2TCP(angleDegrees-90,45);
    else
        T_BTCP = mT_BW*getT_World2TCP(-angleDegrees-90,45);

    Eigen::AngleAxisd angleAxis(T_BTCP.block<3,3>(0,0));
    Eigen::Vector3d axis = angleAxis.axis();

    bThrowPos[3] = axis(0)*angleAxis.angle();
    bThrowPos[4] = axis(1)*angleAxis.angle();
    bThrowPos[5] = axis(2)*angleAxis.angle();


    if(y>0)
        T_BTCP = mT_BW*getT_World2TCP(angleDegrees-90,-20);
    else
        T_BTCP = mT_BW*getT_World2TCP(-angleDegrees-90,-20);

    angleAxis = T_BTCP.block<3,3>(0,0);
    axis = angleAxis.axis();

    bStartPos[3] = axis(0)*angleAxis.angle();;
    bStartPos[4] = axis(1)*angleAxis.angle();;
    bStartPos[5] = axis(2)*angleAxis.angle();;

    return std::make_pair(bThrowPos, bStartPos);
}

void UR5::throwFixed(Eigen::Vector3d throwCordsW, Eigen::Vector3d throwSpeedW, Eigen::Vector3d startCordsW)
{

    //Convert from world -> Base frame (with throw orientation)
    auto baseCartesianThrowStartPos = getBaseStartThrowPosition(throwCordsW, startCordsW); //Pair of std::vector<double>

    //Convert positions from Carteesian -> Joint Space
    std::vector<double> jThrowPos = mRTDE_ctrl.getInverseKinematics(baseCartesianThrowStartPos.first);
    std::vector<double> jStartPos = mRTDE_ctrl.getInverseKinematics(baseCartesianThrowStartPos.second);

    std::cout << "Throw: ";
    for(int i = 0; i<6; ++i)
    {
         std::cout << jThrowPos[i] << ", ";
    }
    std::cout << std::endl << "Start: ";
    for(int i = 0; i<6; ++i)
    {
        std::cout << jStartPos[i] << ", ";
    }
    std::cout << std::endl;
    //Calculate joint speeds of throw
    Eigen::VectorXd throwJointSpeeds = getThrowJointSpeeds(jThrowPos, throwSpeedW);
    std::cout << "JointSpeeds:" << std::endl;
    std::cout << throwJointSpeeds<< std::endl;

    // --------------------------------------------------------- CALCULATE ACCERLATION AND ACCLERATION START TIME  --------------------------------------------------------------------


    double T = 2;

    std::vector<double> a(6), b(6), c(6), d(6), e(6);

    for (int i = 0; i < 6; ++i)
    {
        // Create the coefficient matrix
        Eigen::MatrixXd A(5, 5);
        A <<
            0               , 0               , 0             , 0, 1,   // P(0) = e = qstart
            std::pow(T, 4)  , std::pow(T, 3)  , std::pow(T, 2), T, 1,   // P(T) = a*t⁴+b*t³+c*t²+d*t+e = qthrow
            0               , 0               , 0             , 1, 0,   // V(0) = d = 0
            4*std::pow(T, 3), 3*std::pow(T, 2), 2*T           , 1, 0,   // V(T) = 4*a*t³+3*b*t²+2*c*t+d
            0               , 0               , 1             , 0, 0;   // A(0) = c = 0

        // Create the right-hand side vector
        Eigen::VectorXd B(5);
        B << jStartPos[i],
             jThrowPos[i],
             0,
             throwJointSpeeds[i],
             0;

        // Solve the linear system
        Eigen::VectorXd solution = A.colPivHouseholderQr().solve(B);

//        std::cout << "sol[J" << i << "]: " << std::endl;
//        std::cout << solution << std::endl;

        // Store the coefficients
        a[i] = solution(0);
        b[i] = solution(1);
        c[i] = solution(2);
        d[i] = solution(3);
        e[i] = solution(4);

        //a = {0.0751, 0.2283, 1.1029, -2.3472, 0.1862, 0.2037};
        //b = {-0.1235, -0.7637, -0.3823, 3.2825, -0.3032, -0.1716};
    }

    // --------------------------------------------------------- THROW PART (WORKS) --------------------------------------------------------------------

    //Move to start position
    mRTDE_ctrl.moveJ(jThrowPos);
    mRTDE_ctrl.moveJ(jStartPos);

    //Ready timer
    std::chrono::system_clock::time_point currentTime;
    std::chrono::duration<double> interval(0.00799); // 8 milliseconds -> 125Hz
    double t;

    //Ready throw
    std::vector<double> jointVelocity(6);
    bool ballReleased = false;
//    QByteArray response;
//    QString command = "release(6,250)\n";
    mRTDE_ctrl.speedStop(20);


    //Start the throw
    auto startTime = std::chrono::high_resolution_clock::now();
    auto lastCommandTime = startTime;
    int count = 0;
    while(true)
    {
        currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> dur = currentTime-startTime;
        t = dur.count();

        //Send speedJ commands with 125Hz
        if(currentTime - lastCommandTime >= interval)
        {
            for(int i= 0; i<6; ++i)
            {           
                    jointVelocity[i] = 4*a[i]*(t*t*t) + 3*b[i]*(t*t);
            }

            //Set joints speeds from acceleration * time. (Acceleration limit: 10, stop time: 100ms)
            mRTDE_ctrl.speedJ({jointVelocity[0], jointVelocity[1], jointVelocity[2], jointVelocity[3], jointVelocity[4], jointVelocity[5]},40,0.0085);
            lastCommandTime = currentTime;
            count++;
        }

        //Throw 100ms before movement end (gripper delay)
        if(t >= T-0.1 && !ballReleased)
        {
//            mGripperSocket.write(command.toUtf8()); //Send throw command
//            if(!mGripperSocket.waitForReadyRead(1000))
//                qDebug() << "Couldn't read ack " << mGripperSocket.errorString();
            ballReleased = true;         

        }

        //End Throw
        if(t >= T)
        {
            break;
        }

    }


    mRTDE_ctrl.speedStop(15);
    std::cout << count << std::endl;
//    mGripperSocket.readAll(); //Empty buffer
}
















