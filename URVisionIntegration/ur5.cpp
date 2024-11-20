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

    mR_BW = mT_BW.topLeftCorner(3, 3);


    mT_TFTCP << 1,  0,  0, 0,
                0,  1,  0, 0,
                0,  0,  1, 0.1817,
                0,  0,  0, 1;

    mT_BW_INV = mT_BW.inverse();
    mT_TFTCP_INV = mT_TFTCP.inverse();



    //Initialize gripper
    mGripperSocket.connectToHost("192.168.1.20", 1000);
    if(!mGripperSocket.waitForConnected(5000))
    {
        qDebug() << "Connection Failed: " << mGripperSocket.errorString();
    }

    gripper_home();

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

void UR5::moveL(double wX, double wY, double wZ, double tcpAngleZ, double tcpAngleX, bool asynchonous)
{
    //Translation
    Eigen::Vector4d P_H(wX, wY, wZ, 1);         //Point int world coordinates
    Eigen::Vector4d P_B = mT_BW*mT_TFTCP * P_H; //Point in Robot coordinates

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

void UR5::throwFixed(Eigen::Vector3d throwCordsW, Eigen::Vector3d throwSpeedW, Eigen::Vector3d startCordsW)
{

    std::cout << "Throwspeed: \n" << throwSpeedW << std::endl;

    //Convert to base cordiantes -translation-
    Eigen::Vector3d throwCordsB = world2baseCords(throwCordsW);
    Eigen::Vector3d startCordsB = world2baseCords(startCordsW);
    std::vector<double> bThrowPos = {throwCordsB(0), throwCordsB(1), throwCordsB(2)};
    std::vector<double> bStartPos = {startCordsB(0), startCordsB(1), startCordsB(2)};

    //Convert to base cordiantes -orientation-
    Eigen::Vector3d pathVector = (throwCordsW-startCordsW);

    // Extract the x and y components of the vector
    double x = pathVector.x();
    double y = pathVector.y();

    // Compute the magnitude of the vector in the x/y plane
    double magnitude = std::sqrt(x * x + y * y);

    // Compute the cosine of the angle with respect to the x-axis
    double cosTheta = x / magnitude;


    // Clamp the value to the range [-1, 1] to handle numerical precision issues
    cosTheta = std::max(-1.0, std::min(1.0, cosTheta));

    // Compute the angle in radians
    double angle = std::acos(cosTheta);

    // Convert to degrees if needed (optional)
    double angleDegrees = angle * (180.0 / M_PI);
    Eigen::Matrix4d T_BTCP;
    if(y>0)
        T_BTCP = mT_BW*getT_World2TCP(angleDegrees-90,45);
    else
        T_BTCP = mT_BW*getT_World2TCP(-angleDegrees-90,45);

    Eigen::AngleAxisd angleAxis(T_BTCP.block<3,3>(0,0));
    Eigen::Vector3d axis = angleAxis.axis();
    bThrowPos.push_back(axis(0)*angleAxis.angle()); bThrowPos.push_back(axis(1)*angleAxis.angle()); bThrowPos.push_back(axis(2)*angleAxis.angle());
    bStartPos.push_back(axis(0)*angleAxis.angle()); bStartPos.push_back(axis(1)*angleAxis.angle()); bStartPos.push_back(axis(2)*angleAxis.angle());

    //Calculate joint positions
    std::vector<double> jThrowPos, jStartPos;
    jThrowPos = mRTDE_ctrl.getInverseKinematics(bThrowPos);
    jStartPos = mRTDE_ctrl.getInverseKinematics(bStartPos);


    // Extract positional part of the Jacobian (3x6)
    Eigen::Matrix<double, 3, 6> positionalJacobian = getJacobean(jThrowPos).topRows(3);

    // Compute the pseudoinverse of the positional part
    Eigen::MatrixXd pseudoinverse = positionalJacobian.transpose() * (positionalJacobian * positionalJacobian.transpose()).inverse();

    // Compute joint speeds
    Eigen::VectorXd throwJointSpeeds = pseudoinverse * (mR_BW * throwSpeedW);

    std::cout << "speeds:\n" << throwJointSpeeds << std::endl;
    mRTDE_ctrl.moveJ(jThrowPos);
    mRTDE_ctrl.moveJ(jStartPos);

    double T = 0;

    //Calculate smallest T (from slowest link)
    for(int i = 0; i<6; ++i)
    {   if(throwJointSpeeds[i] != 0)
        {            double TJ = std::abs((2*(jThrowPos[i]-jStartPos[i]))/throwJointSpeeds[i]); //Minimum time T to reach joint speed for the throw (with linear acceleration and the exact joint-distance)

            //Save the largest
            if(TJ > T)
                T = TJ;
        }
    }

    T = 0.4;
    std::cout << "T: " << T << std::endl;

    //Calcualte accelrations
    Eigen::VectorXd jointAccelerations(6);
    for(int i = 0; i<6; ++i)
    {
        jointAccelerations[i] = ((2*(jThrowPos[i]-jStartPos[i]))/(T*T));
    }
    std::cout << "accelerations:\n" << jointAccelerations << std::endl;
    //Move to start position
    mRTDE_ctrl.moveJ(jThrowPos);
    mRTDE_ctrl.moveJ(jStartPos);

    //Start the throw
    std::chrono::system_clock::time_point currentTime;
    std::chrono::duration<double> interval(0.008); // 8 milliseconds -> 125Hz

    auto startTime = std::chrono::high_resolution_clock::now();
    auto lastCommandTime = startTime;
    while(true)
    {
        currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> t = currentTime-startTime;

        //Send speedJ commands with 125Hz
        if(currentTime - lastCommandTime >= interval)
        {
            //Set joints speeds from acceleration * time. (Acceleration limit: 10, stop time: 100ms)
            mRTDE_ctrl.speedJ({jointAccelerations[0]*t.count(), jointAccelerations[1]*t.count(), jointAccelerations[2]*t.count(), jointAccelerations[3]*t.count(), jointAccelerations[4]*t.count(), jointAccelerations[5]*t.count()},10,0.1);
            lastCommandTime = currentTime;

        }

        //End Throw
        if(t.count() >= T)
            break;
    }
    //Throw here (or a little earlier)
    mRTDE_ctrl.speedStop(5);
}
















