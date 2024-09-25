#include <iostream>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <cmath>


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
    ur_rtde::RTDEControlInterface rtde_control("192.168.56.101");
    ur_rtde::RTDEIOInterface rtde_IO("192.168.56.101");
    ur_rtde::RTDEReceiveInterface rtde_recv("192.168.56.101");
    std::cout << "Connected to the robot!" << std::endl;

    rtde_control.moveJ({A2R(-64),A2R(-90),A2R(-100),A2R(-45),A2R(90),0}); //Home Pos
    //rtde_IO.setSpeedSlider(0.2);

//    //Draw square
//    for(int i = 0; i<3; ++i)
//    {
//        rtde_control.moveL({0.4,0,0,A2R(180),0,0});
//        rtde_control.moveL({0.8,0,0,A2R(180),0,0});
//        rtde_control.moveL({0.8,0.4,0,A2R(180),0,0});
//        rtde_control.moveL({0.4,0.4,0,A2R(180),0,0});
//        rtde_IO.setStandardDigitalOut(3, !rtde_recv.getDigitalOutState(3)); //Toggle output 3
//    }

//    rtde_control.moveJ({A2R(180),A2R(-80),A2R(120),A2R(-200),A2R(-90),0}); //Throw Pos

//    std::vector<double> throwPos = rtde_recv.getActualTCPPose();
//    std::vector<double> throwEndPos = throwPos;
//    throwEndPos[0] += 0.2;
//    throwEndPos[2] += 0.2;
//    rtde_control.moveL(throwEndPos, 1, calcMinAcc(1, throwPos, throwEndPos), true);
//    while(!rtde_control.isSteady())
//    {
//        //Check when x pos > than 50% of xthrowPath x startspos -> Release
//    }


    //rtde_control.moveJ({A2R(50),0,0,0,0,0});
    std::cout << "Done" << std::endl;

    return 0;
}
