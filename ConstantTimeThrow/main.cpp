#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <thread>
#include <chrono>

using namespace ur_rtde;
using namespace std::chrono;

std::vector<double> degreesToRad(const std::vector<double>& degrees) {
    std::vector<double> radians;
    radians.reserve(degrees.size());

    for (double degree : degrees) {
        radians.push_back(degree * M_PI / 180.0);
    }

    return radians;
}

int main(int argc, char* argv[])
{
    RTDEControlInterface rtde_control("192.168.8.100");
    RTDEReceiveInterface rtde_receive("192.168.8.100");

    // Parameters
    double acceleration = 40;
    double dt = 1.0/125; // 8ms
    //std::vector<double> joint_q = {-1.54, -1.83, -2.28, -0.59, 1.60, 0.023};
    std::vector<double> joint_speed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};



    //--------------------------------------------------------
    std::vector<double> joint_start = {0,  -100,  -130, -130,  90, 0.0};
    std::vector<double> joint_end   = {0,   -95,   -70,  -60,  90, 0.0};
    std::vector<double> joint_start_rad = degreesToRad(joint_start);
    std::vector<double> joint_end_rad   = degreesToRad(joint_end);

    std::vector<double> actual_joint_start_rad = rtde_receive.getActualQ();
    std::cout << "Actual Start joint angles[rad]:";
    for (const auto &angle : actual_joint_start_rad)
    {
        std::cout << angle << " ";
    }
    std::cout << std::endl;
    //--------------------------------------------------------



    // Move to initial joint position with a regular moveJ
    rtde_control.moveJ(joint_start_rad);

    auto start = high_resolution_clock::now();

    // Execute 125Hz control loop for 0.5 seconds, each cycle is ~8ms
    for (unsigned int i=0; i<60; i++)
    {
        steady_clock::time_point t_start = rtde_control.initPeriod();
        rtde_control.speedJ(joint_speed, acceleration, dt);
        //t=0.5

        //a=q(dot)/t closest so far...
        // joint_speed[1] += 1.2962;
        // joint_speed[2] += 6.705;
        // joint_speed[3] += 5.4088;

        //a=q(dot)*0.008/t
        // joint_speed[1] += 0.0104;
        // joint_speed[2] += 0.0536;
        // joint_speed[3] += 0.0433;


        //a=2(q-q_s)/t² does not work
        // joint_speed[1] += 0.69816;
        // joint_speed[2] += 27.9253;


        //a=q(dot)²/2(q-q_s) does not work
        // joint_speed[1] += 2.4065;
        // joint_speed[2] += 5.3663;

        //a=q(dot)²/2(q-q_s) does not work
        joint_speed[1] += 0.0126;
        joint_speed[2] += 0.441;
        joint_speed[3] += 0.061;

        //t=2(q_end-q_start)/q(dot), choosing maxT=0.9 -> a=q(dot)/maxT
        //when std::vector<double> joint_end   = {0,   -95,   -70,  -60} degrees*
        // joint_speed[1] += 0.720;
        // joint_speed[2] += 3.005;
        // joint_speed[3] += 2.434;
        rtde_control.waitPeriod(t_start);
    }
    auto stop = high_resolution_clock::now();

    //rtde_control.moveJ(joint_end_rad);

    std::vector<double> actual_joint_end_rad = rtde_receive.getActualQ();
    std::cout << "Actual End joint angles[rad]:  ";
    for (const auto &angle : actual_joint_end_rad)
    {
        std::cout << angle << " ";
    }
    std::cout << std::endl;

    std::cout << "\nWanted End joint angles[rad]:  ";
    for (const auto &angle : joint_end_rad)
    {
        std::cout << angle << " ";
    }
    std::cout << std::endl;

    auto duration = duration_cast<milliseconds>(stop - start);

    std::cout << "\nTime for throwing: " << duration.count() << "ms" << std::endl;

    rtde_control.speedStop();
    rtde_control.stopScript();

    return 0;
}
