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
    std::vector<double> joint_speed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    //--------------------------------------------------------
    // Using q_s=q_f-1/2at², where a=v/t will not make the tcp stop at the end position, q_f...

    std::vector<double> joint_start_rad = {0, -0.9682, -2.7441, -0.2146, 1.571, 0}; //If using q_s=q_f-1/2at², where a=v/t
    std::vector<double> joint_end_rad   = {0, -1.6089, -1.4417, -0.8765, 1.571, 0};

    //--------------------------------------------------------

    // Move to initial joint position with a regular moveJ
    rtde_control.moveJ(joint_start_rad);

    std::vector<double> actual_joint_start_rad = rtde_receive.getActualQ();
    std::cout << "Actual Start joint angles[rad]:";
    for (const auto &angle : actual_joint_start_rad)
    {
        std::cout << angle << " ";
    }
    std::cout << std::endl;

    auto start = high_resolution_clock::now();

    // Execute 125Hz control loop for 0.5 seconds, each cycle is ~8ms
    for (unsigned int i=0; i<63; i++)
    {
        steady_clock::time_point t_start = rtde_control.initPeriod();
        rtde_control.speedJ(joint_speed, acceleration, dt);
        //t=0.5

        //speed=q⁰/(f*t), f: frequency(125Hz), t: 0.5s, speed from matlab using joint_end_rad={0, -1.6089, -1.4417, -0.8765, 1.571, 0}
        joint_speed[1] -= 2.5625/63;
        joint_speed[2] += 5.2097/63;
        joint_speed[3] -= 2.6473/63;

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
