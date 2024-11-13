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

std::vector<double> radToDeg(const std::vector<double>& rad_angles) {
    const double rad_to_deg = 180.0 / M_PI;
    std::vector<double> deg_angles;
    deg_angles.reserve(rad_angles.size());

    for (double rad : rad_angles) {
        deg_angles.push_back(rad * rad_to_deg);
    }

    return deg_angles;
}

// std::vector<double> calTime(const std::vector<double>& start, const std::vector<double>& end){
//     std::vector<double> time;

//     for (double)
// }

int main(int argc, char* argv[])
{
    RTDEControlInterface rtde_control("192.168.8.100");
    RTDEReceiveInterface rtde_receive("192.168.8.100");

    // Parameters
    double acceleration = 30;
    double dt = 1.0/125; // 8ms

    std::vector<double> joint_start = {0,  -100,  -130, -130,  90, 0.0};
    std::vector<double> joint_end   = {0,   -95,   -70,  -60,  90, 0.0};

    std::vector<double> joint_start_rad = degreesToRad(joint_start);
    std::vector<double> joint_end_rad   = degreesToRad(joint_end);

    std::vector<double> joint_speed = {0, 0.6481, 3.3525, 2.7044, 0, 0};//Need new speeds



    std::cout << "Start joint angles[deg]: ";
    for (const auto &angle : joint_start)
    {
        std::cout << angle << " ";
    }
    std::cout << std::endl;

    std::cout << "Start joint angles[rad]: ";
    for (const auto &angle : joint_start_rad)
    {
        std::cout << angle << " ";
    }
    std::cout << std::endl;

    std::cout << "\nEnd joint angles[deg]: ";
    for (const auto &angle : joint_end)
    {
        std::cout << angle << " ";
    }
    std::cout << std::endl;

    std::cout << "End joint angles[rad]: ";
    for (const auto &angle : joint_end_rad)
    {
        std::cout << angle << " ";
    }
    std::cout << std::endl;




    // Move to initial joint position with a regular moveJ
    rtde_control.moveJ(joint_start_rad);

    for (unsigned int i=0; i<33; i++) //has to be 33....
    {
        steady_clock::time_point t_start = rtde_control.initPeriod();
        rtde_control.speedJ(joint_speed, acceleration, dt);
        rtde_control.waitPeriod(t_start);
    }

    //rtde_control.moveJ(joint_end_rad);

    std::vector<double> actual_joint_end_rad = rtde_receive.getActualQ();
    std::cout << "\nActual End joint angles: ";
    for (const auto &angle : actual_joint_end_rad)
    {
        std::cout << angle << " ";
    }
    std::cout << std::endl;

    rtde_control.speedStop();
    rtde_control.stopScript();

    return 0;
}
