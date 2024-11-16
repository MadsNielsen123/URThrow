#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <thread>
#include <chrono>
#include <vector>
#include <iomanip>
#include <iostream>

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

std::vector<double> cal_joint_start_rad(const std::vector<double>& joint_end_rad, const std::vector<double>& jvelocity, double time, double vscale) {
    std::vector<double> joint_start_rad;
    joint_start_rad.reserve(joint_end_rad.size());

    for (int i = 0; i < joint_end_rad.size(); ++i) {
        double q_s = joint_end_rad[i] - 0.5 * vscale * jvelocity[i] * time;
        joint_start_rad.push_back(q_s);
    }

    // for (int i = 0; i < joint_end_rad.size(); ++i) {
    //     double q_s = jvelocity[i] * time - joint_end_rad[i];
    //     joint_start_rad.push_back(q_s);
    // }
    return joint_start_rad;
}

double cal_TCP_speed(const double& distance){
    // double angle_deg = 45.0; // Throwing angle in degrees
    // double angle_rad = angle_deg * M_PI / 180.0; // Convert degrees to radians
    // double cos_val = std::cos(angle_rad);
    // double tan_val = std::cos(angle_rad);

    double r = distance;
    double g = 9.82;
    double y = 0.0;
    double y_0 = 0.59;

    //Assuming angle is 45 degrees
    double speed = std::sqrt((g * r * r) / (r + y_0 - y));

    //If angle if different from 45 degrees
    //double speed = std::sqrt((g * r * r) / (2 * cos_val *cos_val * (tan_val * r + y_0 - y))); //Slightly wrong...

    return speed;
}

int main(int argc, char* argv[])
{
    //***********************************************
    //**************Connects to robot****************
    //***********************************************
    RTDEControlInterface rtde_control("192.168.8.103");
    RTDEReceiveInterface rtde_receive("192.168.8.103");




    //***********************************************
    //************Calculate speed required***********
    //***********************************************
    double distance = 0.55;

    double tcp_initial_velocity = cal_TCP_speed(distance); //Parameter is the distance to target!!!
    std::cout << "Initial speed required: " << tcp_initial_velocity << " m/s" << std::endl;

    //Required speed to throw x-meters
    double tcp_speed_x_y = std::sqrt(2)/2 * tcp_initial_velocity;
    double scale = tcp_speed_x_y;

    std::cout << "Velocity [m/s] along x- and z-axis from releasing point. Scale: " << scale << std::endl;





    //***********************************************
    //*******************THROWING********************
    //***********************************************

    // Parameters
    double acceleration = 40;
    double dt = 1.0/125; // 8ms
    double time = 0.4;
    std::vector<double> joint_speed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> cal_joint_speed = {0, -2.5625, 5.2097, -2.6473, 0, 0}; //Used to calculate initial position. Back track

    std::vector<double> joint_end_rad   = {0, -1.60197, -1.4542, -0.8713, 1.571, 0}; //Position where ball is released
    std::vector<double> joint_start_rad = cal_joint_start_rad(joint_end_rad, cal_joint_speed, time, scale);//Initial position is calculated

    if(distance > 0.55){ //joint_start_rad[1] > -1.017, singularity
        std::cout << "Distance is too far!!!" << std::endl;
        rtde_control.stopScript();
    }

    // Move to initial joint position with moveJ
    rtde_control.moveJ(joint_start_rad);

    std::vector<double> actual_joint_start_rad = rtde_receive.getActualQ();
    std::cout << "\nActual Start joint angles[rad]:";
    for (const auto &angle : actual_joint_start_rad)
    {
        std::cout << angle << " ";
    }
    std::cout << std::endl;

    auto start = high_resolution_clock::now();

    // Execute 125Hz control loop for 0.4 seconds, each cycle is ~8ms
    //From 0.1m to 0.25m the precision of q_end decreases, but from 0.3m to 0.55m the precision increases. Most exact at around 0.1m and 0.55m. i<50 has to be adjusted
    for (unsigned int i=0; i<50; i++)
    {
        steady_clock::time_point t_start = rtde_control.initPeriod();
        //t=0.4 => /50 -> 61 iterationer

        //speed=q⁰/(f*t), f: frequency(125Hz), t: 0.4s, speed from matlab using joint_end_rad={0, -1.6089, -1.4417, -0.8765, 1.571, 0}
        //Used if speed in [x, y, z] = {1, 0, 1} - Throw in one direction
        joint_speed[1] -= scale * 2.5625/50;
        joint_speed[2] += scale * 5.2097/50;
        joint_speed[3] -= scale * 2.6473/50;

        //Used if speed in [x, y, z] = {1, 1, 1} - Throw in two directions. Note i<50 might need to be in intervals!!!
        // joint_speed[0] += scale * 2.1240/50;
        // joint_speed[1] -= scale * 2.0150/50;
        // joint_speed[2] += scale * 4.6437/50;
        // joint_speed[3] -= scale * 2.6283/50;

        rtde_control.speedJ(joint_speed, acceleration, dt);
        rtde_control.waitPeriod(t_start);
    }
    auto stop = high_resolution_clock::now();

    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000)));
    // rtde_control.stopJ(2.0); //"Another thread is already controlling the robot"....
    rtde_control.speedStop();   //Stops robot instantly. If this is placed right before stopScript the for-loop for speedJ, the end positions are different

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

    rtde_control.stopScript();

    return 0;
}
