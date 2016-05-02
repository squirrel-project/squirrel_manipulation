#include <uibk_arm_controller/arm_controller.hpp>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

using namespace std;
using namespace uibk_arm_controller;

int main() {

    Arm robotinoArm({1, 2, 3, 4, 5}, "/dev/ttyArm",
        {std::make_pair<int, int>(-125000, 130000),
        std::make_pair<int, int>(-140000, 185000),
        std::make_pair<int, int>(-150000, 150000),
        std::make_pair<int, int>(-100000, 100000),
        std::make_pair<int, int>(-140000, 140000)}, 2.0, 3000000);

    robotinoArm.initialize();
    auto armThread = robotinoArm.runArm();

    std::cout << "moving to home position" << std::endl;
    robotinoArm.moveHome();

    std::cout << "moving to custom position" << std::endl;
    robotinoArm.jointPtp({25000, -25000, 25000, -25000, -25000});

    std::cout << "moving to home position" << std::endl;
    robotinoArm.moveHome();

    std::cout << "pressk key to stop arm" << std::endl;
    getchar();
    robotinoArm.shutdown();

    return 0;

}
