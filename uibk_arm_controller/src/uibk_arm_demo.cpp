#include <uibk_arm_controller/arm_controller.hpp>

using namespace uibk_arm_controller;

int main() {
	
    Arm robotinoArm({1, 2, 3, 4, 5}, "/dev/ttyArm",
        {std::make_pair<double, double>(-125000.0 / Motor::TICKS_FOR_180_DEG * M_PI, 130000 / Motor::TICKS_FOR_180_DEG * M_PI),
        std::make_pair<double, double>(-140000 / Motor::TICKS_FOR_180_DEG * M_PI, 185000 / Motor::TICKS_FOR_180_DEG * M_PI),
        std::make_pair<double, double>(-150000 / Motor::TICKS_FOR_180_DEG * M_PI, 150000 / Motor::TICKS_FOR_180_DEG * M_PI),
        std::make_pair<double, double>(-100000 / Motor::TICKS_FOR_180_DEG * M_PI, 100000 / Motor::TICKS_FOR_180_DEG * M_PI),
        std::make_pair<double, double>(-140000 / Motor::TICKS_FOR_180_DEG * M_PI, 140000 / Motor::TICKS_FOR_180_DEG * M_PI)}, 2.0, 3000000);
	
	robotinoArm.initialize();
	auto armThread = robotinoArm.runArm();
	
	std::cout << "moving to home position" << std::endl;
	robotinoArm.moveHome();
	
	std::cout << "moving to custom position" << std::endl;
	robotinoArm.jointPtp({25000 / Motor::TICKS_FOR_180_DEG * M_PI, -25000 / Motor::TICKS_FOR_180_DEG * M_PI, 25000 / Motor::TICKS_FOR_180_DEG * M_PI, -25000 / Motor::TICKS_FOR_180_DEG * M_PI, -25000 / Motor::TICKS_FOR_180_DEG * M_PI});
	
	std::cout << "moving to home position" << std::endl;
	robotinoArm.moveHome();
	
	std::cout << "pressk key to stop arm" << std::endl;
	getchar();
	robotinoArm.shutdown();

    return 0;
    
}
