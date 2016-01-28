#include "../include/squirrel_arm_control/squirrelhand.hpp"

using namespace std;
using namespace arma;
using namespace kukadu;

SquirrelKclHand::SquirrelKclHand(KUKADU_SHARED_PTR<ros::NodeHandle> node) : sensorTopic("/fingertips") {

    sensorSub = node->subscribe(sensorTopic, 2, &SquirrelKclHand::sensorCallback, this);

}

void SquirrelKclHand::connectHand() {

}

void SquirrelKclHand::closeHand(double percentage, double velocity) {

}

void SquirrelKclHand::disconnectHand() {

}

std::vector<arma::mat> SquirrelKclHand::getTactileSensing() {

    vector<mat> tactileRet;

    tactileMutex.lock();

        if(tactile.n_elem == numOfFingers * tactileSensorsPerFinger) {

            for(int j = 0; j < numOfFingers; ++j) {

                mat finger(tactileSensorsPerFinger, 1);
                for(int i = 0; i < tactileSensorsPerFinger; ++i)
                    finger(i) = tactile(tactileSensorsPerFinger * j + i);

                tactileRet.push_back(finger);

            }

        }

    tactileMutex.unlock();

    return tactileRet;

}

std::string SquirrelKclHand::getHandName() {
    return string("kclhand");
}

void SquirrelKclHand::sensorCallback(std_msgs::Float64MultiArray sense) {

    tactileMutex.lock();
        tactile = stdToArmadilloVec(sense.data);
    tactileMutex.unlock();

}
