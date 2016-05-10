#include <string>
#include <vector>
#include <exception>
#include <iostream>
#include <sstream>
#include <mutex>
#include <memory>
#include <utility>
#include <thread>
#include <chrono>
#include <limits.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <dynamixel_sdk/DynamixelSDK.h>

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          562                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          596
#define ADDR_PRO_PRESENT_POSITION       611

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

namespace uibk_arm_controller {

class MotorException : public std::exception {
	
	private:
		const char* msg;
	public:
	
		MotorException(const char* msg);
		
		virtual const char* what() const throw();
	
};

class Motor {
	
	private:
	
		bool nextCommandSet;
	
        double currentState;
        double nextCommand;
	
		int motorId;
        double lowerLimit;
        double upperLimit;
		int baudRate;
	
		std::string deviceName;
		float  protocolVersion;
	
		ROBOTIS::PortHandler* portHandler;
		ROBOTIS::PacketHandler* packetHandler;
		
		std::mutex stateMutex;
		std::mutex commandMutex;
		
        void sendNextCommand(double pos);

        void submitPacket(ROBOTIS::PortHandler* portHandler, int motorId, int address, int value);
        void submitPacket(ROBOTIS::PortHandler* portHandler, int motorId, int address, UINT8_T value);

        int receivePacket(ROBOTIS::PortHandler* portHandler, int motorId, int address);

        int receiveState();
		
	public:
	
		static auto constexpr STD_FREQUENCY = 80.0;

        static auto constexpr TICKS_FOR_180_DEG = 150000.0;

        static auto constexpr STD_STEP_SIZE = 20.0 / TICKS_FOR_180_DEG * M_PI;
        static auto constexpr STD_MAX_VEL_LIMIT = 2000.0 / TICKS_FOR_180_DEG * M_PI;
	
        Motor(std::string deviceName, int motorId, float protocolVersion, double lowerLimit, double upperLimit, int baudRate);
		
        double getStepSize();
		double getFrequency();
		
		void initialize();
		
		void setTorqueMode(bool mode);
		
		void startTorqueMode();
		
		void stopTorqueMode();
		
		void setBrakes(bool brakesOpen);
		
		void releaseBrakes();
		
		void closeBrakes();
		
		void shutdown();
		
		void spinOnce();
		
		void simplePtp(int targetPos);
		
		void flushNextState();
		
		void goHome();
		
        double getCurrentState();
		
        void setNextState(double state);

        double getMaxVelLimit();
	
};

class Arm {

    private:

        bool keepThreadRunning;
        bool firstJointStateRetrieved;

        std::vector<std::shared_ptr<Motor> > motors;
        std::vector<double> currentJointState;

        std::mutex jointStateMutex;

        std::shared_ptr<std::thread> runnerThread;

        void armLoop();

        bool checkDistance(std::vector<double> &current, std::vector<double> &target);

    public:

        Arm(std::vector<int> ids, std::string portName, std::vector< std::pair<double, double> > jointLimits, double protocolVersion, int baudRate);

        std::vector<double> getCurrentJointState();

        void initialize();

        void move(std::vector<double> nextJointPos);

        std::shared_ptr<std::thread> runArm();

        void shutdown();

        void jointPtp(std::vector<double> targetPos);

        void moveHome();

        double getStepSize();
        int getDegOfFreedom();

        double getFrequency();
        double getCycleTime();
        double getMaxStepPerCycle();

};

}
