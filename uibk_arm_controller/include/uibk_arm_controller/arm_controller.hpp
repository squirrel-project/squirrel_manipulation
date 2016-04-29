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
#include "DynamixelSDK.h" 

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
	
		int currentState;
		int nextCommand;
	
		int motorId;
		int lowerLimit;
		int upperLimit;
		int baudRate;
	
		std::string deviceName;
		float  protocolVersion;
	
		ROBOTIS::PortHandler* portHandler;
		ROBOTIS::PacketHandler* packetHandler;
		
		std::mutex stateMutex;
		std::mutex commandMutex;
		
		void sendNextCommand(int pos);
		
	public:
	
		static auto constexpr STD_STEP_SIZE = 20;
		static auto constexpr STD_FREQUENCY = 80.0;
	
		Motor(std::string deviceName, int motorId, float protocolVersion, int lowerLimit, int upperLimit, int baudRate);
		
		int getStepSize();
		double getFrequency();
		
		void initialize();
		
		void setTorqueMode(bool mode);
		
		void startTorqueMode();
		
		void stopTorqueMode();
		
		void setBrakes(bool brakesOpen);
		
		void releaseBrakes();
		
		void closeBrakes();
		
		void shutdown();
		
		int receiveState();
		
		void spinOnce();
		
		void simplePtp(int targetPos);
		
		void flushNextState();
		
		void goHome();
		
		int getCurrentState();
		
		void setNextState(int state);
	
};

class Arm {
	
	private:
	
		bool keepThreadRunning;
		bool firstJointStateRetrieved;
	
		std::vector<std::shared_ptr<Motor> > motors;
		std::vector<int> currentJointState;
		
		std::mutex jointStateMutex;
		
		std::shared_ptr<std::thread> runnerThread;
		
		void armLoop();
		
	public:
	
		Arm(std::vector<int> ids, std::string portName, std::vector< std::pair<int, int> > jointLimits, double protocolVersion, int baudRate);
		
		std::vector<int> getCurrentJointState();
		
		void initialize();
		
		void move(std::vector<int> nextJointPos);
		
		std::shared_ptr<std::thread> runArm();
		
		void shutdown();
		
		void jointPtp(std::vector<int> targetPos);
		
		void moveHome();
	
		int getStepSize();
		double getFrequency();

};

}
