#include <uibk_arm_controller/arm_controller.hpp>
#include <math.h>

using namespace ROBOTIS;                                    // Uses functions defined in ROBOTIS namespace

namespace uibk_arm_controller {

	MotorException::MotorException(const char* msg) { this->msg = msg; }
			
	const char* MotorException::what() const throw() {
	 return msg;
	}

    void Motor::submitPacket(ROBOTIS::PortHandler* portHandler, int motorId, int address, int value) {

        UINT8_T dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        for(int i = 0; i < 3; ++i) {

            dxl_comm_result = packetHandler->Write4ByteTxRx(portHandler, motorId, address, value, &dxl_error);
            if(dxl_comm_result == COMM_SUCCESS)
                return;

        }

        // if not worked in once in these 3 times --> throw exception
        throw MotorException("failed to commicate with motor");

    }

    void Motor::submitPacket(ROBOTIS::PortHandler* portHandler, int motorId, int address, UINT8_T value) {

        UINT8_T dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        for(int i = 0; i < 3; ++i) {

            dxl_comm_result = packetHandler->Write1ByteTxRx(portHandler, motorId, address, value, &dxl_error);
            if(dxl_comm_result == COMM_SUCCESS)
                return;

        }

        // if not worked in once in these 3 times --> throw exception
        throw MotorException("failed to commicate with motor");

    }

    int Motor::receivePacket(ROBOTIS::PortHandler* portHandler, int motorId, int address) {

        UINT8_T dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;
        int dxl_present_position = 0;

        for(int i = 0; i < 3; ++i) {

            dxl_comm_result = packetHandler->Read4ByteTxRx(portHandler, motorId, address, (UINT32_T*)&dxl_present_position, &dxl_error);
            if(dxl_comm_result == COMM_SUCCESS)
                return dxl_present_position;

        }

        return dxl_present_position;

    }

    void Motor::sendNextCommand(double pos) {
		
        if(pos < upperLimit && pos > lowerLimit){
            submitPacket(portHandler, motorId, ADDR_PRO_GOAL_POSITION, (int) (pos / M_PI * TICKS_FOR_180_DEG));
        }else
			std::cerr << "commanded position out of security limits (com: " << pos << ", lim: " << lowerLimit << ", " << upperLimit << ")" << std::endl;
				
	}
		
    Motor::Motor(std::string deviceName, int motorId, float protocolVersion, double lowerLimit, double upperLimit, int baudRate) {
		this->deviceName = deviceName;
		this->motorId = motorId;
		this->protocolVersion = protocolVersion;
		this->lowerLimit = lowerLimit;
		this->upperLimit = upperLimit;
		this->baudRate = baudRate;
		nextCommandSet = false;
	}
			
    double Motor::getStepSize() { return STD_STEP_SIZE; }
	double Motor::getFrequency() { return STD_FREQUENCY; }

	void Motor::initialize() {
		
		// Initialize PortHandler instance
		// Set the port path
		// Get methods and members of PortHandlerLinux or PortHandlerWindows		
		portHandler = PortHandler::GetPortHandler(deviceName.c_str());
		
		// Initialize Packethandler instance
		// Set the protocol version
		// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
		packetHandler = PacketHandler::GetPacketHandler(protocolVersion);
		
		// open port
		if(portHandler->OpenPort())
			std::cout << "Succeeded to open the port " << deviceName << std::endl;
		else
			throw MotorException(std::string("opening for id " + deviceName + " failed").c_str());
			
		// Set port baudrate
		if(portHandler->SetBaudRate(baudRate)) {
			std::cout << "Succeeded to set a baud rate of " << baudRate << std::endl;
		} else {
			std::stringstream s;
			s << "setting baud rate of " << baudRate << " failed";
			throw MotorException(s.str().c_str());
		}
		
		nextCommandSet = false;
		
	}
			
	void Motor::setTorqueMode(bool mode) {
		
		// Enable Dynamixel#1 Torque
		int torqueMode = 0;
		if(mode)
			torqueMode = 1;

        submitPacket(portHandler, motorId, ADDR_PRO_TORQUE_ENABLE, torqueMode);
		
	}

	void Motor::startTorqueMode() {
		
		setTorqueMode(true);
		
	}

	void Motor::stopTorqueMode() {
		
		setTorqueMode(false);
		
	}

	void Motor::setBrakes(bool brakesOpen) {
		
		int brakeVal = 0;
		if(brakesOpen)
			brakeVal = 4095;

        submitPacket(portHandler, motorId, 626, brakeVal);
		
	}

	void Motor::releaseBrakes() {
		
		setBrakes(true);
		
	}

	void Motor::closeBrakes() {
		
		setBrakes(false);
		
	}

	void Motor::shutdown() {
		
		closeBrakes();
		stopTorqueMode();
		portHandler->ClosePort();
		
	}

    int Motor::receiveState() {

        // Read Dynamixel#1 present position
        auto dxl_present_position = receivePacket(portHandler, motorId, ADDR_PRO_PRESENT_POSITION);
		return dxl_present_position;
			
	}

	void Motor::spinOnce() {
		
		stateMutex.lock();
			bool worked = false;
			for(int i = 0; i < 3 && !worked; ++i) {
				try {
                    currentState = (double) (receiveState() / TICKS_FOR_180_DEG * M_PI);
					worked = true;
				} catch(MotorException &ex) {
					
				}
			}
			if(!worked)
				throw MotorException("transmission problem (check your connection)");
				
		stateMutex.unlock();
		
		commandMutex.lock();
		
			if(nextCommandSet) {
				
                nextCommandSet = false;
                sendNextCommand(nextCommand);
					
			}
			
		commandMutex.unlock();
		
	}

	void Motor::simplePtp(int targetPos) {
		
		int stdSleepTime = (int) (1.0 / STD_FREQUENCY * 1E3);
		
		flushNextState();
		spinOnce();
		
		auto currState = getCurrentState();
		int sig = ((targetPos - currState) > 0) ? 1 : -1;
		for(; fabs(currState - targetPos) > (500 / TICKS_FOR_180_DEG * M_PI); currState += sig * STD_STEP_SIZE) {
			setNextState(currState);
			spinOnce();
			std::this_thread::sleep_for(std::chrono::milliseconds(stdSleepTime));
		}
		
	}

	void Motor::flushNextState() {
		commandMutex.lock();
			nextCommandSet = false;
		commandMutex.unlock();
	}

	void Motor::goHome() {
		simplePtp(0);
	}

    double Motor::getCurrentState() {
		
		double stateBackup;
		stateMutex.lock();
			stateBackup = currentState;
		stateMutex.unlock();
		return stateBackup;
			
	}

    void Motor::setNextState(double state) {
		
		commandMutex.lock();
			nextCommandSet = true;
			nextCommand = state;
		commandMutex.unlock();
		
	}

    double Motor::getMaxVelLimit() {
        return STD_MAX_VEL_LIMIT;
    }

	void Arm::armLoop() {
		
		auto frequ = motors.front()->getFrequency();
		int sleepTime = (int) (1.0 / frequ * 1e3);
		while(keepThreadRunning) {
			
			for(auto motor : motors)
				motor->spinOnce();
			
			jointStateMutex.lock();
			
				currentJointState.clear();
                currentJointState.push_back(base->getCurrentState());
				for(auto motor : motors) {

					auto currState = motor->getCurrentState();
                    currentJointState.push_back(currState);

				}
				
			jointStateMutex.unlock();
			if(!firstJointStateRetrieved)
				firstJointStateRetrieved = true;
			std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
			
		}
		
	}

    Arm::Arm(ros::NodeHandle& node, std::vector<int> ids, std::string portName, std::vector<std::pair<double, double> > jointLimits, double protocolVersion, int baudRate) {
		
		firstJointStateRetrieved = false;
		for(unsigned int i = 0; i < ids.size(); ++i) {
			auto id = ids.at(i);
			auto limit = jointLimits.at(i);
			motors.push_back(std::make_shared<Motor>(portName, id, protocolVersion, limit.first, limit.second, baudRate));
		}

        base = std::make_shared<RobotinoBaseControl>(node, 20, 0.6);
		
		keepThreadRunning = false;
		
	}

    std::vector<double> Arm::getCurrentJointState() {
        std::vector<double> jointStateBkp;
		jointStateMutex.lock();
			jointStateBkp = currentJointState;
		jointStateMutex.unlock();
		return jointStateBkp;
	}

	void Arm::initialize() {
		
		for(auto motor : motors) {
			motor->initialize();
			motor->startTorqueMode();
			motor->releaseBrakes();
			motor->spinOnce();
		}
		
	}

    void Arm::move(std::vector<double> nextJointPos) {
		
        if(nextJointPos.size() != (motors.size() + 1))
			throw MotorException("number of joints doesn't fit the number of motors");

        base->move(nextJointPos.front());
			
        auto js = getCurrentJointState();
        double velLimit = 0.0;
        double exceededDist = 0.0;
        if(checkDistance(js, nextJointPos, exceededDist, velLimit)) {
            for(unsigned int i = 1; i < nextJointPos.size(); ++i)
                motors.at(i - 1)->setNextState(nextJointPos.at(i));
        } else {
            std::cerr << "velocity limit exceeded (maxVel: " << velLimit << ", commandedVel: " << exceededDist << ")" << std::endl;
        }
			
	}

    bool Arm::checkDistance(std::vector<double>& current, std::vector<double>& target, double& exceededDist, double& maxDist) {

        if(fabs(current.front() - target.front()) > 0.6) {
            exceededDist = fabs(current.front() - target.front());
            maxDist = 0.6;
            return false;
        }

        for(int i = 1; i < current.size(); ++i) {

            if(fabs(current.at(i) - target.at(i)) > motors.at(i - 1)->getMaxVelLimit()) {
                exceededDist = fabs(current.at(i) - target.at(i));
                maxDist = motors.at(i - 1)->getMaxVelLimit();
                return false;
            }

        }
        return true;
    }

    double Arm::getMaxStepPerCycle() {
        return motors.front()->getMaxVelLimit();
    }

	std::shared_ptr<std::thread> Arm::runArm() {
		
		keepThreadRunning = true;
		runnerThread = std::make_shared<std::thread>(&Arm::armLoop, this);
		while(!firstJointStateRetrieved)
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		
		return runnerThread;
		
	}

	void Arm::shutdown() {
		
		keepThreadRunning = false;
		if(runnerThread)
			runnerThread->join();
		for(auto motor : motors) {
			motor->shutdown();
		}
		
	}

    void Arm::jointPtp(std::vector<double> targetPos) {
		
		auto jointState = getCurrentJointState();
		auto runnerState = jointState;
		int sleepTime = (int) (1.0 / getFrequency() * 1e3);
		auto stepSize = getStepSize() * 3;
		
		auto targetReached = false;
		while(!targetReached) {

			targetReached = true;
			for(unsigned int i = 1; i < runnerState.size(); ++i) {
				auto sig = ((runnerState.at(i) - targetPos.at(i)) > 0) ? -1 : 1;
				if(fabs(runnerState.at(i) - targetPos.at(i)) > (stepSize * 2)) {
					runnerState.at(i) += sig * stepSize;
					targetReached = false;
				}
			}
				
			move(runnerState);
				
			std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
			
		}
			

	}

	void Arm::moveHome() {
		
        std::vector<double> homeJoints;
		for(unsigned int i = 0; i < motors.size(); ++i)
            homeJoints.push_back(0.0);
			
		jointPtp(homeJoints);
		
	}

    double Arm::getStepSize() { return motors.front()->getStepSize(); }
    int Arm::getDegOfFreedom() { return motors.size() + 1; }
	double Arm::getFrequency() { return motors.front()->getFrequency(); }
    double Arm::getCycleTime() { return 1.0 / getFrequency(); }

}
