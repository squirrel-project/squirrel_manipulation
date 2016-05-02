#include <uibk_arm_controller/arm_controller.hpp>

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

    void Motor::sendNextCommand(int pos) {
		
        if(pos < upperLimit && pos > lowerLimit)
            submitPacket(portHandler, motorId, ADDR_PRO_GOAL_POSITION, pos);
        else
			std::cerr << "commanded position out of security limits (com: " << pos << ", lim: " << lowerLimit << ", " << upperLimit << ")" << std::endl;
				
	}
		
	Motor::Motor(std::string deviceName, int motorId, float protocolVersion, int lowerLimit, int upperLimit, int baudRate) {
		this->deviceName = deviceName;
		this->motorId = motorId;
		this->protocolVersion = protocolVersion;
		this->lowerLimit = lowerLimit;
		this->upperLimit = upperLimit;
		this->baudRate = baudRate;
		nextCommandSet = false;
	}
			
	int Motor::getStepSize() { return STD_STEP_SIZE; }
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
		
		UINT8_T dxl_error = 0;
		int dxl_comm_result = COMM_TX_FAIL;
		INT32_T dxl_present_position = 0;
		
		// Read Dynamixel#1 present position
		dxl_comm_result = packetHandler->Read4ByteTxRx(portHandler, motorId, ADDR_PRO_PRESENT_POSITION, (UINT32_T*)&dxl_present_position, &dxl_error);
		if(dxl_comm_result != COMM_SUCCESS) {
			std::stringstream s;
			s << "communication error";
			packetHandler->PrintTxRxResult(dxl_comm_result);
			throw MotorException(s.str().c_str());
		} else if(dxl_error != 0) {
			std::stringstream s;
			s << "error";
			packetHandler->PrintRxPacketError(dxl_error);
			throw MotorException(s.str().c_str());
		}
		
		return dxl_present_position;
			
	}

	void Motor::spinOnce() {
		
		stateMutex.lock();
			bool worked = false;
			for(int i = 0; i < 3 && !worked; ++i) {
				try {
					currentState = receiveState();
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
		for(; abs(currState - targetPos) > 500; currState += sig * STD_STEP_SIZE) {
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

	int Motor::getCurrentState() {
		
		int stateBackup;
		stateMutex.lock();
			stateBackup = currentState;
		stateMutex.unlock();
		return stateBackup;
			
	}

	void Motor::setNextState(int state) {
		
		commandMutex.lock();
			nextCommandSet = true;
			nextCommand = state;
		commandMutex.unlock();
		
	}

	void Arm::armLoop() {
		
		auto frequ = motors.front()->getFrequency();
		int sleepTime = (int) (1.0 / frequ * 1e3);
		while(keepThreadRunning) {
			
			for(auto motor : motors)
				motor->spinOnce();
			
			jointStateMutex.lock();
			
				currentJointState.clear();
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

	Arm::Arm(std::vector<int> ids, std::string portName, std::vector< std::pair<int, int> > jointLimits, double protocolVersion, int baudRate) {
		
		firstJointStateRetrieved = false;
		for(unsigned int i = 0; i < ids.size(); ++i) {
			auto id = ids.at(i);
			auto limit = jointLimits.at(i);
			motors.push_back(std::make_shared<Motor>(portName, id, protocolVersion, limit.first, limit.second, baudRate));
		}
		
		keepThreadRunning = false;
		
	}

	std::vector<int> Arm::getCurrentJointState() {
		std::vector<int> jointStateBkp;
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

	void Arm::move(std::vector<int> nextJointPos) {
		
		if(nextJointPos.size() != motors.size())
			throw MotorException("number of joints doesn't fit the number of motors");
			
		for(unsigned int i = 0; i < nextJointPos.size(); ++i)
			motors.at(i)->setNextState(nextJointPos.at(i));
			
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

	void Arm::jointPtp(std::vector<int> targetPos) {
		
		auto jointState = getCurrentJointState();
		auto runnerState = jointState;
		int sleepTime = (int) (1.0 / getFrequency() * 1e3);
		auto stepSize = getStepSize() * 3;
		
		auto targetReached = false;
		while(!targetReached) {

			targetReached = true;
			for(unsigned int i = 0; i < runnerState.size(); ++i) {
				auto sig = ((runnerState.at(i) - targetPos.at(i)) > 0) ? -1 : 1;
				if(abs(runnerState.at(i) - targetPos.at(i)) > (stepSize * 2)) {
					runnerState.at(i) += sig * stepSize;
					targetReached = false;
				}
			}
				
			move(runnerState);
				
			std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
			
		}
			

	}

	void Arm::moveHome() {
		
		std::vector<int> homeJoints;
		for(unsigned int i = 0; i < motors.size(); ++i)
			homeJoints.push_back(0);
			
		jointPtp(homeJoints);
		
	}

	int Arm::getStepSize() { return motors.front()->getStepSize(); }
	double Arm::getFrequency() { return motors.front()->getFrequency(); }

}
