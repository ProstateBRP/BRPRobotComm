#ifndef __OPENIGTLINK_HPP__
#define __OPENIGTLINK_HPP__

// System includes
#include <iostream>
#include <iomanip>
#include <math.h>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

// IGTL includes
#include "igtlServerSocket.h"

// Prostate Robot Work Phase includes
#include "ProstateRobotStatus.hpp"
#include "ProstateRobotPhaseBase.hpp"
#include "ProstateRobotUndefinedPhase.hpp"
#include "ProstateRobotStartUpPhase.hpp"
#include "ProstateRobotPlanningPhase.hpp"
#include "ProstateRobotCalibrationPhase.hpp"
#include "ProstateRobotTargetingPhase.hpp"
#include "ProstateRobotMoveToTargetPhase.hpp"
#include "ProstateRobotManualPhase.hpp"
#include "ProstateRobotStopPhase.hpp"
#include "ProstateRobotEmergencyPhase.hpp"
// // Neuro Robot Work Phase includes
// #include "NeuroRobotStatus.hpp"
// #include "NeuroRobotPhaseBase.hpp"
// #include "NeuroRobotUndefinedPhase.hpp"
// #include "NeuroRobotStartUpPhase.hpp"
// #include "NeuroRobotPlanningPhase.hpp"
// #include "NeuroRobotCalibrationPhase.hpp"
// #include "NeuroRobotTargetingPhase.hpp"
// #include "NeuroRobotMoveToTargetPhase.hpp"
// #include "NeuroRobotManualPhase.hpp"
// #include "NeuroRobotStopPhase.hpp"
// #include "NeuroRobotEmergencyPhase.hpp"
// #include "NeuroRobotDrapingPhase.hpp"

template <typename TRobot>
class OpenIGTLink
{
public:
	//================ Constructor ================
	OpenIGTLink(shared_ptr<TRobot> robot, int port);

	//================ Parameters =================
	int clientSocketConnected{1};
	igtl::Socket::Pointer socket;
	shared_ptr<TRobot> robot;
	bool retransmit;
	int port;
	// For TCP Keep Alive
	bool keepAlive;
	// Timer keepAliveTimer;
	std::vector<shared_ptr<ProstateRobotPhaseBase>> prostate_workphase_list{};
	// std::vector<shared_ptr<NeuroRobotPhaseBase>> neuro_workphase_list{};

	//================ Public Methods ==============
	// This method receives data to the controller via OpenIGTLink
	static void *ThreadIGT(void *);
	int Session();
	// This method disconnect the current socket
	void DisconnectSocket();
};

#endif /* __OPENIGTLINK_HPP__ */
