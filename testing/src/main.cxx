//============================================================================
// Name        : SurgicalRobot.cpp
// Author      : Produced in the WPI AIM Lab
// Description : This file is where the code starts via the main function.
//				 It includes initializing the menu and setting up the system.
//============================================================================

#include <utility>
#include <memory>
#include <map>
#include <string>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include "malloc.h"
#include <endian.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include "./ProstateRobot/ProstateRobot.hpp"
#include "./OpenIGTLink/OpenIGTLink.hpp"
#include "./Utilities/Logger/Logger.hpp"
#include "./Utilities/Timer/Timer.hpp"

template <typename TRobot>
void RobotFunctionality(shared_ptr<TRobot> _robot, Timer _timer, unsigned int loopRate, int port)
{

	//====================================== PTHREADS ========================================
	// Set up thread affinities -- these can be used to run threads on specific cores
	cpu_set_t cpuset0;
	cpu_set_t cpuset1;
	CPU_ZERO(&cpuset0);
	CPU_ZERO(&cpuset1);
	CPU_SET(0, &cpuset0);
	CPU_SET(1, &cpuset1);

	// Set the main thread to use core 0
	pthread_t current_thread = pthread_self();
	pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset0);

	// Create a thread for OpenIGT Communications
	pthread_t _igtID;
	OpenIGTLink<TRobot> _igtModule = OpenIGTLink<TRobot>(_robot, 18936);
	void *_joinIgt;
	pthread_create(&_igtID, NULL, &OpenIGTLink<TRobot>::ThreadIGT, (void *)&_igtModule);
	vector<OpenIGTLink<TRobot> *> _igtList = {&_igtModule};
	pthread_setaffinity_np(_igtID, sizeof(cpu_set_t), &cpuset1);

	//====================================== MAIN LOOP ========================================
	// Launch main robot program
	while (1)
	{
		_robot->Update();
		// Profile code to maintain a consistent loop time
		do
		{
			_timer.toc();
		} while ((_timer.time()) < loopRate);
	}

	// Wait for all threads to finish
	pthread_join(_igtID, &_joinIgt);
}

int main(int argc, char *argv[])
{
	// Log that the system has been initialized
	Logger &log = Logger::GetInstance();
	log.Log("Starting ProstateRobot Comm testing...", logger::INFO, true);

	// Create a Timer to profile the while loop at the bottom of this main() And determine a loop rate
	Timer _timer;
	unsigned int loopRate = 900; // microseconds

	//===================================== ROBOTS ==========================================
	// Launch Robot Objects -- Assume we're using the NeuroRobot unless told otherwise
	int port{18936};
	if (argc > 1)
	{
		port = atoi(argv[1]);
	}
	string ss = "Launching ProstateRobot Comm testing system on port " + std::to_string(port);
	ProstateRobot _robot = ProstateRobot();
	std::shared_ptr<ProstateRobot> robot_ptr(&_robot, [](ProstateRobot*) {});
	RobotFunctionality<ProstateRobot>(robot_ptr, _timer, loopRate, port);
	
	return 0;
}
