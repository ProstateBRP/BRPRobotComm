//============================================================================
// Name        : CommModuleTest.hpp
// Author      : Produced in the WPI AIM Lab
// Description : This file is where the code starts via the main function.
//				 It includes initializing the menu and setting up the system.
//============================================================================

#ifndef SURGICALROBOT_HPP_
#define SURGICALROBOT_HPP_

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

#include "ProstateRobot.hpp"
#include "Logger.hpp"
#include "OpenIGTLink.hpp"

using namespace std;

//================ Structure Definitions ==============
struct cardStatus
{
	uint32_t encoderCount;
	uint8_t channelA;
	uint8_t channelB;
	uint8_t channelI;
	uint32_t indexLatch;

	uint8_t lowerLimit;
	uint8_t upperLimmit;
	uint8_t homeLimit;
	uint32_t lowerLatch;
	uint32_t upperLatch;
	uint32_t homeLatch;
};

#endif /* SURGICALROBOT_HPP_ */
