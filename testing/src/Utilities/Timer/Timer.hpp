//============================================================================
// Name        : Timer.hpp
// Author      : Produced in the WPI AIM Lab
// Description : This file is used to profile the code
//============================================================================

#ifndef TIMER_HPP_
#define TIMER_HPP_

#include <time.h>
#include <stdio.h>

class Timer {
public:
	//================ Constructor =================
	Timer();

	//================ Parameters =================
	// Parameters are declared in constructor
	struct timespec _clockStart, _clockFinish;
	unsigned long long _usStart, _usFinish; //microseconds

	//================ Public Methods ==============
	void tic();
	void toc();
	unsigned long long time();
	double ConvertMicrosecToSec(const double &t);
};

#endif /* TIMER_HPP_ */
