//============================================================================
// Name        : Logger.hpp
// Author      : Produced in the WPI AIM Lab
// Description : This file is used for logging information from the system to a text file
//============================================================================

#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <pthread.h>
#include "Eigen/Dense"
#include "igtlMath.h"

#include "Timer.hpp"

using namespace std;

namespace logger
{
	enum LogSeverity
	{
		DEBUG = 0,
		INFO = 1,
		WARNING = 2,
		ERROR = 3,
		CRITICAL = 4,
		DATA = 5
	};

}

class Logger
{
public:
	//================ Constructor ================
	Logger();

	//================ Parameters =================
	// Parameters are initialized in constructor

	Timer _timer;
	string _filename;
	fstream _file; // for reading and writing to the file
	int _logLevelSeverity;
	int _currentLineNumber;
	std::string _actualTime;

	pthread_mutex_t logMutex;

	//================ Public Methods ==============
	string GetFileName();
	vector<vector<string>> ReadLogData(int numberOfLinesToRead);
	void SetLogLevel(int severity);
	void Log(const string &logData, int severity, bool printToConsole = false);
	void Log(const string &logData, string actual_time, int severity, bool printToConsole = false);
	void Log(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &logData, string name, string actual_time, int severity, bool printToConsole = false);
	void Log(const Eigen::Matrix<double, 4, 1, Eigen::DontAlign> &logData, string name, string actual_time, int severity, bool printToConsole = false);
	void Log(const igtl::Matrix4x4 &logData, string name, string actual_time, int severity, bool printToConsole = false);
	void SetActualTime(std::string actualTime);
	string GetActualTime() { return this->_actualTime; }
	//======== Singleton Specific Methods ===========
	static Logger &GetInstance()
	{
		static Logger instance;
		return instance;
	}

	Logger(Logger const &) = delete;
	void operator=(Logger const &) = delete;
};

#endif /* LOGGER_HPP_ */
