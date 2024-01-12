//============================================================================
// Name        : WebServer.cpp
// Author      : Produced in the WPI AIM Lab
// Description : This file is where the API requests and the C++ Server runs
//============================================================================
template <typename T> class OpenIGTLink;

#ifndef SERVER_HPP_
#define SERVER_HPP_

// Added for the default_resource example
#include <algorithm>
#include <fstream>
#include <regex>
#include <thread>
#include <vector>

#include "OpenIGTLink.hpp"
#include "SimpleServer/server_http.hpp"
#include "picojson.h"

using namespace std;
using HttpServer = SimpleWeb::Server<SimpleWeb::HTTP>;

template <typename TRobot>
class CustomWebServer
{
public:
	CustomWebServer(TRobot *robot, vector<OpenIGTLink<TRobot> *> igtList, int port);

	vector<OpenIGTLink<TRobot> *> _igtList;
	HttpServer server;
	TRobot *_robot;

	void ParseStringToArray(const string &input, char delim, vector<float> *result);
	template <class T>
	string ParseEigenToString(T input);
	static void *runServer(void *webServer);
};

#endif /* SERVER_HPP_ */
