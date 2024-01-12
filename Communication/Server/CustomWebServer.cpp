//============================================================================
// Name        : WebServer.hpp
// Author      : Produced in the WPI AIM Lab
// Description : This file is where the API requests and the C++ Server runs
//============================================================================

#include "CustomWebServer.hpp"

#ifndef DEGREE_TO_RADIAN
#define DEGREE_TO_RADIAN (1 / 57.29578)
#endif // DEGREE_TO_RADIAN

template <>
CustomWebServer<NeuroRobot>::CustomWebServer(NeuroRobot *robot, vector<OpenIGTLink<NeuroRobot> *> igtList, int port)
{

	// Instantiate the constructor parameters
	_igtList = igtList;
	_robot = robot;
	server.config.port = port;

	// ****************************************************
	// **** POST /api/login
	server.resource["/api/login"]["POST"] = [](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{
			// Get the request body
			picojson::value request_body;
			std::string err = picojson::parse(request_body, request->content);
			if (!err.empty())
			{
				cerr << err << endl;
			}

			// Record requests to the log
			log.Log("Started POST '/api/login' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			if ((request_body.get("username").to_str() == "root") && (request_body.get("password").to_str() == "123456"))
			{
				// Set the appropriate headers for this request
				SimpleWeb::CaseInsensitiveMultimap header;
				header.emplace("Access-Control-Allow-Origin", "*");

				response->write(SimpleWeb::StatusCode::success_ok, "{\"token\":\"SIMPLE_TOKEN_1\"}", header);
			}
			else
			{
				SimpleWeb::CaseInsensitiveMultimap header;
				header.emplace("Access-Control-Allow-Origin", "*");
				response->write(SimpleWeb::StatusCode::client_error_not_found, "Wrong username or password.", header);
			}
		}

		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed POST '/api/login' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** GET /api/robots
	server.resource["/api/robots"]["GET"] = [](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{

			// Set the appropriate headers for this request
			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			// Record requests to the log
			log.Log("Started GET '/api/robots' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			// Prepare the response to be sent to the Web GUI
			response->write(SimpleWeb::StatusCode::success_ok, "[ { \"id\": 1, \"name\": \" NeuroRobot \" }, { \"id\": 2, \"name\": \" ProstateRobot \" } ]", header);
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed GET '/api/robots' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** POST /api/robots
	server.resource["/api/robots"]["POST"] = [](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{
			// Get the request body
			picojson::value request_body;
			std::string err = picojson::parse(request_body, request->content);
			if (!err.empty())
			{
				cerr << err << endl;
			}

			// Record requests to the log
			log.Log("Started POST '/api/robots' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			// If we can successfully instantiate the robot
			// TODO: Implement this
			if (request_body.get("id").to_str() == "1")
			{
				SimpleWeb::CaseInsensitiveMultimap header;
				header.emplace("Access-Control-Allow-Origin", "*");

				response->write(SimpleWeb::StatusCode::success_ok, "{ \"id\": 1, \"name\": \" NeuroRobot \" }", header);
			}
			else
			{
				SimpleWeb::CaseInsensitiveMultimap header;
				header.emplace("Access-Control-Allow-Origin", "*");

				response->write(SimpleWeb::StatusCode::client_error_bad_request, "Invalid robot ID.", header);
			}
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed POST '/api/robots' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** GET /api/motors
	server.resource["/api/motors"]["GET"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{
			// Record requests to the log
			log.Log("Started GET '/api/motors' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			// Put together motor data as JSON for processing
			vector<Motor *> motors = _robot->ListMotors();
			string json = "[";
			for (uint i = 0; i < motors.size(); i++)
			{
				if (i == 0)
				{
					json += "{";
				}
				else
				{
					json += ",{";
				}
				json += "\"slot\":" + to_string(motors[i]->_cardID) + ",";
				json += "\"name\": \"" + motors[i]->_name + "\",";
				json += "\"type\": \"none\","; // TODO: Add card type here
				json += "\"enabled\":" + to_string(motors[i]->_enabled) + ",";
				json += "\"current_ticks\":" + to_string(motors[i]->GetEncoderPositionTicks()) + ",";
				json += "\"desired_ticks\":" + to_string(motors[i]->_setpoint) + ",";
				json += "\"min_ticks\":" + to_string(motors[i]->GetMinTicks()) + ",";
				json += "\"max_ticks\":" + to_string(motors[i]->GetMaxTicks()) + ",";
				if (motors[i]->_unit == "degree")
				{
					json += "\"conversion_factor\":" + to_string(motors[i]->_ticksPerUnit * DEGREE_TO_RADIAN) + ",";
				}
				else
				{
					json += "\"conversion_factor\":" + to_string(motors[i]->_ticksPerUnit) + ",";
				}
				json += "\"unit\": \"" + motors[i]->_unit + "\",";
				json += "\"limits\": [{\"name\": \"Upper\", \"value\":" + to_string(motors[i]->IsLimit()) + "}],";
				json += "\"homed\":" + to_string(motors[i]->_homed) + ",";
				json += "\"homing\":" + to_string(motors[i]->_homing) + ",";
				json += "\"faults\": []";
				json += "}";
			}
			json += "]";

			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			response->write(SimpleWeb::StatusCode::success_ok, json, header);
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed GET '/api/motors' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** GET /api/motors/restore
	server.resource["/api/motors/restore"]["PUT"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{
			// Get the request body
			picojson::value request_body;
			std::string err = picojson::parse(request_body, request->content);
			if (!err.empty())
			{
				cerr << err << endl;
			}

			// Record requests to the log
			log.Log("Started GET '/api/motors/restore' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			int request_size = request_body.get<picojson::array>().size();
			for (int i = 0; i < request_size; i++)
			{
				int motorID = stoi(request_body.get(i).get("slot").to_str());
				int newCurrent = stoi(request_body.get(i).get("current_ticks").to_str());
				bool newHomedValue = stoi(request_body.get(i).get("homed").to_str());

				Motor *motor = _robot->GetMotor(motorID);
				motor->SetEncoderOffsetByTicks(newCurrent);
				motor->_homed = newHomedValue;
			}

			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");
			response->write(SimpleWeb::StatusCode::success_ok, "{}", header);
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed GET '/api/motors/restore' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	//**** PUT /api/motors/:slot
	server.resource["/api/motors"]["PUT"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{
			// Record requests to the log
			log.Log("Started PUT '/api/motors' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			// Process the requests
			picojson::value request_body;
			std::string err = picojson::parse(request_body, request->content);
			if (!err.empty())
			{
				cerr << err << endl;
			}

			log.Log("Started PUT '/api/motors' for " + request_body.to_str(), logger::DEBUG, false);

			// The slot determined from the match in the URL
			int _current_slot = stoi(request_body.get("slot").to_str());

			// Extract data from the request
			Motor *motor = _robot->GetMotor(_current_slot);
			motor->_setpoint = stoi(request_body.get("desired_ticks").to_str());
			motor->SetHome(request_body.get("homed").to_str());
			motor->_homing = (request_body.get("homing").to_str() == "true");
			motor->_enabled = (request_body.get("enabled").to_str() == "true");

			string json = "{";
			json += "\"slot\":" + to_string(motor->_cardID) + ",";
			json += "\"name\": \"" + motor->_name + "\",";
			json += "\"type\": \"none\","; // TODO: Add card type here
			json += "\"enabled\":" + to_string(motor->_enabled) + ",";
			json += "\"current_ticks\":" + to_string(motor->GetEncoderPositionTicks()) + ",";
			json += "\"desired_ticks\":" + to_string(motor->_setpoint) + ",";
			json += "\"min_ticks\":" + to_string(motor->GetMinTicks()) + ",";
			json += "\"max_ticks\":" + to_string(motor->GetMaxTicks()) + ",";
			if (motor->_unit == "degree")
			{
				json += "\"conversion_factor\":" + to_string(motor->_ticksPerUnit * DEGREE_TO_RADIAN) + ",";
			}
			else
			{
				json += "\"conversion_factor\":" + to_string(motor->_ticksPerUnit) + ",";
			}
			json += "\"units\": \"" + motor->_unit + "\",";
			json += "\"limits\": [{\"name\": \"Upper\", \"value\":" + to_string(motor->IsLimit()) + "}],";
			json += "\"homed\":" + to_string(motor->_homed) + ",";
			json += "\"homing\":" + to_string(motor->_homing) + ",";
			json += "\"faults\": []";
			json += "}";

			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			response->write(SimpleWeb::StatusCode::success_ok, json, header);

			_robot->RunAxisSetpointValidator();
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed PUT '/api/motors' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** GET /api/targetting
	server.resource["/api/targeting"]["GET"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{
			// Record requests to the log
			log.Log("Started GET '/api/targeting' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			// Put together motor data as JSON for processing
			string json = "{";
			json += "\"id\": 1,";
			json += "\"registration\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetRegistration()) + " ,";
			json += "\"ignore_registration_rotation\": false,"; // TODO: Add card type here

			// Broadcast robot-specific values
			json += "\"entry_point\":" + ParseEigenToString<Eigen::Matrix<double, 3, 1, Eigen::DontAlign>>(_robot->GetEntryPointPosVectorImageCoord()) + ",";
			json += "\"treatment_to_tip_offset\":" + to_string(_robot->GetProbe()._treatmentToTip) + ",";
			json += "\"canula_to_treatment_offset\":" + to_string(_robot->GetProbe()._cannulaToTreatment) + ",";
			json += "\"robot_to_entry_offset\":" + to_string(_robot->GetProbe()._robotToEntry) + ",";
			json += "\"treatment_to_robot_at_home\":" + to_string(_robot->GetProbe()._robotToTreatmentAtHome) + ",";
			json += "\"target_point\":" + ParseEigenToString<Eigen::Matrix<double, 3, 1, Eigen::DontAlign>>(_robot->GetTargetPointPosVectorImageCoord()) + ",";
			json += "\"current_position\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetCurrentPositionImageCoord()) + ",";
			json += "\"desired_position\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetReachableTargetImageCoord());
			json += "}";

			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			response->write(SimpleWeb::StatusCode::success_ok, json, header);
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed GET '/api/targeting' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** PUT /api/targeting/registration
	server.resource["/api/targeting/registration"]["PUT"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{
			// Get the request body
			picojson::value request_body;
			std::string err = picojson::parse(request_body, request->content);
			if (!err.empty())
			{
				cerr << err << endl;
			}

			// Record requests to the log
			Logger &log = Logger::GetInstance();
			log.Log("Started PUT '/api/targeting/registration' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			picojson::value registrationVector = request_body.get("registration");

			Eigen::Matrix<double, 4, 4, Eigen::DontAlign> new_registration;
			new_registration << stof(registrationVector.get(0).to_str()),
				stof(registrationVector.get(1).to_str()), stof(registrationVector.get(2).to_str()), stof(registrationVector.get(3).to_str()),
				stof(registrationVector.get(4).to_str()), stof(registrationVector.get(5).to_str()), stof(registrationVector.get(6).to_str()), stof(registrationVector.get(7).to_str()),
				stof(registrationVector.get(8).to_str()), stof(registrationVector.get(9).to_str()), stof(registrationVector.get(10).to_str()), stof(registrationVector.get(11).to_str()),
				stof(registrationVector.get(12).to_str()), stof(registrationVector.get(13).to_str()), stof(registrationVector.get(14).to_str()), stof(registrationVector.get(15).to_str());
			_robot->SetRegistration(new_registration);

			string json = "{";
			json += "\"id\": 1,";
			json += "\"registration\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetRegistration()) + " ,";
			json += "\"ignore_registration_rotation\": false,"; // TODO: Add card type here

			// Broadcast robot-specific values
			json += "\"entry_point\":" + ParseEigenToString<Eigen::Matrix<double, 3, 1, Eigen::DontAlign>>(_robot->GetEntryPointPosVectorImageCoord()) + ",";
			json += "\"treatment_to_tip_offset\":" + to_string(_robot->GetProbe()._treatmentToTip) + ",";
			json += "\"canula_to_treatment_offset\":" + to_string(_robot->GetProbe()._cannulaToTreatment) + ",";
			json += "\"robot_to_entry_offset\":" + to_string(_robot->GetProbe()._robotToEntry) + ",";
			json += "\"treatment_to_robot_at_home\":" + to_string(_robot->GetProbe()._robotToTreatmentAtHome) + ",";
			json += "\"target_point\":" + ParseEigenToString<Eigen::Matrix<double, 3, 1, Eigen::DontAlign>>(_robot->GetTargetPointPosVectorImageCoord()) + ",";
			json += "\"current_position\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetCurrentPositionImageCoord()) + ",";
			json += "\"desired_position\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetReachableTargetImageCoord());
			json += "}";

			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			response->write(SimpleWeb::StatusCode::success_ok, json, header);
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed PUT '/api/targeting/registration' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** PUT /api/targeting/current
	server.resource["/api/targeting/current"]["PUT"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{
			// Get the request body
			picojson::value request_body;
			std::string err = picojson::parse(request_body, request->content);
			if (!err.empty())
			{
				cerr << err << endl;
			}

			// Record requests to the log
			log.Log("Started PUT '/api/targeting/current' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			picojson::value currentVector = request_body.get("current_position");
			Eigen::Matrix<double, 4, 4, Eigen::DontAlign> current_pose_image_coord;
			current_pose_image_coord << stof(currentVector.get(0).to_str()), stof(currentVector.get(1).to_str()), stof(currentVector.get(2).to_str()), stof(currentVector.get(3).to_str()),
				stof(currentVector.get(4).to_str()), stof(currentVector.get(5).to_str()), stof(currentVector.get(6).to_str()), stof(currentVector.get(7).to_str()),
				stof(currentVector.get(8).to_str()), stof(currentVector.get(9).to_str()), stof(currentVector.get(10).to_str()), stof(currentVector.get(11).to_str()),
				stof(currentVector.get(12).to_str()), stof(currentVector.get(13).to_str()), stof(currentVector.get(14).to_str()), stof(currentVector.get(15).to_str());
			// Maybe deprecated in the future
			_robot->SetCurrentPoseImageCoord(current_pose_image_coord);

			string json = "{";
			json += "\"id\": 1,";
			json += "\"registration\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetRegistration()) + " ,";
			json += "\"ignore_registration_rotation\": false,"; // TODO: Add card type here

			// Broadcast robot-specific values
			json += "\"entry_point\":" + ParseEigenToString<Eigen::Matrix<double, 3, 1, Eigen::DontAlign>>(_robot->GetEntryPointPosVectorImageCoord()) + ",";
			json += "\"treatment_to_tip_offset\":" + to_string(_robot->GetProbe()._treatmentToTip) + ",";
			json += "\"canula_to_treatment_offset\":" + to_string(_robot->GetProbe()._cannulaToTreatment) + ",";
			json += "\"robot_to_entry_offset\":" + to_string(_robot->GetProbe()._robotToEntry) + ",";
			json += "\"treatment_to_robot_at_home\":" + to_string(_robot->GetProbe()._robotToTreatmentAtHome) + ",";
			json += "\"target_point\":" + ParseEigenToString<Eigen::Matrix<double, 3, 1, Eigen::DontAlign>>(_robot->GetTargetPointPosVectorImageCoord()) + ",";
			json += "\"current_position\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetCurrentPositionImageCoord()) + ",";
			json += "\"desired_position\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetReachableTargetImageCoord());
			json += "}";

			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			response->write(SimpleWeb::StatusCode::success_ok, json, header);
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed PUT '/api/targeting/current' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** PUT /api/targeting/target
	server.resource["/api/targeting/target"]["PUT"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{
			// Get the request body
			picojson::value request_body;
			std::string err = picojson::parse(request_body, request->content);
			if (!err.empty())
			{
				cerr << err << endl;
			}

			// Record requests to the log
			log.Log("Started PUT '/api/targeting/target' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			// TODO: replace registration vector with current vector
			picojson::value entryPointVector = request_body.get("entry_point");
			_robot->SetEntryPointPosVectorImageCoord((Eigen::Matrix<double, 3, 1, Eigen::DontAlign>() << stof(entryPointVector.get(0).to_str()), stof(entryPointVector.get(1).to_str()), stof(entryPointVector.get(2).to_str())).finished());
			Probe probe;
			probe._treatmentToTip = stof(request_body.get("treatment_to_tip_offset").to_str());
			probe._cannulaToTreatment = stof(request_body.get("canula_to_treatment_offset").to_str());
			probe._robotToEntry = stof(request_body.get("robot_to_entry_offset").to_str());
			probe._robotToTreatmentAtHome = stof(request_body.get("treatment_to_robot_at_home").to_str());
			_robot->SetProbeSpecs(probe);
			picojson::value targetPointVector = request_body.get("target_point");
			_robot->SetTargetPointPosVectorImageCoord((Eigen::Matrix<double, 3, 1, Eigen::DontAlign>() << stof(targetPointVector.get(0).to_str()), stof(targetPointVector.get(1).to_str()), stof(targetPointVector.get(2).to_str())).finished());
			string json = "{";
			json += "\"id\": 1,";
			json += "\"registration\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetRegistration()) + " ,";
			json += "\"ignore_registration_rotation\": false,"; // TODO: Add card type here

			// Broadcast robot-specific values
			json += "\"entry_point\":" + ParseEigenToString<Eigen::Matrix<double, 3, 1, Eigen::DontAlign>>(_robot->GetEntryPointPosVectorImageCoord()) + ",";
			json += "\"treatment_to_tip_offset\":" + to_string(_robot->GetProbe()._treatmentToTip) + ",";
			json += "\"canula_to_treatment_offset\":" + to_string(_robot->GetProbe()._cannulaToTreatment) + ",";
			json += "\"robot_to_entry_offset\":" + to_string(_robot->GetProbe()._robotToEntry) + ",";
			json += "\"treatment_to_robot_at_home\":" + to_string(_robot->GetProbe()._robotToTreatmentAtHome) + ",";
			json += "\"target_point\":" + ParseEigenToString<Eigen::Matrix<double, 3, 1, Eigen::DontAlign>>(_robot->GetTargetPointPosVectorImageCoord()) + ",";
			json += "\"current_position\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetCurrentPositionImageCoord()) + ",";
			json += "\"desired_position\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetReachableTargetImageCoord());
			json += "}";

			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			response->write(SimpleWeb::StatusCode::success_ok, json, header);
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed PUT '/api/targeting/target' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** GET /api/communications
	server.resource["/api/communications"]["GET"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{

			// Set the appropriate headers for this request
			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			log.Log("Started GET '/api/communications' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			string json = "[";
			for (unsigned int i = 0; i < _igtList.size(); i++)
			{
				if (i == 0)
				{
					json += "{";
				}
				else
				{
					json += ",{";
				}

				string keepAlive = "\"no\"";
				if (_igtList[i]->keepAlive)
				{
					keepAlive = "\"yes\"";
				}

				json += "\"id\":" + to_string(1) + ",";
				json += "\"type\": \"openigt\" ,";
				json += "\"port\":" + to_string(_igtList[i]->port) + ",";
				json += "\"keepAlive\":" + keepAlive + ",";

				string status = (_igtList[i]->socket.IsNotNull() && _igtList[i]->clientSocketConnected != 0 ? "\"Connected\"" : "\"Listening...\"");
				string currentState = "\"" + _robot->GetCurrentState() + "\"";
				json += "\"currentState\":" + currentState + ",";
				json += "\"status\":" + status;
				json += "}";
			}
			json += "]";

			response->write(SimpleWeb::StatusCode::success_ok, json, header);
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed GET '/api/communications' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** PUT /api/communications/disconnect/:port
	server.resource["/api/communications/disconnect"]["PUT"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{

			// Set the appropriate headers for this request
			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			// Record requests to the log
			log.Log("Started PUT '/api/communication/disconnect' for " + request->remote_endpoint_address(), logger::DEBUG, false);
			// Get the request body
			picojson::value request_body;
			std::string err = picojson::parse(request_body, request->content);
			if (!err.empty())
			{
				cerr << err << endl;
			}

			string json = "[";
			int desired_disconnect_port = stoi(request_body.get("desired_disconnect_port").to_str());

			for (unsigned int i = 0; i < _igtList.size(); i++)
			{
				// Only disconnect the desired port
				if (_igtList[i]->port == desired_disconnect_port)
				{
					_igtList[i]->DisconnectSocket();
				}

				if (i == 0)
				{
					json += "{";
				}
				else
				{
					json += ",{";
				}

				string keepAlive = "no";
				if (_igtList[i]->keepAlive)
				{
					keepAlive = "yes";
				}

				json += "\"id\":" + to_string(1) + ",";
				json += "\"type\": \"openigt\" ,";
				json += "\"port\":" + to_string(_igtList[i]->port) + ",";
				json += "\"keepAlive\":" + keepAlive + ",";

				string status = (_igtList[i]->socket.IsNotNull() && _igtList[i]->clientSocketConnected != 0 ? "\"Connected\"" : "\"Listening...\"");
				string currentState = "\"" + _robot->GetCurrentState() + "\"";
				json += "\"currentState\":" + currentState + ",";
				json += "\"status\":" + status;
				json += "}";
			}

			json += "]";

			response->write(SimpleWeb::StatusCode::success_ok, json, header);
		}

		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed PUT '/api/communication/disconnect' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** PUT /api/communications/keepAlive/:port
	server.resource["/api/communications/keepAlive"]["PUT"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{

			// Set the appropriate headers for this request
			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			// Record requests to the log
			log.Log("Started PUT '/api/communication/keepAlive' for " + request->remote_endpoint_address(), logger::DEBUG, false);
			// Get the request body
			picojson::value request_body;
			std::string err = picojson::parse(request_body, request->content);
			if (!err.empty())
			{
				cerr << err << endl;
			}

			string json = "[";
			int desired_keep_alive_port = stoi(request_body.get("desired_keep_alive_port").to_str());
			string shouldKeepAlive = request_body.get("should_keep_alive").to_str();

			for (unsigned int i = 0; i < _igtList.size(); i++)
			{
				// Only disconnect the desired port
				if (_igtList[i]->port == desired_keep_alive_port)
				{
					if (shouldKeepAlive.compare("yes") == 0)
					{
						_igtList[i]->keepAliveTimer.tic();
						_igtList[i]->keepAlive = true;
					}
					else
					{
						_igtList[i]->keepAlive = false;
					}
				}

				if (i == 0)
				{
					json += "{";
				}
				else
				{
					json += ",{";
				}

				string keepAlive = "no";
				if (_igtList[i]->keepAlive)
				{
					keepAlive = "yes";
				}

				json += "\"id\":" + to_string(1) + ",";
				json += "\"type\": \"openigt\" ,";
				json += "\"port\":" + to_string(_igtList[i]->port) + ",";
				json += "\"keepAlive\":" + keepAlive + ",";

				string status = (_igtList[i]->socket.IsNotNull() && _igtList[i]->clientSocketConnected != 0 ? "\"Connected\"" : "\"Listening...\"");
				string currentState = "\"" + _robot->GetCurrentState() + "\"";
				json += "\"currentState\":" + currentState + ",";
				json += "\"status\":" + status;
				json += "}";
			}

			json += "]";

			response->write(SimpleWeb::StatusCode::success_ok, json, header);
		}

		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed PUT '/api/communication/keepAlive' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** GET /api/status
	server.resource["/api/status"]["GET"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{

			// Set the appropriate headers for this request
			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			// Record requests to the log
			log.Log("Started GET '/api/status' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			vector<string> robot_modes_list = _robot->GetRobotModesList();
			string modes = "[";
			for (unsigned int i = 0; i < robot_modes_list.size(); i++)
			{
				if (i == 0)
				{
					modes += "\"" + robot_modes_list[i] + "\"";
				}
				else
				{
					modes += ",\"" + robot_modes_list[i] + "\"";
				}
			}
			modes += "]";

			// Put together the get request and send result to the front end
			string json = "{ \"id\": 1, \"footpedal\":" + to_string(_robot->_fpga_util->IsFootPedalPressed()) + ", \"available_modes\":" + modes + ", \"estop\": false, \"desired_mode\": \"" + _robot->_mode + "\", \"name\": \" neurorobot \", \"slots_status\": [] }";
			response->write(SimpleWeb::StatusCode::success_ok, json, header);
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed GET '/api/status' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** PUT /api/mode
	server.resource["/api/mode"]["PUT"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{

			// Set the appropriate headers for this request
			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			// Record requests to the log
			log.Log("Started PUT '/api/mode' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			// Get the request body
			picojson::value request_body;
			std::string err = picojson::parse(request_body, request->content);
			if (!err.empty())
			{
				cerr << err << endl;
			}
			_robot->_mode = request_body.get("desired_mode").to_str();

			vector<string> robot_modes_list = _robot->GetRobotModesList();
			string modes = "[";
			for (unsigned int i = 0; i < robot_modes_list.size(); i++)
			{
				if (i == 0)
				{
					modes += "\"" + robot_modes_list[i] + "\"";
				}
				modes += ",\"" + robot_modes_list[i] + "\"";
			}
			modes += "]";

			// Put together the get request and send result to the front end
			string json = "{ \"id\": 1, \"footpedal\":" + to_string(_robot->_fpga_util->IsFootPedalPressed()) + ", \"available_modes\":" + modes + ", \"estop\": false, \"desired_mode\": \"" + _robot->_mode + "\", \"name\": \" neurorobot \", \"slots_status\": [] }";
			response->write(SimpleWeb::StatusCode::success_ok, json, header);
		}

		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed PUT '/api/mode' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** GET /api/logs
	server.resource["/api/logs"]["GET"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{

			// Set the appropriate headers for this request
			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			// Record requests to the log
			log.Log("Started GET '/api/log' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			// Read the last ten lines from the log
			vector<vector<string>> logLines = log.ReadLogData(20);

			// Put together the get request and send information the to Web GUI front end
			string json = "[";

			for (unsigned int i = 0; i < logLines.size(); i++)
			{
				if (i == 0)
				{
					json += "{\"id\":" + logLines[i][0] + ",\"log\":\"" + logLines[i][1] + "\"}";
				}
				else
				{
					json += ",{\"id\":" + logLines[i][0] + ",\"log\":\"" + logLines[i][1] + "\"}";
				}
			}

			json += "]";

			response->write(SimpleWeb::StatusCode::success_ok, json, header);
		}

		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed GET '/api/log' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** Header for CORS requests
	server.default_resource["OPTIONS"] = [](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		try
		{

			// Set header fields
			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Content-Type", "text/plain");
			header.emplace("Access-Control-Allow-Origin", "*");
			header.emplace("Access-Control-Allow-Methods", "GET, POST, OPTIONS, PUT, DELETE");
			header.emplace("Access-Control-Max-Age", "1728000");
			header.emplace("Access-Control-Allow-Headers", "authorization,content-type");

			response->write(SimpleWeb::StatusCode::success_ok, "", header);
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::client_error_bad_request, e.what());
		}
	};

	// ****************************************************
	server.on_error = [](shared_ptr<HttpServer::Request> /*request*/, const SimpleWeb::error_code & /*ec*/)
	{
		// Handle errors here
	};
}

template <>
CustomWebServer<ProstateRobot>::CustomWebServer(ProstateRobot *robot, vector<OpenIGTLink<ProstateRobot> *> igtList, int port)
{

	// Instantiate the constructor parameters
	_igtList = igtList;
	_robot = robot;
	server.config.port = port;

	// ****************************************************
	// **** POST /api/login
	server.resource["/api/login"]["POST"] = [](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{
			// Get the request body
			picojson::value request_body;
			std::string err = picojson::parse(request_body, request->content);
			if (!err.empty())
			{
				cerr << err << endl;
			}

			// Record requests to the log
			log.Log("Started POST '/api/login' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			if ((request_body.get("username").to_str() == "root") && (request_body.get("password").to_str() == "123456"))
			{
				// Set the appropriate headers for this request
				SimpleWeb::CaseInsensitiveMultimap header;
				header.emplace("Access-Control-Allow-Origin", "*");

				response->write(SimpleWeb::StatusCode::success_ok, "{\"token\":\"SIMPLE_TOKEN_1\"}", header);
			}
			else
			{
				SimpleWeb::CaseInsensitiveMultimap header;
				header.emplace("Access-Control-Allow-Origin", "*");
				response->write(SimpleWeb::StatusCode::client_error_not_found, "Wrong username or password.", header);
			}
		}

		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed POST '/api/login' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** GET /api/robots
	server.resource["/api/robots"]["GET"] = [](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{

			// Set the appropriate headers for this request
			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			// Record requests to the log
			log.Log("Started GET '/api/robots' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			// Prepare the response to be sent to the Web GUI
			response->write(SimpleWeb::StatusCode::success_ok, "[ { \"id\": 1, \"name\": \" NeuroRobot \" }, { \"id\": 2, \"name\": \" ProstateRobot \" } ]", header);
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed GET '/api/robots' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** POST /api/robots
	server.resource["/api/robots"]["POST"] = [](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{
			// Get the request body
			picojson::value request_body;
			std::string err = picojson::parse(request_body, request->content);
			if (!err.empty())
			{
				cerr << err << endl;
			}

			// Record requests to the log
			log.Log("Started POST '/api/robots' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			// If we can successfully instantiate the robot
			// TODO: Implement this
			if (request_body.get("id").to_str() == "1")
			{
				SimpleWeb::CaseInsensitiveMultimap header;
				header.emplace("Access-Control-Allow-Origin", "*");

				response->write(SimpleWeb::StatusCode::success_ok, "{ \"id\": 1, \"name\": \" NeuroRobot \" }", header);
			}
			else
			{
				SimpleWeb::CaseInsensitiveMultimap header;
				header.emplace("Access-Control-Allow-Origin", "*");

				response->write(SimpleWeb::StatusCode::client_error_bad_request, "Invalid robot ID.", header);
			}
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed POST '/api/robots' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** GET /api/motors
	server.resource["/api/motors"]["GET"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{
			// Record requests to the log
			log.Log("Started GET '/api/motors' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			// Put together motor data as JSON for processing
			vector<Motor *> motors = _robot->ListMotors();
			string json = "[";
			for (uint i = 0; i < motors.size(); i++)
			{
				if (i == 0)
				{
					json += "{";
				}
				else
				{
					json += ",{";
				}
				json += "\"slot\":" + to_string(motors[i]->_cardID) + ",";
				json += "\"name\": \"" + motors[i]->_name + "\",";
				json += "\"type\": \"none\","; // TODO: Add card type here
				json += "\"enabled\":" + to_string(motors[i]->_enabled) + ",";
				json += "\"current_ticks\":" + to_string(motors[i]->GetEncoderPositionTicks()) + ",";
				json += "\"desired_ticks\":" + to_string(motors[i]->_setpoint) + ",";
				json += "\"min_ticks\":" + to_string(motors[i]->GetMinTicks()) + ",";
				json += "\"max_ticks\":" + to_string(motors[i]->GetMaxTicks()) + ",";
				if (motors[i]->_unit == "degree")
				{
					json += "\"conversion_factor\":" + to_string(motors[i]->_ticksPerUnit * DEGREE_TO_RADIAN) + ",";
				}
				else
				{
					json += "\"conversion_factor\":" + to_string(motors[i]->_ticksPerUnit) + ",";
				}
				json += "\"unit\": \"" + motors[i]->_unit + "\",";
				json += "\"limits\": [{\"name\": \"Upper\", \"value\":" + to_string(motors[i]->IsLimit()) + "}],";
				json += "\"homed\":" + to_string(motors[i]->_homed) + ",";
				json += "\"homing\":" + to_string(motors[i]->_homing) + ",";
				json += "\"faults\": []";
				json += "}";
			}
			json += "]";

			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			response->write(SimpleWeb::StatusCode::success_ok, json, header);
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed GET '/api/motors' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** GET /api/motors/restore
	server.resource["/api/motors/restore"]["PUT"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{
			// Get the request body
			picojson::value request_body;
			std::string err = picojson::parse(request_body, request->content);
			if (!err.empty())
			{
				cerr << err << endl;
			}

			// Record requests to the log
			log.Log("Started GET '/api/motors/restore' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			int request_size = request_body.get<picojson::array>().size();
			for (int i = 0; i < request_size; i++)
			{
				int motorID = stoi(request_body.get(i).get("slot").to_str());
				int newCurrent = stoi(request_body.get(i).get("current_ticks").to_str());
				bool newHomedValue = stoi(request_body.get(i).get("homed").to_str());

				Motor *motor = _robot->GetMotor(motorID);
				motor->SetEncoderOffsetByTicks(newCurrent);
				motor->_homed = newHomedValue;
			}

			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");
			response->write(SimpleWeb::StatusCode::success_ok, "{}", header);
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed GET '/api/motors/restore' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	//**** PUT /api/motors/:slot
	server.resource["/api/motors"]["PUT"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{
			// Record requests to the log
			log.Log("Started PUT '/api/motors' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			// Process the requests
			picojson::value request_body;
			std::string err = picojson::parse(request_body, request->content);
			if (!err.empty())
			{
				cerr << err << endl;
			}

			log.Log("Started PUT '/api/motors' for " + request_body.to_str(), logger::DEBUG, false);

			// The slot determined from the match in the URL
			int _current_slot = stoi(request_body.get("slot").to_str());

			// Extract data from the request
			Motor *motor = _robot->GetMotor(_current_slot);
			motor->_setpoint = stoi(request_body.get("desired_ticks").to_str());
			motor->SetHome(request_body.get("homed").to_str());
			motor->_homing = (request_body.get("homing").to_str() == "true");
			motor->_enabled = (request_body.get("enabled").to_str() == "true");

			string json = "{";
			json += "\"slot\":" + to_string(motor->_cardID) + ",";
			json += "\"name\": \"" + motor->_name + "\",";
			json += "\"type\": \"none\","; // TODO: Add card type here
			json += "\"enabled\":" + to_string(motor->_enabled) + ",";
			json += "\"current_ticks\":" + to_string(motor->GetEncoderPositionTicks()) + ",";
			json += "\"desired_ticks\":" + to_string(motor->_setpoint) + ",";
			json += "\"min_ticks\":" + to_string(motor->GetMinTicks()) + ",";
			json += "\"max_ticks\":" + to_string(motor->GetMaxTicks()) + ",";
			if (motor->_unit == "degree")
			{
				json += "\"conversion_factor\":" + to_string(motor->_ticksPerUnit * DEGREE_TO_RADIAN) + ",";
			}
			else
			{
				json += "\"conversion_factor\":" + to_string(motor->_ticksPerUnit) + ",";
			}
			json += "\"units\": \"" + motor->_unit + "\",";
			json += "\"limits\": [{\"name\": \"Upper\", \"value\":" + to_string(motor->IsLimit()) + "}],";
			json += "\"homed\":" + to_string(motor->_homed) + ",";
			json += "\"homing\":" + to_string(motor->_homing) + ",";
			json += "\"faults\": []";
			json += "}";

			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			response->write(SimpleWeb::StatusCode::success_ok, json, header);

			_robot->RunAxisSetpointValidator();
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed PUT '/api/motors' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** GET /api/targetting
	server.resource["/api/targeting"]["GET"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{
			// Record requests to the log
			log.Log("Started GET '/api/targeting' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			// Put together motor data as JSON for processing
			string json = "{";
			json += "\"id\": 1,";
			json += "\"registration\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetRegistration()) + " ,";
			json += "\"ignore_registration_rotation\": false,"; // TODO: Add card type here

			// Broadcast robot-specific values
			json += "\"needle_length\":" + to_string(_robot->GetBiopsyNeedle()._needleLength) + ",";
			json += "\"target_point\":" + ParseEigenToString<Eigen::Matrix<double, 3, 1, Eigen::DontAlign>>(_robot->GetTargetPointPosVectorImageCoord()) + ",";
			json += "\"current_position\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetCurrentPositionImageCoord()) + ",";
			json += "\"desired_position\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetReachableTargetImageCoord());
			json += "}";

			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			response->write(SimpleWeb::StatusCode::success_ok, json, header);
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed GET '/api/targeting' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** PUT /api/targeting/registration
	server.resource["/api/targeting/registration"]["PUT"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{
			// Get the request body
			picojson::value request_body;
			std::string err = picojson::parse(request_body, request->content);
			if (!err.empty())
			{
				cerr << err << endl;
			}

			// Record requests to the log
			Logger &log = Logger::GetInstance();
			log.Log("Started PUT '/api/targeting/registration' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			picojson::value registrationVector = request_body.get("registration");
			Eigen::Matrix<double, 4, 4, Eigen::DontAlign> new_registration;
			new_registration << stof(registrationVector.get(0).to_str()), stof(registrationVector.get(1).to_str()), stof(registrationVector.get(2).to_str()), stof(registrationVector.get(3).to_str()),
				stof(registrationVector.get(4).to_str()), stof(registrationVector.get(5).to_str()), stof(registrationVector.get(6).to_str()), stof(registrationVector.get(7).to_str()),
				stof(registrationVector.get(8).to_str()), stof(registrationVector.get(9).to_str()), stof(registrationVector.get(10).to_str()), stof(registrationVector.get(11).to_str()),
				stof(registrationVector.get(12).to_str()), stof(registrationVector.get(13).to_str()), stof(registrationVector.get(14).to_str()), stof(registrationVector.get(15).to_str());
			_robot->SetRegistration(new_registration);

			string json = "{";
			json += "\"id\": 1,";
			json += "\"registration\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetRegistration()) + " ,";
			json += "\"ignore_registration_rotation\": false,"; // TODO: Add card type here

			// Broadcast robot-specific values
			json += "\"needle_length\":" + to_string(_robot->GetBiopsyNeedle()._needleLength) + ",";
			json += "\"target_point\":" + ParseEigenToString<Eigen::Matrix<double, 3, 1, Eigen::DontAlign>>(_robot->GetTargetPointPosVectorImageCoord()) + ",";
			json += "\"current_position\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetCurrentPositionImageCoord()) + ",";
			json += "\"desired_position\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetReachableTargetImageCoord());
			json += "}";

			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			response->write(SimpleWeb::StatusCode::success_ok, json, header);
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed PUT '/api/targeting/registration' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** PUT /api/targeting/current
	server.resource["/api/targeting/current"]["PUT"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{
			// Get the request body
			picojson::value request_body;
			std::string err = picojson::parse(request_body, request->content);
			if (!err.empty())
			{
				cerr << err << endl;
			}

			// Record requests to the log
			log.Log("Started PUT '/api/targeting/current' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			picojson::value currentVector = request_body.get("current_position");
			Eigen::Matrix<double, 4, 4, Eigen::DontAlign> current_pose_image_coord;
			current_pose_image_coord << stof(currentVector.get(0).to_str()), stof(currentVector.get(1).to_str()), stof(currentVector.get(2).to_str()), stof(currentVector.get(3).to_str()),
				stof(currentVector.get(4).to_str()), stof(currentVector.get(5).to_str()), stof(currentVector.get(6).to_str()), stof(currentVector.get(7).to_str()),
				stof(currentVector.get(8).to_str()), stof(currentVector.get(9).to_str()), stof(currentVector.get(10).to_str()), stof(currentVector.get(11).to_str()),
				stof(currentVector.get(12).to_str()), stof(currentVector.get(13).to_str()), stof(currentVector.get(14).to_str()), stof(currentVector.get(15).to_str());
			_robot->SetCurrentPoseImageCoord(current_pose_image_coord);
			string json = "{";
			json += "\"id\": 1,";
			json += "\"registration\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetRegistration()) + " ,";
			json += "\"ignore_registration_rotation\": false,"; // TODO: Add card type here

			// Broadcast robot-specific values
			json += "\"needle_length\":" + to_string(_robot->GetBiopsyNeedle()._needleLength) + ",";
			json += "\"target_point\":" + ParseEigenToString<Eigen::Matrix<double, 3, 1, Eigen::DontAlign>>(_robot->GetTargetPointPosVectorImageCoord()) + ",";
			json += "\"current_position\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetCurrentPositionImageCoord()) + ",";
			json += "\"desired_position\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetReachableTargetImageCoord());
			json += "}";

			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			response->write(SimpleWeb::StatusCode::success_ok, json, header);
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed PUT '/api/targeting/current' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** PUT /api/targeting/target
	server.resource["/api/targeting/target"]["PUT"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{
			// Get the request body
			picojson::value request_body;
			std::string err = picojson::parse(request_body, request->content);
			if (!err.empty())
			{
				cerr << err << endl;
			}

			// Record requests to the log
			log.Log("Started PUT '/api/targeting/target' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			// TODO: replace registration vector with current vector
			// Update needle length and current tip position
			_robot->SetNeedleLength(stof(request_body.get("needle_length").to_str()));

			picojson::value targetPointVector = request_body.get("target_point");
			_robot->SetTargetPointPosVectorImageCoord((Eigen::Matrix<double, 3, 1, Eigen::DontAlign>() << stof(targetPointVector.get(0).to_str()), stof(targetPointVector.get(1).to_str()), stof(targetPointVector.get(2).to_str())).finished());

			string json = "{";
			json += "\"id\": 1,";
			json += "\"registration\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetRegistration()) + " ,";
			json += "\"ignore_registration_rotation\": false,"; // TODO: Add card type here

			// Broadcast robot-specific values
			json += "\"needle_length\":" + to_string(_robot->GetBiopsyNeedle()._needleLength) + ",";
			json += "\"target_point\":" + ParseEigenToString<Eigen::Matrix<double, 3, 1, Eigen::DontAlign>>(_robot->GetTargetPointPosVectorImageCoord()) + ",";
			json += "\"current_position\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetCurrentPositionImageCoord()) + ",";
			json += "\"desired_position\":" + ParseEigenToString<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>(_robot->GetReachableTargetImageCoord());
			json += "}";

			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			response->write(SimpleWeb::StatusCode::success_ok, json, header);
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed PUT '/api/targeting/target' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** GET /api/communications
	server.resource["/api/communications"]["GET"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{

			// Set the appropriate headers for this request
			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			log.Log("Started GET '/api/communications' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			string json = "[";
			for (unsigned int i = 0; i < _igtList.size(); i++)
			{
				if (i == 0)
				{
					json += "{";
				}
				else
				{
					json += ",{";
				}

				string keepAlive = "\"no\"";
				if (_igtList[i]->keepAlive)
				{
					keepAlive = "\"yes\"";
				}

				json += "\"id\":" + to_string(1) + ",";
				json += "\"type\": \"openigt\" ,";
				json += "\"port\":" + to_string(_igtList[i]->port) + ",";
				json += "\"keepAlive\":" + keepAlive + ",";

				string status = (_igtList[i]->socket.IsNotNull() && _igtList[i]->clientSocketConnected != 0 ? "\"Connected\"" : "\"Listening...\"");
				string currentState = "\"" + _robot->GetCurrentState() + "\"";
				json += "\"currentState\":" + currentState + ",";
				json += "\"status\":" + status;
				json += "}";
			}
			json += "]";

			response->write(SimpleWeb::StatusCode::success_ok, json, header);
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed GET '/api/communications' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** PUT /api/communications/disconnect/:port
	server.resource["/api/communications/disconnect"]["PUT"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{

			// Set the appropriate headers for this request
			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			// Record requests to the log
			log.Log("Started PUT '/api/communication/disconnect' for " + request->remote_endpoint_address(), logger::DEBUG, false);
			// Get the request body
			picojson::value request_body;
			std::string err = picojson::parse(request_body, request->content);
			if (!err.empty())
			{
				cerr << err << endl;
			}

			string json = "[";
			int desired_disconnect_port = stoi(request_body.get("desired_disconnect_port").to_str());

			for (unsigned int i = 0; i < _igtList.size(); i++)
			{
				// Only disconnect the desired port
				if (_igtList[i]->port == desired_disconnect_port)
				{
					_igtList[i]->DisconnectSocket();
				}

				if (i == 0)
				{
					json += "{";
				}
				else
				{
					json += ",{";
				}

				string keepAlive = "no";
				if (_igtList[i]->keepAlive)
				{
					keepAlive = "yes";
				}

				json += "\"id\":" + to_string(1) + ",";
				json += "\"type\": \"openigt\" ,";
				json += "\"port\":" + to_string(_igtList[i]->port) + ",";
				json += "\"keepAlive\":" + keepAlive + ",";

				string status = (_igtList[i]->socket.IsNotNull() && _igtList[i]->clientSocketConnected != 0 ? "\"Connected\"" : "\"Listening...\"");
				string currentState = "\"" + _robot->GetCurrentState() + "\"";
				json += "\"currentState\":" + currentState + ",";
				json += "\"status\":" + status;
				json += "}";
			}

			json += "]";

			response->write(SimpleWeb::StatusCode::success_ok, json, header);
		}

		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed PUT '/api/communication/disconnect' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** PUT /api/communications/keepAlive/:port
	server.resource["/api/communications/keepAlive"]["PUT"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{

			// Set the appropriate headers for this request
			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			// Record requests to the log
			log.Log("Started PUT '/api/communication/keepAlive' for " + request->remote_endpoint_address(), logger::DEBUG, false);
			// Get the request body
			picojson::value request_body;
			std::string err = picojson::parse(request_body, request->content);
			if (!err.empty())
			{
				cerr << err << endl;
			}

			string json = "[";
			int desired_keep_alive_port = stoi(request_body.get("desired_keep_alive_port").to_str());
			string shouldKeepAlive = request_body.get("should_keep_alive").to_str();

			for (unsigned int i = 0; i < _igtList.size(); i++)
			{
				// Only disconnect the desired port
				if (_igtList[i]->port == desired_keep_alive_port)
				{
					if (shouldKeepAlive.compare("yes") == 0)
					{
						_igtList[i]->keepAliveTimer.tic();
						_igtList[i]->keepAlive = true;
					}
					else
					{
						_igtList[i]->keepAlive = false;
					}
				}

				if (i == 0)
				{
					json += "{";
				}
				else
				{
					json += ",{";
				}

				string keepAlive = "no";
				if (_igtList[i]->keepAlive)
				{
					keepAlive = "yes";
				}

				json += "\"id\":" + to_string(1) + ",";
				json += "\"type\": \"openigt\" ,";
				json += "\"port\":" + to_string(_igtList[i]->port) + ",";
				json += "\"keepAlive\":" + keepAlive + ",";

				string status = (_igtList[i]->socket.IsNotNull() && _igtList[i]->clientSocketConnected != 0 ? "\"Connected\"" : "\"Listening...\"");
				string currentState = "\"" + _robot->GetCurrentState() + "\"";
				json += "\"currentState\":" + currentState + ",";
				json += "\"status\":" + status;
				json += "}";
			}

			json += "]";

			response->write(SimpleWeb::StatusCode::success_ok, json, header);
		}

		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed PUT '/api/communication/keepAlive' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** GET /api/status
	server.resource["/api/status"]["GET"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{

			// Set the appropriate headers for this request
			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			// Record requests to the log
			log.Log("Started GET '/api/status' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			vector<string> robot_modes_list = _robot->GetRobotModesList();
			string modes = "[";
			for (unsigned int i = 0; i < robot_modes_list.size(); i++)
			{
				if (i == 0)
				{
					modes += "\"" + robot_modes_list[i] + "\"";
				}
				else
				{
					modes += ",\"" + robot_modes_list[i] + "\"";
				}
			}
			modes += "]";

			// Put together the get request and send result to the front end
			string json = "{ \"id\": 1, \"footpedal\":" + to_string(_robot->_fpga_util->IsFootPedalPressed()) + ", \"available_modes\":" + modes + ", \"estop\": false, \"desired_mode\": \"" + _robot->_mode + "\", \"name\": \" neurorobot \", \"slots_status\": [] }";
			response->write(SimpleWeb::StatusCode::success_ok, json, header);
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed GET '/api/status' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** PUT /api/mode
	server.resource["/api/mode"]["PUT"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{

			// Set the appropriate headers for this request
			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			// Record requests to the log
			log.Log("Started PUT '/api/mode' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			// Get the request body
			picojson::value request_body;
			std::string err = picojson::parse(request_body, request->content);
			if (!err.empty())
			{
				cerr << err << endl;
			}

			_robot->_mode = request_body.get("desired_mode").to_str();

			//		  if(_robot->_mode == "HOME"){
			//			_robot->UnHomeRobot();
			//		  }

			vector<string> robot_modes_list = _robot->GetRobotModesList();
			string modes = "[";
			for (unsigned int i = 0; i < robot_modes_list.size(); i++)
			{
				if (i == 0)
				{
					modes += "\"" + robot_modes_list[i] + "\"";
				}
				modes += ",\"" + robot_modes_list[i] + "\"";
			}
			modes += "]";

			// Put together the get request and send result to the front end
			string json = "{ \"id\": 1, \"footpedal\":" + to_string(_robot->_fpga_util->IsFootPedalPressed()) + ", \"available_modes\":" + modes + ", \"estop\": false, \"desired_mode\": \"" + _robot->_mode + "\", \"name\": \" neurorobot \", \"slots_status\": [] }";
			response->write(SimpleWeb::StatusCode::success_ok, json, header);
		}

		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed PUT '/api/mode' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** GET /api/logs
	server.resource["/api/logs"]["GET"] = [&](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		Logger &log = Logger::GetInstance();
		try
		{

			// Set the appropriate headers for this request
			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Access-Control-Allow-Origin", "*");

			// Record requests to the log
			log.Log("Started GET '/api/log' for " + request->remote_endpoint_address(), logger::DEBUG, false);

			// Read the last ten lines from the log
			vector<vector<string>> logLines = log.ReadLogData(20);

			// Put together the get request and send information the to Web GUI front end
			string json = "[";

			for (unsigned int i = 0; i < logLines.size(); i++)
			{
				if (i == 0)
				{
					json += "{\"id\":" + logLines[i][0] + ",\"log\":\"" + logLines[i][1] + "\"}";
				}
				else
				{
					json += ",{\"id\":" + logLines[i][0] + ",\"log\":\"" + logLines[i][1] + "\"}";
				}
			}

			json += "]";

			response->write(SimpleWeb::StatusCode::success_ok, json, header);
		}

		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
			log.Log("Failed GET '/api/log' for " + request->remote_endpoint_address() + " error: " + e.what(), logger::ERROR, false);
		}
	};

	// ****************************************************
	// **** Header for CORS requests
	server.default_resource["OPTIONS"] = [](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
	{
		try
		{

			// Set header fields
			SimpleWeb::CaseInsensitiveMultimap header;
			header.emplace("Content-Type", "text/plain");
			header.emplace("Access-Control-Allow-Origin", "*");
			header.emplace("Access-Control-Allow-Methods", "GET, POST, OPTIONS, PUT, DELETE");
			header.emplace("Access-Control-Max-Age", "1728000");
			header.emplace("Access-Control-Allow-Headers", "authorization,content-type");

			response->write(SimpleWeb::StatusCode::success_ok, "", header);
		}
		catch (const exception &e)
		{
			response->write(SimpleWeb::StatusCode::client_error_bad_request, e.what());
		}
	};

	// ****************************************************
	server.on_error = [](shared_ptr<HttpServer::Request> /*request*/, const SimpleWeb::error_code & /*ec*/)
	{
		// Handle errors here
	};
}

template <typename TRobot>
void CustomWebServer<TRobot>::ParseStringToArray(const string &input, char delim, vector<float> *result)
{
	std::stringstream ss(input);
	std::string item;
	while (std::getline(ss, item, delim))
	{
		result->push_back(atoi(item.c_str()));
	}
}

template <typename TRobot>
template <class T>
string CustomWebServer<TRobot>::ParseEigenToString(T input)
{
	string result = "[";
	for (int i = 0; i < input.rows(); i++)
	{
		for (int j = 0; j < input.cols(); j++)
		{
			std::ostringstream ss;
			if (i == 0 && j == 0)
			{
				ss << input(i, j);
			}
			else
			{
				result += ",";
				ss << input(i, j);
			}

			string s(ss.str());
			result += s;
		}
	}

	result += "]";
	return result;
}

template <typename TRobot>
void *CustomWebServer<TRobot>::runServer(void *webServer)
{
	CustomWebServer<TRobot> *theServer = (CustomWebServer<TRobot> *)webServer;
	theServer->server.start();
	return NULL;
}

template class CustomWebServer<ProstateRobot>;
template class CustomWebServer<NeuroRobot>;
