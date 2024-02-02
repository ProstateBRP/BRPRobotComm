//============================================================================
// Name        : ProstateRobot.h
// Author      : Produced in the WPI AIM Lab
// Description : This file defines the methods and parameters of the BRP Robot
//============================================================================

#ifndef _ProstateRobot_HPP_
#define _ProstateRobot_HPP_

#include "../Robot.hpp"

#include <string>

#include "Packets.hpp"
#include "FPGA_Utilities.hpp"
#include "ForceSensor.hpp"
#include "ProstateKinematics.hpp"
#include "ProstateRobotMotors.hpp"
#include "ProstateRobotSensors.hpp"
#include "ProstateRobotMotionController.hpp"
#include "ProstateRobotKinematicsController.hpp"
#include "ProstateRobotManualMode.hpp"
#include "ProstateRobotClinicalMode.hpp"
#include "ProstateRobotAutomatedHomingMode.hpp"
#include "ProstateRobotFrequencySweepMode.hpp"
#include "ProstateRobotActiveSteeringMode.hpp"
#include <queue>

using namespace std;

class ProstateRobot : public Robot
{
public:
	//================ Constructor ================
	ProstateRobot(Packets *packets, FPGA_Utilities *fpga_util, int loopRate);
	~ProstateRobot();
	//================ Parameters =================
	// Outgoing and Incoming Packets via SPI
	Packets *_packets;
	// The FPGA Utility object was included to allow for control over LEDs
	FPGA_Utilities *_fpga_util;
	// Timer
	Timer _timer;
	int _loopRate;
	// Prostate Robot modes
	ProstateRobotModes kRobotModes;
	ProstateRobotConstants kConstants;
	ProstateRobotStates kStateNames;
	//=================== Prostate Robot Kinematics ==================
	// This class contains the forward and inverse kinematics for this robot
	ProstateKinematics prostate_kinematics_;
	BiopsyNeedle biopsy_needle; // Defines the specific needle structure on the robot
	ProstateRobotMotors motors_;
	std::queue<Motor*> motors_queue;
	ProstateRobotSensors sensors_;
	ProstateRobotMotionController motion_controller_;
	ProstateRobotKinematicsController kinematics_controller_;
	//======================= Robot Modes =======================
	shared_ptr<ProstateRobotClinicalMode> clinical_mode{nullptr};
	shared_ptr<ProstateRobotManualMode> manual_mode{nullptr};
	shared_ptr<ProstateRobotAutomatedHomingMode> automated_homing_mode{nullptr};
	shared_ptr<ProstateRobotFrequencySweepMode> frequency_sweep_mode{nullptr};
	shared_ptr<ProstateRobotActiveSteeringMode> active_steering_mode{nullptr};
	vector<weak_ptr<ProstateRobotModeBase>> robot_modes_list_;
	/* ========= Prostate Robot Specific Methods ========= */
	const BiopsyNeedle GetBiopsyNeedle() { return biopsy_needle; }
	Eigen::Matrix<double, 4, 4, Eigen::DontAlign> GetCurrentPositionKinematicImageCoord();
	void LogInverseKinematicsOutput();
	void SetNeedleLength(const double &needle_length);
	void UpdateNeedleTipPose();
	void UpdateTargetImageCoord(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &);
	void InitSynMotion(ProstateRobotMotorSetpointMap);
	ProstateRobotMotorSetpointMap RunAxisSetpointValidator();
	ProstateRobotForwardKinematicsInput GetAllMotorsSetpointPositionUnit();
	ProstateRobotForwardKinematicsInput GetAllMotorsCurrentPositionUnit();
	/* ========= Abstract Methods ========= */
	virtual bool CheckForStalls();
	virtual bool isHomed();
	virtual void Reset();
	virtual void RunInverseKinematics();
	virtual void SetRegistration(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &);
	virtual void SetTargetImageCoord(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &);
	virtual void SetTargetPointPosVectorImageCoord(const Eigen::Matrix<double, 3, 1, Eigen::DontAlign> &);
	virtual void StopRobot();
	virtual void Update();
	virtual void UpdateBaseToTarget();
	virtual void ZeroRobot();
	virtual Motor *GetMotor(int cardID);
	virtual vector<Motor *> ListMotors();
	virtual Eigen::Matrix<double, 3, 1, Eigen::DontAlign> GetTargetPointPosVectorImageCoord();
	virtual Eigen::Matrix<double, 4, 4, Eigen::DontAlign> ConvertFromImagerToRobotBase(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &);
	virtual Eigen::Matrix<double, 4, 4, Eigen::DontAlign> ConvertFromRobotBaseToImager(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &);
	virtual vector<string> GetRobotModesList();
};

#endif /* _ProstateRobot_HPP_ */
